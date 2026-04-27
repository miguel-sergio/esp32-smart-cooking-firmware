#include "thermal_task.h"
#include "app_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "bme280.h"

static const char *TAG = "thermal";

/* ── Task constants ─────────────────────────────────────────────────────── */

#define THERM_STACK_WORDS   (4096u / sizeof(StackType_t))
#define THERM_PRIORITY      4
#define THERM_CORE          1

#define ACTIVE_PERIOD_MS    1000u   /* 1 Hz  — enabled (heating active)    */
#define IDLE_PERIOD_MS      10000u  /* 0.1 Hz — disabled (IDLE state)      */

#define HYSTERESIS_DEG      1.0f    /* bang-bang dead band ±1 °C           */

/* WDT chunk: half the configured WDT timeout in ticks.
 * vTaskDelayUntil is called in these chunks so the WDT is fed well within
 * the deadline regardless of the sleep period length. */
#define WDT_CHUNK_TICKS     pdMS_TO_TICKS((CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000u) / 2u)

/* BME280 valid output ranges */
#define TEMP_MIN_DEG       -40.0f
#define TEMP_MAX_DEG        85.0f
#define HUM_MIN_PCT          0.0f
#define HUM_MAX_PCT        100.0f

/* ── Module state ───────────────────────────────────────────────────────── */

static thermal_task_config_t s_cfg;

/* ── Helpers ────────────────────────────────────────────────────────────── */

static void relay_set(bool on) {
    gpio_set_level((gpio_num_t)s_cfg.relay_gpio, on ? 1 : 0);
}

/* ── Task ───────────────────────────────────────────────────────────────── */

static void thermal_task(void *arg) {
    (void)arg;

    /* ── Hardware init (inside task, before loop) ───────────────────────── */
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << s_cfg.relay_gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));
    relay_set(false);

    bme280_dev_t bme;
    ESP_ERROR_CHECK(bme280_init(s_cfg.i2c_bus, s_cfg.bme280_addr, &bme));
    ESP_LOGI(TAG, "BME280 initialised");

    /* ── Loop state ─────────────────────────────────────────────────────── */
    thermal_cmd_t current_cmd  = { .enabled = false, .setpoint = 0.0f };
    bool          relay_on     = false;
    TickType_t    period_ms    = IDLE_PERIOD_MS;
    TickType_t    last_wake    = xTaskGetTickCount();
#if CONFIG_SMART_COOKING_STABILITY_TEST
    TickType_t    last_diag    = 0u; /* 30 s diagnostic anchor */
#endif

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    for (;;) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        /* ── 1. Drain thermal_q — keep most recent command ──────────────── */
        thermal_cmd_t cmd;
        bool period_changed = false;
        while (xQueueReceive(s_cfg.thermal_q, &cmd, 0) == pdTRUE) {
            TickType_t new_period = cmd.enabled ? ACTIVE_PERIOD_MS : IDLE_PERIOD_MS;
            if (new_period != period_ms) {
                period_ms      = new_period;
                period_changed = true;
            }
            current_cmd = cmd;
        }
        /* Reset wake anchor so the new period starts from now, not the past */
        if (period_changed) {
            last_wake = xTaskGetTickCount();
        }

        /* ── 2. Read BME280 ─────────────────────────────────────────────── */
        bme280_data_t data;
        esp_err_t     ret   = bme280_read(&bme, &data);
        bool          valid = false;

        if (ret == ESP_OK) {
            valid = (data.temperature >= TEMP_MIN_DEG &&
                     data.temperature <= TEMP_MAX_DEG &&
                     data.humidity    >= HUM_MIN_PCT  &&
                     data.humidity    <= HUM_MAX_PCT);
            if (!valid) {
                ESP_LOGW(TAG, "BME280 out of range: T=%.1f H=%.1f",
                         data.temperature, data.humidity);
            }
        } else {
            ESP_LOGW(TAG, "bme280_read failed: %s", esp_err_to_name(ret));
        }

        /* ── 3. Bang-bang relay control ─────────────────────────────────── */
        if (!current_cmd.enabled) {
            if (relay_on) {
                relay_on = false;
                relay_set(false);
                ESP_LOGD(TAG, "Relay OFF (disabled)");
            }
        } else if (valid) {
            float sp = current_cmd.setpoint;
            if (data.temperature < (sp - HYSTERESIS_DEG)) {
                if (!relay_on) {
                    relay_on = true;
                    relay_set(true);
                    ESP_LOGD(TAG, "Relay ON  (%.1f°C < %.1f°C)",
                             data.temperature, sp - HYSTERESIS_DEG);
                }
            } else if (data.temperature > (sp + HYSTERESIS_DEG)) {
                if (relay_on) {
                    relay_on = false;
                    relay_set(false);
                    ESP_LOGD(TAG, "Relay OFF (%.1f°C > %.1f°C)",
                             data.temperature, sp + HYSTERESIS_DEG);
                }
            }
            /* within hysteresis band: hold current relay state */
        } else {
            if (relay_on) {
                relay_on = false;
                relay_set(false);
                ESP_LOGW(TAG, "Relay OFF (invalid sensor reading)");
            }
        }

        /* ── 4. Publish reading to control_task ─────────────────────────── */
        temp_reading_t reading = {
            .temperature  = valid ? data.temperature : 0.0f,
            .humidity     = valid ? data.humidity    : 0.0f,
            .valid        = valid,
            .timestamp_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount()),
        };
        if (xQueueSend(s_cfg.temp_q, &reading, 0) != pdTRUE) {
            ESP_LOGW(TAG, "temp_q full — reading dropped");
        }

        if (valid) {
            ESP_LOGI(TAG, "T: %.1f°C  H: %.1f%%  relay: %s",
                     data.temperature, data.humidity, relay_on ? "ON" : "OFF");
        }

#if CONFIG_SMART_COOKING_STABILITY_TEST
        /* ── Periodic diagnostics (every 30 s) ──────────────────────────── */
        if ((xTaskGetTickCount() - last_diag) >= pdMS_TO_TICKS(30000u)) {
            ESP_LOGI(TAG, "DIAG stack_hwm=%u words",
                     uxTaskGetStackHighWaterMark(NULL));
            last_diag = xTaskGetTickCount();
        }
#endif

        /* ── 5. Sleep for period_ms, waking immediately on incoming command.
         *
         * xQueuePeek is used as the blocking primitive with a timeout of
         * WDT_CHUNK_TICKS. This means the task unblocks as soon as a command
         * is queued (not just at the next chunk boundary), eliminating the
         * up-to-WDT_CHUNK_TICKS latency of the previous vTaskDelayUntil
         * approach that caused SENSOR_TIMEOUT false-positives on PREHEAT entry.
         *
         * WDT is reset on every iteration (timed out or command received) so
         * feeding is unconditional. The cadence anchor advances by exactly
         * period_ms on the normal path to prevent drift accumulation.        */
        {
            TickType_t deadline = last_wake + pdMS_TO_TICKS(period_ms);
            bool       early    = false;

            for (;;) {
                TickType_t now       = xTaskGetTickCount();
                TickType_t remaining = (TickType_t)(deadline - now);

                /* Period elapsed or tick counter wrapped past deadline */
                if (remaining == 0u || remaining > pdMS_TO_TICKS(period_ms)) {
                    break;
                }

                TickType_t    wait = (remaining < WDT_CHUNK_TICKS) ? remaining : WDT_CHUNK_TICKS;
                thermal_cmd_t peek;
                if (xQueuePeek(s_cfg.thermal_q, &peek, wait) == pdTRUE) {
                    early = true;
                    break;
                }
                ESP_ERROR_CHECK(esp_task_wdt_reset());
            }

            last_wake = early ? xTaskGetTickCount()
                              : (last_wake + pdMS_TO_TICKS(period_ms));
        }
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void thermal_task_start(const thermal_task_config_t *cfg) {
    configASSERT(cfg != NULL);
    configASSERT(cfg->temp_q != NULL);
    configASSERT(cfg->i2c_bus != NULL);
    configASSERT(GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)cfg->relay_gpio));
    s_cfg = *cfg;

    BaseType_t ret = xTaskCreatePinnedToCore(
        thermal_task,
        "thermal",
        THERM_STACK_WORDS,
        NULL,
        THERM_PRIORITY,
        NULL,
        THERM_CORE
    );
    configASSERT(ret == pdPASS);
}
