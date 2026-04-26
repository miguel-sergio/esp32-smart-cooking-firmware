#include "motor_task.h"
#include "app_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "motor";

/* ── Task constants ─────────────────────────────────────────────────────── */

#define MOTOR_STACK_WORDS   (4096u / sizeof(StackType_t))
#define MOTOR_PRIORITY      4
#define MOTOR_CORE          1

#define RAMP_STEP_MS        10u     /* delay between ramp steps (ms)        */
#define RAMP_UP_MS          2000u   /* 0% → target in 2 s (200 steps)       */
#define RAMP_DOWN_MS        1000u   /* target → 0% in 1 s (100 steps)       */

/* ── Module state ───────────────────────────────────────────────────────── */

static motor_task_config_t s_cfg;

/* ── Helpers ────────────────────────────────────────────────────────────── */

/**
 * Ramp from current HAL speed to target_pct over ramp_ms.
 * Drains motor_q each step — returns true if a new command arrived
 * (most recent stored in *next_cmd), false if ramp completed without interruption.
 * Returns false immediately on drv8833_set_speed() failure (error is logged).
 */
static bool ramp_to(drv8833_dev_t *drv, int8_t target_pct, uint32_t ramp_ms, motor_cmd_t *next_cmd) {
    int8_t  start = drv->cur_speed[s_cfg.ch];
    int32_t steps = (int32_t)(ramp_ms / RAMP_STEP_MS);

    if (steps <= 0) {
        esp_err_t err = drv8833_set_speed(drv, s_cfg.ch, target_pct);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ramp_to: drv8833_set_speed failed: %s", esp_err_to_name(err));
        }
        return false;
    }

    for (int32_t i = 1; i <= steps; i++) {
        /* Drain queue each step — keep only the most recent command */
        motor_cmd_t tmp;
        bool got_cmd = false;
        while (xQueueReceive(s_cfg.motor_q, &tmp, 0) == pdTRUE) {
            *next_cmd = tmp;
            got_cmd = true;
        }
        if (got_cmd) {
            return true;
        }

        int8_t step_speed = (int8_t)(start + (int32_t)(target_pct - start) * i / steps);
        esp_err_t err = drv8833_set_speed(drv, s_cfg.ch, step_speed);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ramp_to: drv8833_set_speed failed: %s", esp_err_to_name(err));
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(RAMP_STEP_MS));
    }

    /* Final step: ensure exact target */
    esp_err_t err = drv8833_set_speed(drv, s_cfg.ch, target_pct);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ramp_to: final drv8833_set_speed failed: %s", esp_err_to_name(err));
    }
    return false;
}

/* ── Task ───────────────────────────────────────────────────────────────── */

static void motor_task(void *arg) {
    (void)arg;

    /* ── Hardware init (inside task, before loop) ───────────────────────── */
    drv8833_dev_t drv;
    ESP_ERROR_CHECK(drv8833_init(&s_cfg.drv_cfg, &drv));
    ESP_LOGI(TAG, "DRV8833 initialised");

    int8_t current_duty = 0;

    for (;;) {
        /* ── 1. Wait for a command ──────────────────────────────────────── */
        motor_cmd_t cmd;
        xQueueReceive(s_cfg.motor_q, &cmd, portMAX_DELAY);

        /* Drain — keep the most recent command */
        motor_cmd_t newer;
        while (xQueueReceive(s_cfg.motor_q, &newer, 0) == pdTRUE) {
            cmd = newer;
        }

        /* ── 2. Brake request: treat as immediate stop/coast, no ramp ─── */
        if (cmd.brake) {
            drv8833_set_speed(&drv, s_cfg.ch, 0);
            current_duty = 0;
            ESP_LOGI(TAG, "Motor stopped immediately");

        /* ── 3. Stop requested ─────────────────────────────────────────── */
        } else if (cmd.duty_pct == 0) {
            if (current_duty != 0) {
                bool interrupted = ramp_to(&drv, 0, RAMP_DOWN_MS, &cmd);
                current_duty = drv.cur_speed[s_cfg.ch];
                if (!interrupted && current_duty == 0) {
                    ESP_LOGI(TAG, "Motor stopped");
                } else if (interrupted) {
                    if (xQueueSendToFront(s_cfg.motor_q, &cmd, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "motor_q full — could not re-queue interrupted command");
                    }
                }
            }

        /* ── 4. Ramp up to target ──────────────────────────────────────── */
        } else if (current_duty == 0) {
            /* Starting from rest — ramp up */
            int8_t target = cmd.duty_pct;
            bool interrupted = ramp_to(&drv, target, RAMP_UP_MS, &cmd);
            current_duty = drv.cur_speed[s_cfg.ch];
            if (!interrupted && current_duty == target) {
                ESP_LOGI(TAG, "Motor running at %d%%", current_duty);
            } else if (interrupted) {
                /* New command arrived mid-ramp — re-queue it */
                if (xQueueSendToFront(s_cfg.motor_q, &cmd, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "motor_q full — could not re-queue interrupted command");
                }
            }
        } else {
            /* Already running — jump directly to new duty (no ramp) */
            drv8833_set_speed(&drv, s_cfg.ch, cmd.duty_pct);
            current_duty = cmd.duty_pct;
            ESP_LOGI(TAG, "Motor duty → %d%%", current_duty);
        }
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void motor_task_start(const motor_task_config_t *cfg) {
    configASSERT(cfg != NULL);
    configASSERT(cfg->motor_q != NULL);
    s_cfg = *cfg;

    BaseType_t ret = xTaskCreatePinnedToCore(
        motor_task,
        "motor",
        MOTOR_STACK_WORDS,
        NULL,
        MOTOR_PRIORITY,
        NULL,
        MOTOR_CORE
    );
    configASSERT(ret == pdPASS);
}
