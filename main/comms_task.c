#include "comms_task.h"
#include "app_types.h"
#include "provisioning.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"

static const char *TAG = "comms";

/* ── Task constants ─────────────────────────────────────────────────────── */

#define COMMS_STACK_WORDS   (4096u / sizeof(StackType_t))
#define COMMS_PRIORITY      3
#define COMMS_CORE          0

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRIES    PROV_WIFI_MAX_RETRIES   /* 3 retries — FR-07 */

/* ── Module state ───────────────────────────────────────────────────────── */

static comms_task_config_t s_cfg;
static EventGroupHandle_t  s_wifi_eg;
static uint8_t             s_retry_count = 0u;

/* ── Wi-Fi event handler ────────────────────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data) {
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_count < WIFI_MAX_RETRIES) {
            s_retry_count++;
            ESP_LOGW(TAG, "Wi-Fi disconnected — retry %u/%u", s_retry_count, WIFI_MAX_RETRIES);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Wi-Fi: max retries reached");
        }

    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected — IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_retry_count = 0u;
        provisioning_on_wifi_success();
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

/* ── Wi-Fi init ─────────────────────────────────────────────────────────── */

static bool wifi_init(void) {
    /* Ensure credentials are in NVS — enters Blufi mode if first boot */
    provisioning_ensure();

    char ssid[PROV_SSID_MAX_LEN];
    char pass[PROV_PASS_MAX_LEN];
    if (!provisioning_load_credentials(ssid, sizeof(ssid),
                                       pass, sizeof(pass))) {
        ESP_LOGE(TAG, "Failed to load Wi-Fi credentials from NVS");
        /* Missing/corrupt credentials — erase/reset and restart into Blufi (FR-07) */
        provisioning_on_wifi_failure();
        /* not reached — provisioning_on_wifi_failure() calls esp_restart() */
        return false;
    }

    s_wifi_eg = xEventGroupCreate();
    configASSERT(s_wifi_eg != NULL);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = { 0 };
    strlcpy((char *)wifi_cfg.sta.ssid,     ssid, sizeof(wifi_cfg.sta.ssid));
    strlcpy((char *)wifi_cfg.sta.password, pass, sizeof(wifi_cfg.sta.password));
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi connecting to SSID: \"%s\"", ssid);
    ESP_LOGI(TAG, "MQTT broker URI configured");

    /* Wait until either connected or all retries exhausted.
     * WIFI_FAIL_BIT is always set after WIFI_MAX_RETRIES disconnects,
     * so this never blocks indefinitely. */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           portMAX_DELAY);

    if ((bits & WIFI_CONNECTED_BIT) != 0u) {
        return true;
    }

    /* All retries exhausted — erase credentials and restart for Blufi (FR-07) */
    provisioning_on_wifi_failure();
    /* not reached — provisioning_on_wifi_failure() calls esp_restart() */
    return false;
}

/* ── MQTT event handler ─────────────────────────────────────────────────── */

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error type: %d", ev->error_handle->error_type);
        break;
    default:
        break;
    }
}

/* ── MQTT init ──────────────────────────────────────────────────────────── */

static esp_mqtt_client_handle_t mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_SMART_COOKING_MQTT_BROKER_URI,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    configASSERT(client != NULL);

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));

    return client;
}

/* ── State name helpers ─────────────────────────────────────────────────── */

static const char *state_name(cooking_state_t s) {
    switch (s) {
    case COOKING_STATE_IDLE:    return "IDLE";
    case COOKING_STATE_PREHEAT: return "PREHEAT";
    case COOKING_STATE_COOKING: return "COOKING";
    case COOKING_STATE_DONE:    return "DONE";
    case COOKING_STATE_ERROR:   return "ERROR";
    default:                    return "UNKNOWN";
    }
}

static const char *fault_name(fault_type_t f) {
    switch (f) {
    case FAULT_NONE:           return "NONE";
    case FAULT_OVERTEMP:       return "OVERTEMP";
    case FAULT_SENSOR_TIMEOUT: return "SENSOR_TIMEOUT";
    case FAULT_ESTOP:          return "ESTOP";
    case FAULT_HEATER_FAIL:    return "HEATER_FAIL";
    default:                   return "UNKNOWN";
    }
}

/* ── Task ───────────────────────────────────────────────────────────────── */

static void comms_task(void *arg) {
    (void)arg;

    /* ── Wi-Fi + MQTT init (inside task, before loop) ───────────────────── */
    bool wifi_ok = wifi_init();

    esp_mqtt_client_handle_t mqtt_client = NULL;
    if (wifi_ok) {
        mqtt_client = mqtt_init();
    }

    /* ── Loop — drain state_q and log ──────────────────────────────────── */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

#if CONFIG_SMART_COOKING_STABILITY_TEST
    TickType_t last_diag = 0u; /* 30 s diagnostic anchor */
#endif

    for (;;) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());

        system_state_t state;
        if (xQueueReceive(s_cfg.state_q, &state, pdMS_TO_TICKS(1000u)) == pdTRUE) {
            ESP_LOGI(TAG, "state=%s  T=%.1f°C  fault=%s",
                     state_name(state.state),
                     state.temperature,
                     fault_name(state.fault));
        }

        /* TODO M4: publish state to MQTT */
        (void)mqtt_client;

#if CONFIG_SMART_COOKING_STABILITY_TEST
        /* ── Periodic diagnostics (every 30 s) ──────────────────────────── */
        if ((xTaskGetTickCount() - last_diag) >= pdMS_TO_TICKS(30000u)) {
            ESP_LOGI(TAG, "DIAG stack_hwm=%u words",
                     uxTaskGetStackHighWaterMark(NULL));
            last_diag = xTaskGetTickCount();
        }
#endif
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void comms_task_start(const comms_task_config_t *cfg) {
    configASSERT(cfg != NULL);
    configASSERT(cfg->state_q != NULL);
    configASSERT(cfg->cmd_q != NULL);
    s_cfg = *cfg;

    BaseType_t ret = xTaskCreatePinnedToCore(
        comms_task,
        "comms",
        COMMS_STACK_WORDS,
        NULL,
        COMMS_PRIORITY,
        NULL,
        COMMS_CORE
    );
    configASSERT(ret == pdPASS);
}
