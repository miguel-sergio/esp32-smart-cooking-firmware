#include "comms_task.h"
#include "ota_task.h"
#include "app_types.h"
#include "provisioning.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_ota_ops.h"

#include "mbedtls/platform_util.h"
#include "esp_timer.h"
#include "cJSON.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"

static const char *TAG = "comms";

/* ── Task constants ─────────────────────────────────────────────────────── */

#define COMMS_STACK_WORDS   (6144u / sizeof(StackType_t))
#define COMMS_PRIORITY      3
#define COMMS_CORE          0

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRIES    PROV_WIFI_MAX_RETRIES   /* 3 retries — FR-07 */

/* MQTT topics (FR-08, FR-09, FR-10) */
#define TOPIC_TELEMETRY     "cooking/telemetry"
#define TOPIC_FAULT         "cooking/fault"
#define TOPIC_CMD           "cooking/cmd"
#define TOPIC_OTA           "cooking/ota"

/* Telemetry publish intervals — FR-08 */
#define TELEM_INTERVAL_ACTIVE_MS   1000u   /*  1 Hz — PREHEAT / COOKING / DONE / ERROR */
#define TELEM_INTERVAL_IDLE_MS    10000u   /* 0.1 Hz — IDLE */

/* ── Module state ───────────────────────────────────────────────────────── */

static comms_task_config_t s_cfg;
static EventGroupHandle_t  s_wifi_eg;
static uint8_t             s_retry_count = 0u;

/* Pending OTA URL — set when URL arrives during an active cooking cycle.
 * Dispatched to ota_url_q once state reaches IDLE / DONE / ERROR.
 * Protected by s_ota_mutex (written from MQTT task, read from comms_task). */
static char              s_pending_ota_url[OTA_URL_MAX_LEN];
static bool              s_ota_pending = false;
static SemaphoreHandle_t s_ota_mutex   = NULL;

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

    /* Zeroize password copies from RAM now that the driver has consumed them */
    mbedtls_platform_zeroize(pass, PROV_PASS_MAX_LEN);
    mbedtls_platform_zeroize(wifi_cfg.sta.password, sizeof(wifi_cfg.sta.password));

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

/* ── CMD payload parser (FR-09) ─────────────────────────────────────────── */

static void handle_cmd_payload(const char *data, int data_len) {
    /* cJSON_ParseWithLength requires a null-terminated string or at least a
     * buffer large enough — MQTT event data is NOT null-terminated, so copy
     * into a stack buffer first. */
    char buf[256];
    if (data_len <= 0 || data_len >= (int)sizeof(buf)) {
        ESP_LOGW(TAG, "CMD: payload length %d out of range — discarded", data_len);
        return;
    }
    memcpy(buf, data, (size_t)data_len);
    buf[data_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        ESP_LOGW(TAG, "CMD: JSON parse failed — payload discarded");
        return;
    }

    cJSON *cmd_item = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    if (!cJSON_IsString(cmd_item) || cmd_item->valuestring == NULL) {
        ESP_LOGW(TAG, "CMD: missing or invalid \"cmd\" field — discarded");
        cJSON_Delete(root);
        return;
    }

    const char *cmd_str = cmd_item->valuestring;
    mqtt_cmd_t  cmd     = { .type = CMD_STOP, .profile_id = 0u };
    bool        valid   = true;

    if (strcmp(cmd_str, "START") == 0) {
        cmd.type = CMD_START;
        cJSON *pid = cJSON_GetObjectItemCaseSensitive(root, "profile_id");
        if (cJSON_IsNumber(pid)) {
            int v = (int)pid->valuedouble;
            cmd.profile_id = (v >= 0 && v <= 255) ? (uint8_t)v : 0u;
        }
    } else if (strcmp(cmd_str, "STOP") == 0) {
        cmd.type = CMD_STOP;
    } else if (strcmp(cmd_str, "ESTOP") == 0) {
        cmd.type = CMD_ESTOP;
    } else if (strcmp(cmd_str, "RESET") == 0) {
        cmd.type = CMD_RESET;
    } else {
        ESP_LOGW(TAG, "CMD: unknown command \"%s\" — discarded", cmd_str);
        valid = false;
    }

    if (valid) {
        if (xQueueSend(s_cfg.cmd_q, &cmd, 0) != pdTRUE) {
            ESP_LOGW(TAG, "CMD: cmd_q full — command dropped");
        } else {
            ESP_LOGI(TAG, "CMD: dispatched %s (profile=%u)",
                     cmd_str, (unsigned)cmd.profile_id);
        }
    }

    cJSON_Delete(root);
}

/* ── MQTT event handler ─────────────────────────────────────────────────── */

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        /* Subscribe to command and OTA topics on every (re)connect — FR-09, FR-10 */
        esp_mqtt_client_subscribe(ev->client, TOPIC_CMD, 1);
        esp_mqtt_client_subscribe(ev->client, TOPIC_OTA, 1);
        ESP_LOGI(TAG, "MQTT subscribed: " TOPIC_CMD ", " TOPIC_OTA);

        /* Boot self-check: Wi-Fi + MQTT connected → mark firmware valid (FR-10, NFR-05).
         * Only acts when running from a freshly flashed OTA partition pending
         * verification; no-op on normal boot or after already marked valid. */
        {
            const esp_partition_t *running = esp_ota_get_running_partition();
            esp_ota_img_states_t   ota_state;
            if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
                ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
                esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "OTA: new firmware validated — rollback cancelled");
                } else {
                    ESP_LOGE(TAG, "OTA: failed to validate new firmware: %s", esp_err_to_name(err));
                }
            }
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    case MQTT_EVENT_DATA:
        if (ev->topic_len > 0 && ev->data_len > 0) {
            if (strncmp(ev->topic, TOPIC_CMD,
                        (size_t)ev->topic_len) == 0) {
                handle_cmd_payload(ev->data, ev->data_len);
            } else if (strncmp(ev->topic, TOPIC_OTA,
                               (size_t)ev->topic_len) == 0) {
                /* Store URL; dispatch to ota_task when state allows (FR-10) */
                int copy_len = ev->data_len < (int)(OTA_URL_MAX_LEN - 1u)
                               ? ev->data_len
                               : (int)(OTA_URL_MAX_LEN - 1u);
                xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
                memcpy(s_pending_ota_url, ev->data, (size_t)copy_len);
                s_pending_ota_url[copy_len] = '\0';
                s_ota_pending = true;
                xSemaphoreGive(s_ota_mutex);
                ESP_LOGI(TAG, "OTA: URL queued — %.*s", copy_len, ev->data);
            }
        }
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

    /* ── Loop — telemetry, fault alerts, command dispatch ──────────────── */
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    system_state_t   last_state    = { .state = COOKING_STATE_IDLE,
                                       .fault = FAULT_NONE };
    bool             have_state    = false;
    bool             fault_alerted = false; /* true after fault published; reset on exit */
    uint32_t         last_telem_ms = 0u;

#if CONFIG_SMART_COOKING_STABILITY_TEST
    TickType_t last_diag = 0u; /* 30 s diagnostic anchor */
#endif

    for (;;) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());

        /* Block for up to 100 ms waiting for the first item, then drain the
         * rest of the queue non-blocking.  This guarantees the loop runs at
         * most 10 times/s, keeping CPU 0 free for IDLE and other tasks. */
        system_state_t incoming;
        if (xQueueReceive(s_cfg.state_q, &incoming,
                          pdMS_TO_TICKS(100u)) == pdTRUE) {
            last_state = incoming;
            have_state = true;
            /* Drain any additional items queued since we started blocking */
            while (xQueueReceive(s_cfg.state_q, &incoming, 0) == pdTRUE) {
                last_state = incoming;
            }
        }

        if (!have_state || mqtt_client == NULL) {
            continue;
        }

        /* ── OTA deferral: dispatch pending URL when cycle is not active ── */
        {
            char  url_copy[OTA_URL_MAX_LEN];
            bool  pending = false;

            xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
            if (s_ota_pending) {
                pending = true;
                strlcpy(url_copy, s_pending_ota_url, OTA_URL_MAX_LEN);
            }
            xSemaphoreGive(s_ota_mutex);

            if (pending) {
                cooking_state_t cs = last_state.state;
                if (cs == COOKING_STATE_IDLE ||
                    cs == COOKING_STATE_DONE  ||
                    cs == COOKING_STATE_ERROR) {
                    configASSERT(s_cfg.ota_url_q != NULL);
                    BaseType_t send_ok = xQueueSend(s_cfg.ota_url_q, url_copy, 0);
                    if (send_ok == pdTRUE) {
                        xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
                        s_ota_pending = false;
                        xSemaphoreGive(s_ota_mutex);
                        ESP_LOGI(TAG, "OTA: URL dispatched to ota_task");
                    } else {
                        ESP_LOGW(TAG, "OTA: deferred URL dispatch failed (queue full?)");
                    }
                } else {
                    ESP_LOGD(TAG, "OTA: deferred — cycle active (%s)",
                             state_name(last_state.state));
                }
            }
        }

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);

        /* ── Fault alert: publish once on transition into ERROR (FR-08) ─── */
        if (last_state.state == COOKING_STATE_ERROR && !fault_alerted) {
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "{\"fault\":\"%s\",\"last_temp\":%.1f,"
                     "\"timestamp_ms\":%" PRIu32 "}",
                     fault_name(last_state.fault),
                     last_state.temperature,
                     now_ms);
            esp_mqtt_client_publish(mqtt_client, TOPIC_FAULT, buf, 0, 1, 0);
            ESP_LOGW(TAG, "Fault alert published: %s", buf);
            fault_alerted = true;
        } else if (last_state.state != COOKING_STATE_ERROR) {
            fault_alerted = false;  /* reset so next ERROR triggers a new alert */
        }

        /* ── Telemetry: rate-limited JSON publish (FR-08) ───────────────── */
        uint32_t interval = (last_state.state == COOKING_STATE_IDLE)
                            ? TELEM_INTERVAL_IDLE_MS
                            : TELEM_INTERVAL_ACTIVE_MS;

        if ((now_ms - last_telem_ms) >= interval) {
            last_telem_ms = now_ms;
            char buf[192];
            snprintf(buf, sizeof(buf),
                     "{\"temp\":%.1f,\"humidity\":%.1f,\"phase\":\"%s\","
                     "\"motor_duty\":%d,\"profile\":%u,\"fault\":\"%s\"}",
                     last_state.temperature,
                     last_state.humidity,
                     state_name(last_state.state),
                     (int)last_state.motor_duty_pct,
                     (unsigned)last_state.active_profile,
                     fault_name(last_state.fault));
            esp_mqtt_client_publish(mqtt_client, TOPIC_TELEMETRY, buf, 0, 1, 0);
            ESP_LOGD(TAG, "Telemetry: %s", buf);
        }

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

    s_ota_mutex = xSemaphoreCreateMutex();
    configASSERT(s_ota_mutex != NULL);

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
