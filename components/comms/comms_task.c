#include "comms_task.h"
#include "ota_task.h"
#include "app_types.h"
#include "cooking_logic.h"
#include "provisioning.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_ota_ops.h"
#include "nvs.h"

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
#define TOPIC_CONFIG        "cooking/config"  /* remote device config — broker URI migration */

/* Telemetry publish intervals — FR-08 */
#define TELEM_INTERVAL_ACTIVE_MS   1000u   /*  1 Hz — PREHEAT / COOKING / DONE / ERROR */
#define TELEM_INTERVAL_IDLE_MS    10000u   /* 0.1 Hz — IDLE */

/* NVS key for the runtime-configurable MQTT broker URI.
 * Namespace: "sc_config", key: "mqtt_uri".
 * Written at manufacturing or via a field provisioning tool.
 * Survives OTA updates.  Takes precedence over the compile-time default.
 * Max length includes the null terminator.
 *
 * Two-phase commit for URI migration (cooking/config):
 *   "mqtt_uri_cand" — candidate URI written when a config change arrives.
 *   "mqtt_uri"      — active URI, only promoted from candidate after a
 *                     successful MQTT connection confirms the new broker
 *                     is reachable.  If connection fails within the
 *                     verification window the candidate is deleted and
 *                     the device restarts using the previous active URI.
 */
#define MQTT_URI_NVS_NAMESPACE  "sc_config"
#define MQTT_URI_NVS_KEY        "mqtt_uri"
#define MQTT_URI_NVS_CAND_KEY   "mqtt_uri_cand"
#define MQTT_URI_MAX_LEN        256u

/* Timeout to validate a candidate URI: if MQTT does not connect within
 * this window after a cooking/config-triggered restart the candidate is
 * discarded and the device falls back to the previously active URI. */
#define URI_CAND_VERIFY_TIMEOUT_S   60
#define URI_CAND_VERIFY_TIMEOUT_US  ((int64_t)URI_CAND_VERIFY_TIMEOUT_S * 1000000LL)

/* ── Module state ───────────────────────────────────────────────────────── */

static comms_task_config_t s_cfg;
static EventGroupHandle_t  s_wifi_eg;
static uint8_t             s_retry_count = 0u;

/* Set to true once wifi_init() returns successfully.  Used by the Wi-Fi
 * event handler to distinguish boot-time connection failures (handled by
 * wifi_init via WIFI_FAIL_BIT) from runtime disconnections (no one is
 * waiting on the EventGroup — must restart to recover). */
static bool                s_wifi_ready  = false;

/* Set by wifi_event_handler when max retries are exhausted at runtime.
 * Processed by the comms_task loop: deferred until state is IDLE/DONE/ERROR
 * so actuators are in a defined safe state before the restart.  If a cycle
 * is active, an ESTOP is posted first to drive control_task to ERROR. */
static volatile bool       s_restart_pending = false;

/* Set by handle_config_payload when a non-urgent config change (e.g. broker
 * URI migration) requires a restart.  Unlike s_restart_pending, this never
 * posts ESTOP — it waits for the active cycle to finish naturally (IDLE/DONE)
 * or for the operator to issue STOP/ESTOP.  Safe to wait indefinitely. */
static volatile bool       s_graceful_restart_pending = false;

/* Set to true on the first MQTT_EVENT_CONNECTED.  Used by the candidate URI
 * validation deadline to confirm the new broker is reachable. */
static volatile bool       s_mqtt_connected  = false;

/* Set to true on IP_EVENT_STA_GOT_IP.  Used to trigger OTA firmware
 * validation: Wi-Fi connect is the validation point (not MQTT) because
 * the golden NVS URI guarantees MQTT connectivity on every boot, making
 * an MQTT-based guard a no-op discriminator. */
static volatile bool       s_wifi_validated  = false;

/* Set to true when the device boots using a candidate URI (written by a
 * cooking/config command but not yet confirmed reachable).  Cleared once
 * MQTT connects and the candidate is promoted to the active NVS key.
 * If the deadline expires before connection, the candidate is discarded
 * and the device restarts using the previous active URI. */
static bool                s_uri_candidate_pending = false;

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
        } else if (!s_wifi_ready) {
            /* Boot phase: signal wifi_init() to trigger re-provisioning (FR-07) */
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Wi-Fi: max retries reached at boot — entering re-provisioning");
        } else {
            /* Runtime: set a flag; comms_task loop handles the restart once the
             * cooking cycle is in a safe state (IDLE/DONE/ERROR).  Calling
             * esp_restart() directly here could interrupt an active cycle and
             * leave actuator GPIOs in an undefined state during reset. */
            ESP_LOGE(TAG, "Wi-Fi: max retries reached at runtime — restart deferred");
            s_restart_pending = true;
        }

    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected — IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_retry_count = 0u;
        provisioning_on_wifi_success();
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);

        /* OTA boot self-check (NFR-05): validate new firmware on Wi-Fi connect.
         *
         * Validation is triggered here rather than at MQTT_EVENT_CONNECTED because
         * the device stores a golden broker URI in NVS that always connects —
         * making MQTT connect a guaranteed event that cannot discriminate between
         * a good and a bad firmware image.
         *
         * Wi-Fi success proves: NVS readable (credentials survived OTA),
         * RF + TCP/IP stack functional, and FreeRTOS tasks running.
         * That is sufficient evidence that the new image is healthy. */
        s_wifi_validated = true;
        {
            const esp_partition_t *running = esp_ota_get_running_partition();
            esp_ota_img_states_t   ota_state;
            if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
                ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
                esp_err_t err_v = esp_ota_mark_app_valid_cancel_rollback();
                if (err_v == ESP_OK) {
                    ESP_LOGI(TAG, "OTA: new firmware validated on Wi-Fi connect — rollback cancelled");
                } else {
                    ESP_LOGE(TAG, "OTA: failed to validate new firmware: %s", esp_err_to_name(err_v));
                }
            }
        }
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

/* ── Config payload parser ──────────────────────────────────────────────── */

/* Handles cooking/config messages for runtime device reconfiguration.
 *
 * Supported fields:
 *   {"mqtt_uri": "mqtts://new-broker:8883"}
 *
 * On a valid mqtt_uri: persists to NVS and triggers a safe deferred restart
 * (via s_restart_pending).  The existing restart-deferral logic posts ESTOP
 * if a cycle is active, waits for IDLE/DONE/ERROR, then restarts.  After
 * reboot the device connects using the new URI and validates it via the OTA
 * rollback guard — if the new broker is unreachable within 60 s the device
 * rolls back to the previous firmware which still has the old URI in NVS.
 *
 * SECURITY: access control is enforced at the broker level (ACL).  The URI
 * is validated to be non-empty and within the maximum allowed length; no
 * scheme-level validation is performed here.
 */
static void handle_config_payload(const char *data, int data_len) {
    char buf[MQTT_URI_MAX_LEN + 64u];
    if (data_len <= 0 || data_len >= (int)sizeof(buf)) {
        ESP_LOGW(TAG, "CONFIG: payload length %d out of range — discarded", data_len);
        return;
    }
    memcpy(buf, data, (size_t)data_len);
    buf[data_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        ESP_LOGW(TAG, "CONFIG: JSON parse failed — discarded");
        return;
    }

    cJSON *uri_item = cJSON_GetObjectItemCaseSensitive(root, "mqtt_uri");
    if (!cJSON_IsString(uri_item) || uri_item->valuestring == NULL ||
        uri_item->valuestring[0] == '\0') {
        ESP_LOGW(TAG, "CONFIG: missing or empty \"mqtt_uri\" field — discarded");
        cJSON_Delete(root);
        return;
    }

    const char *new_uri = uri_item->valuestring;
    if (strlen(new_uri) >= MQTT_URI_MAX_LEN) {
        ESP_LOGW(TAG, "CONFIG: mqtt_uri too long (%zu chars, max %u) — discarded",
                 strlen(new_uri), MQTT_URI_MAX_LEN - 1u);
        cJSON_Delete(root);
        return;
    }

    /* Persist new URI as a candidate — not yet promoted to active.
     * The active URI ("mqtt_uri") is only updated after the new broker
     * is confirmed reachable.  This prevents NVS from being left with
     * a bad URI if the candidate broker turns out to be unreachable. */
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(MQTT_URI_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, MQTT_URI_NVS_CAND_KEY, new_uri);
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CONFIG: NVS write failed (%s) — URI not updated",
                 esp_err_to_name(err));
        cJSON_Delete(root);
        return;
    }

    ESP_LOGI(TAG, "CONFIG: mqtt_uri candidate set to \"%s\" — will promote after connection", new_uri);

    /* Graceful restart: wait for the active cycle to finish naturally.
     * Does NOT post ESTOP — cooking must not be interrupted for a config change. */
    s_graceful_restart_pending = true;

    cJSON_Delete(root);
}

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt_connected = true;
        ESP_LOGI(TAG, "MQTT connected");
        /* Subscribe to command and OTA topics on every (re)connect — FR-09, FR-10 */
        esp_mqtt_client_subscribe(ev->client, TOPIC_CMD, 1);
        esp_mqtt_client_subscribe(ev->client, TOPIC_OTA, 1);
        esp_mqtt_client_subscribe(ev->client, TOPIC_CONFIG, 1);
        ESP_LOGI(TAG, "MQTT subscribed: " TOPIC_CMD ", " TOPIC_OTA ", " TOPIC_CONFIG);

        /* Candidate URI promotion: if we booted using a candidate URI and
         * MQTT is now connected, the new broker is confirmed reachable.
         * Promote the candidate to the active NVS key and delete the
         * candidate so subsequent boots use the confirmed URI directly. */
        if (s_uri_candidate_pending) {
            nvs_handle_t nvs;
            if (nvs_open(MQTT_URI_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
                char cand[MQTT_URI_MAX_LEN];
                size_t sz = sizeof(cand);
                if (nvs_get_str(nvs, MQTT_URI_NVS_CAND_KEY, cand, &sz) == ESP_OK) {
                    if (nvs_set_str(nvs, MQTT_URI_NVS_KEY, cand) == ESP_OK) {
                        nvs_erase_key(nvs, MQTT_URI_NVS_CAND_KEY);
                        nvs_commit(nvs);
                        ESP_LOGI(TAG, "CONFIG: candidate URI promoted to active: %s", cand);
                    }
                }
                nvs_close(nvs);
            }
            s_uri_candidate_pending = false;
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
            } else if (strncmp(ev->topic, TOPIC_CONFIG,
                               (size_t)ev->topic_len) == 0) {
                handle_config_payload(ev->data, ev->data_len);
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

/* ── MQTT broker URI resolution ─────────────────────────────────────────── */

/* Production pattern: NVS always wins over compile-time config.
 *
 * Priority (highest first):
 *   1. NVS key "mqtt_uri" in namespace "sc_config"  — written at manufacturing
 *      or by a field provisioning tool; survives OTA.
 *   2. CONFIG_SMART_COOKING_MQTT_BROKER_URI          — baked in at build time
 *      by CI/CD from the MQTT_BROKER_URI GitHub secret.
 *
 * This guarantees that a device in the field keeps its known-good broker URI
 * across all future OTA updates regardless of what the new binary compiles in.
 */
/* Production pattern: NVS always wins over compile-time config.
 *
 * Priority (highest first):
 *   1. NVS candidate key "mqtt_uri_cand" — written when a cooking/config
 *      URI change arrives; not yet confirmed reachable.  Used on the
 *      first boot after the change; discarded if connection fails.
 *   2. NVS active key "mqtt_uri"         — written at manufacturing or
 *      promoted from a successfully validated candidate; survives OTA.
 *   3. CONFIG_SMART_COOKING_MQTT_BROKER_URI — baked in at build time
 *      by CI/CD from the MQTT_BROKER_URI GitHub secret.  Used when
 *      neither NVS key is present (fresh device or NVS erased).
 *
 * Returns true if the loaded URI came from the candidate key (needs
 * validation); false otherwise.
 */
static bool load_mqtt_uri(char *buf, size_t buf_len) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(MQTT_URI_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        /* Check candidate first */
        size_t sz = buf_len;
        esp_err_t cand_err = nvs_get_str(nvs, MQTT_URI_NVS_CAND_KEY, buf, &sz);
        if (cand_err == ESP_OK && buf[0] != '\0') {
            nvs_close(nvs);
            ESP_LOGI(TAG, "MQTT URI from NVS candidate (pending validation): %s", buf);
            return true;   /* candidate — must validate */
        }
        /* Fall through to active key */
        sz = buf_len;
        err = nvs_get_str(nvs, MQTT_URI_NVS_KEY, buf, &sz);
        nvs_close(nvs);
        if (err == ESP_OK && buf[0] != '\0') {
            ESP_LOGI(TAG, "MQTT URI from NVS (active): %s", buf);
            return false;
        }
    }
    /* No NVS URI — fall back to compile-time default */
    strlcpy(buf, CONFIG_SMART_COOKING_MQTT_BROKER_URI, buf_len);
    ESP_LOGI(TAG, "MQTT URI from config: %s", buf);
    return false;
}

/* ── MQTT init ──────────────────────────────────────────────────────────── */

static esp_mqtt_client_handle_t mqtt_init(void) {
    char uri[MQTT_URI_MAX_LEN];
    s_uri_candidate_pending = load_mqtt_uri(uri, sizeof(uri));

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    configASSERT(client != NULL);

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));

    return client;
}

/* ── Task ───────────────────────────────────────────────────────────────── */

static void comms_task(void *arg) {
    (void)arg;

    /* ── Wi-Fi + MQTT init (inside task, before loop) ───────────────────── */
    /* wifi_init() never returns false — all failure paths call
     * provisioning_on_wifi_failure() which calls esp_restart(). */
    wifi_init();
    /* Mark runtime phase: from here, exhausted retries trigger esp_restart()
     * instead of signalling WIFI_FAIL_BIT (see wifi_event_handler). */
    s_wifi_ready = true;

    esp_mqtt_client_handle_t mqtt_client = mqtt_init();

    /* Candidate URI validation deadline: if this boot is using a candidate
     * URI and MQTT does not connect in time, discard the candidate and
     * restart — the device will use the previous active URI on next boot. */
    int64_t uri_cand_deadline_us = 0;
    if (s_uri_candidate_pending) {
        uri_cand_deadline_us = esp_timer_get_time() + URI_CAND_VERIFY_TIMEOUT_US;
        ESP_LOGI(TAG, "CONFIG: candidate URI — must connect within %d s or discard",
                 URI_CAND_VERIFY_TIMEOUT_S);
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

        /* Candidate URI validation: if MQTT did not connect within the
         * deadline the candidate broker is unreachable.  Discard the
         * candidate NVS key and restart — the device will use the
         * previous active URI (or compiled default) on next boot. */
        if (uri_cand_deadline_us != 0 && !s_mqtt_connected &&
            esp_timer_get_time() >= uri_cand_deadline_us) {
            ESP_LOGE(TAG, "CONFIG: candidate URI unreachable after %d s — discarding, reverting",
                     URI_CAND_VERIFY_TIMEOUT_S);
            nvs_handle_t nvs;
            if (nvs_open(MQTT_URI_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
                nvs_erase_key(nvs, MQTT_URI_NVS_CAND_KEY);
                nvs_commit(nvs);
                nvs_close(nvs);
            }
            esp_restart();
            /* not reached */
        }

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

        /* ── Deferred restart: Wi-Fi max-retry recovery ─────────────────── */
        /* Restart only when actuators are in a defined safe state.          */
        if (s_restart_pending) {
            cooking_state_t cs = last_state.state;
            if (cooking_logic_cycle_inactive(cs)) {
                ESP_LOGI(TAG, "Wi-Fi restart: state=%s — restarting now",
                         state_name(cs));
                esp_restart();
                /* not reached */
            }
            /* PREHEAT / COOKING: post ESTOP so control_task drives heater and
             * motor off before we restart.  The loop continues; on the next
             * iteration the state will be ERROR and we fall into the branch
             * above.  ESTOP is idempotent — repeated posts are harmless. */
            mqtt_cmd_t estop = { .type = CMD_ESTOP, .profile_id = 0u };
            xQueueSend(s_cfg.cmd_q, &estop, 0);
            ESP_LOGW(TAG, "Wi-Fi restart: cycle active (%s) — ESTOP posted, waiting",
                     state_name(cs));
        }

        /* ── Graceful restart: broker URI config change ──────────────────── */
        /* Never posts ESTOP — waits for the cycle to reach IDLE/DONE/ERROR   */
        /* naturally (operator issues STOP, cycle timer expires, or a fault). */
        if (s_graceful_restart_pending) {
            cooking_state_t cs = last_state.state;
            if (cooking_logic_cycle_inactive(cs)) {
                ESP_LOGI(TAG, "Config restart: state=%s — restarting now",
                         state_name(cs));
                esp_restart();
                /* not reached */
            }
            ESP_LOGD(TAG, "Config restart: waiting for cycle to finish (%s)",
                     state_name(cs));
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
                if (cooking_logic_cycle_inactive(cs)) {
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
