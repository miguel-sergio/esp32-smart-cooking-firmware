#include "provisioning.h"
#include "provisioning_security.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_blufi_api.h"
#include "esp_blufi.h"

#include "nvs.h"

static const char *TAG = "provisioning";

/* ── Blufi constants ─────────────────────────────────────────────────────── */

#define BLUFI_PROVISIONED_BIT   BIT0
#define PROV_BLE_DEVICE_NAME    "SmartCooking"

/* ── Module state ────────────────────────────────────────────────────────── */

static EventGroupHandle_t s_prov_eg;

static char s_ssid[PROV_SSID_MAX_LEN];
static char s_pass[PROV_PASS_MAX_LEN];
static bool s_got_ssid = false;
static bool s_got_pass = false;

/* ── NVS helpers ─────────────────────────────────────────────────────────── */

static bool nvs_has_credentials(void) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(PROV_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return false;
    }

    size_t ssid_len = 0u;
    size_t pass_len = 0u;
    esp_err_t ssid_err = nvs_get_str(nvs, PROV_NVS_KEY_SSID, NULL, &ssid_len);
    esp_err_t pass_err = nvs_get_str(nvs, PROV_NVS_KEY_PASS, NULL, &pass_len);
    nvs_close(nvs);

    return (ssid_err == ESP_OK && ssid_len > 1u &&
            pass_err == ESP_OK && pass_len > 1u);
}

static bool nvs_save_credentials(const char *ssid, const char *pass) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(PROV_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_str(nvs, PROV_NVS_KEY_SSID, ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS write SSID failed: %s", esp_err_to_name(err));
        nvs_close(nvs);
        return false;
    }

    err = nvs_set_str(nvs, PROV_NVS_KEY_PASS, pass);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS write password failed: %s", esp_err_to_name(err));
        nvs_close(nvs);
        return false;
    }

    err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Credentials saved to NVS (SSID length: %u)", (unsigned)strlen(ssid));
    return true;
}

static bool nvs_erase_credentials(void) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(PROV_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open for erase failed: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_erase_key(nvs, PROV_NVS_KEY_SSID);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "NVS erase SSID failed: %s", esp_err_to_name(err));
        nvs_close(nvs);
        return false;
    }

    err = nvs_erase_key(nvs, PROV_NVS_KEY_PASS);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "NVS erase password failed: %s", esp_err_to_name(err));
        nvs_close(nvs);
        return false;
    }

    err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit after erase failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "NVS credentials erased");
    return true;
}

/* ── BLE advertising ────────────────────────────────────────────────────── */

static esp_ble_adv_data_t s_adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00u,
    .manufacturer_len    = 0u,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0u,
    .p_service_data      = NULL,
    .service_uuid_len    = 0u,
    .p_service_uuid      = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min       = 0x0100u,
    .adv_int_max       = 0x0100u,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ── GAP event handler ───────────────────────────────────────────────────── */

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: {
        esp_err_t err = esp_ble_gap_start_advertising(&s_adv_params);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "BLE start advertising failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "BLE advertising started");
        }
        break;
    }

    case ESP_GAP_BLE_SEC_REQ_EVT: {
        /* Phone requested security — reply with no-bond accept so the
         * SMP exchange completes immediately instead of timing out. */
        esp_err_t err = esp_ble_gap_security_rsp(
            param->ble_security.ble_req.bd_addr, true);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "BLE security response failed: %s", esp_err_to_name(err));
        }
        break;
    }

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGW(TAG, "BLE auth failed, reason: 0x%x",
                     param->ble_security.auth_cmpl.fail_reason);
        }
        break;

    default:
        break;
    }
}

/* ── Blufi event callback ────────────────────────────────────────────────── */

static void blufi_event_callback(esp_blufi_cb_event_t event,
                                 esp_blufi_cb_param_t *param) {
    switch (event) {
    case ESP_BLUFI_EVENT_INIT_FINISH: {
        esp_err_t err = esp_ble_gap_set_device_name(PROV_BLE_DEVICE_NAME);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "BLE set device name failed: %s", esp_err_to_name(err));
        }
        err = esp_ble_gap_config_adv_data(&s_adv_data);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "BLE config adv data failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Blufi init complete — starting BLE advertising");
        }
        break;
    }

    case ESP_BLUFI_EVENT_BLE_CONNECT:
        ESP_LOGI(TAG, "Blufi: BLE client connected");
        esp_blufi_adv_stop();
        break;

    case ESP_BLUFI_EVENT_BLE_DISCONNECT:
        ESP_LOGI(TAG, "Blufi: BLE client disconnected");
        /* Resume advertising only if provisioning is not already complete.
         * After credentials are saved the EventGroup bit is set, so we skip
         * the restart to avoid spurious advertising during teardown. */
        if ((!s_got_ssid || !s_got_pass) &&
            (s_prov_eg == NULL ||
             !(xEventGroupGetBits(s_prov_eg) & BLUFI_PROVISIONED_BIT))) {
            esp_err_t err = esp_ble_gap_start_advertising(&s_adv_params);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "BLE restart advertising failed: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "BLE advertising restarted — waiting for credentials");
            }
        }
        break;

    case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {
        /* Device is in provisioning mode — not connected to any AP yet.
         * Respond with CONN_FAIL so the app shows the credential form. */
        esp_err_t err = esp_blufi_send_wifi_conn_report(WIFI_MODE_NULL,
                                                        ESP_BLUFI_STA_CONN_FAIL,
                                                        0, NULL);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Blufi send wifi status failed: %s", esp_err_to_name(err));
        }
        break;
    }

    case ESP_BLUFI_EVENT_RECV_STA_SSID: {
        /* Blufi sends SSID as raw bytes without null terminator — use
         * explicit length to avoid copying garbage past the payload. */
        size_t len = param->sta_ssid.ssid_len;
        if (len >= sizeof(s_ssid)) {
            len = sizeof(s_ssid) - 1u;
        }
        memcpy(s_ssid, param->sta_ssid.ssid, len);
        s_ssid[len] = '\0';
        s_got_ssid = true;
        ESP_LOGI(TAG, "Blufi: received SSID: %s", s_ssid);
        break;
    }

    case ESP_BLUFI_EVENT_RECV_STA_PASSWD: {
        size_t len = param->sta_passwd.passwd_len;
        if (len >= sizeof(s_pass)) {
            len = sizeof(s_pass) - 1u;
        }
        memcpy(s_pass, param->sta_passwd.passwd, len);
        s_pass[len] = '\0';
        s_got_pass = true;
        ESP_LOGI(TAG, "Blufi: received Wi-Fi password");
        break;
    }

    case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
        esp_blufi_disconnect();
        break;

    default:
        break;
    }

    /* Both credentials received — validate, save, and signal the waiting
     * task.  On any failure reset the flags so the mobile app can reconnect
     * and re-send credentials (advertising resumes on BLE_DISCONNECT). */
    if (s_got_ssid && s_got_pass && s_prov_eg != NULL) {
        s_got_ssid = false;
        s_got_pass = false;

        size_t ssid_len = strlen(s_ssid);
        size_t pass_len = strlen(s_pass);

        /* IEEE 802.11: SSID 1–32 chars; WPA2-PSK passphrase 8–63 chars. */
        if (ssid_len < 1u || ssid_len > 32u) {
            ESP_LOGE(TAG, "Invalid SSID length %u — must be 1-32 chars; "
                          "provisioning remains active", (unsigned)ssid_len);
        } else if (pass_len < 8u || pass_len > 63u) {
            ESP_LOGE(TAG, "Invalid password length %u — WPA2-PSK requires "
                          "8-63 chars; provisioning remains active",
                     (unsigned)pass_len);
        } else if (nvs_save_credentials(s_ssid, s_pass)) {
            xEventGroupSetBits(s_prov_eg, BLUFI_PROVISIONED_BIT);
        } else {
            ESP_LOGE(TAG, "NVS save failed — provisioning remains active");
        }
    }
}

static esp_blufi_callbacks_t s_blufi_callbacks = {
    .event_cb               = blufi_event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func           = blufi_aes_encrypt,
    .decrypt_func           = blufi_aes_decrypt,
    .checksum_func          = blufi_crc_checksum,
};

/* ── BLE stack lifecycle ─────────────────────────────────────────────────── */

static void ble_init_and_start_blufi(void) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    /* No bonding required for provisioning — prevents SMP timeout when
     * the mobile app tries to pair before sending credentials. */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;
    esp_ble_io_cap_t   iocap    = ESP_IO_CAP_NONE;
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE,
                                                   &auth_req, sizeof(auth_req)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE,
                                                   &iocap, sizeof(iocap)));

    ESP_ERROR_CHECK(blufi_security_init());
    ESP_ERROR_CHECK(esp_blufi_register_callbacks(&s_blufi_callbacks));
    ESP_ERROR_CHECK(esp_blufi_profile_init());

    ESP_LOGI(TAG, "Blufi BLE stack started — device name: \"%s\"",
             PROV_BLE_DEVICE_NAME);
}

static void ble_teardown(void) {
    /* Brief delay to let the Blufi profile deliver any final response */
    vTaskDelay(pdMS_TO_TICKS(200u));

    esp_blufi_profile_deinit();
    blufi_security_deinit();

    ESP_ERROR_CHECK(esp_bluedroid_disable());
    ESP_ERROR_CHECK(esp_bluedroid_deinit());
    ESP_ERROR_CHECK(esp_bt_controller_disable());
    ESP_ERROR_CHECK(esp_bt_controller_deinit());

    ESP_LOGI(TAG, "BLE stack fully disabled — RF coex released for Wi-Fi");
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void provisioning_ensure(void) {
    if (nvs_has_credentials()) {
        ESP_LOGI(TAG, "NVS credentials present — Blufi provisioning skipped");
        return;
    }

    ESP_LOGI(TAG, "No NVS credentials — entering Blufi provisioning mode");

    s_prov_eg  = xEventGroupCreate();
    configASSERT(s_prov_eg != NULL);
    s_got_ssid = false;
    s_got_pass = false;

    ble_init_and_start_blufi();

    /* Block indefinitely until mobile app delivers credentials */
    xEventGroupWaitBits(s_prov_eg, BLUFI_PROVISIONED_BIT,
                        pdFALSE, pdFALSE, portMAX_DELAY);

    ble_teardown();

    vEventGroupDelete(s_prov_eg);
    s_prov_eg = NULL;

    ESP_LOGI(TAG, "Provisioning complete");
}

bool provisioning_load_credentials(char *ssid, size_t ssid_len,
                                   char *pass, size_t pass_len) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(PROV_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_get_str(nvs, PROV_NVS_KEY_SSID, ssid, &ssid_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS read SSID failed: %s", esp_err_to_name(err));
        nvs_close(nvs);
        return false;
    }

    err = nvs_get_str(nvs, PROV_NVS_KEY_PASS, pass, &pass_len);
    nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS read password failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

void provisioning_on_wifi_failure(void) {
    ESP_LOGE(TAG, "Wi-Fi failed after %u attempts — erasing credentials, "
                  "restarting into Blufi provisioning mode",
             PROV_WIFI_MAX_RETRIES);
    if (!nvs_erase_credentials()) {
        ESP_LOGE(TAG, "NVS erase failed — restarting anyway");
    }
    esp_restart();
    /* not reached */
}

void provisioning_on_wifi_success(void) {
    /* Reserved for future cross-boot failure counting */
}
