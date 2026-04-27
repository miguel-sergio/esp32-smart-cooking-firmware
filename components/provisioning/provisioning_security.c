#include "provisioning_security.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_crc.h"
#include "esp_blufi_api.h"
#include "soc/soc_caps.h"

#include "psa/crypto.h"
#include "mbedtls/platform_util.h"

#if !SOC_MPI_SUPPORTED
#include "esp_task_wdt.h"
#endif

static const char *TAG = "prov_security";

/* ── Blufi negotiate-key packet type codes ──────────────────────────────────
 * These match the self-defined type bytes used by the Blufi reference app and
 * the official ESP-IDF Blufi example.
 */
#define SEC_TYPE_DH_PARAM_LEN   0x00
#define SEC_TYPE_DH_PARAM_DATA  0x01
#define SEC_TYPE_DH_P           0x02
#define SEC_TYPE_DH_G           0x03
#define SEC_TYPE_DH_PUBLIC      0x04

/* ── Security context ───────────────────────────────────────────────────────
 * Uses the PSA Crypto API (mbedTLS 4.x / ESP-IDF v6.0).
 * DH key agreement: PSA FFDH RFC7919, 3072-bit only.
 * Session encryption: AES-256-CTR with domain-separated IVs derived via SHA-256.
 */
#define DH_PARAM_LEN_MAX        1024
#define DH_SELF_PUB_KEY_LEN     384   /* 3072-bit / 8 */
#define SHARE_KEY_LEN           384
#define PSK_LEN                 32u   /* SHA-256 output → AES-256 key */
#define BLUFI_IV_SIZE           16u
#define BLUFI_HASH_SIZE         32u
#define BLUFI_IV_PREFIX_LEN     9u
#define BLUFI_ENC_DOMAIN_STR    "blufi_enc"
#define BLUFI_DEC_DOMAIN_STR    "blufi_dec"

#if !SOC_MPI_SUPPORTED
/* On SoCs without hardware MPI, 3072-bit modular exponentiation takes ~12s
 * in software.  Pre-generate the DH keypair during BLE advertising so only
 * key agreement remains on the critical path when the phone connects. */
#define BLUFI_DH_PREGEN_STACK_SIZE   4096
#define BLUFI_DH_PREGEN_PRIO        (tskIDLE_PRIORITY + 1)
#define BLUFI_DH_PREGEN_WAIT_MS      30000
#define BLUFI_DH_HEAVY_CRYPTO_WDT_MS 30000
#endif

struct blufi_security {
    uint8_t  self_public_key[DH_SELF_PUB_KEY_LEN];
    uint8_t  share_key[SHARE_KEY_LEN];
    size_t   share_len;
    uint8_t  psk[PSK_LEN];
    uint8_t *dh_param;
    int      dh_param_len;
    psa_key_id_t             aes_key;
    psa_cipher_operation_t   enc_operation;
    psa_cipher_operation_t   dec_operation;
};

static struct blufi_security *blufi_sec;

/* Forward declaration of internal ESP-IDF BT error reporter. */
extern void btc_blufi_report_error(esp_blufi_error_state_t state);

/* ── Internal cleanup helpers ───────────────────────────────────────────── */

static void blufi_cleanup_dh_param(void)
{
    if (blufi_sec && blufi_sec->dh_param) {
        free(blufi_sec->dh_param);
        blufi_sec->dh_param = NULL;
    }
}

static void blufi_cleanup_aes_session(bool abort_enc, bool abort_dec)
{
    if (!blufi_sec) {
        return;
    }
    if (abort_enc) {
        psa_cipher_abort(&blufi_sec->enc_operation);
    }
    if (abort_dec) {
        psa_cipher_abort(&blufi_sec->dec_operation);
    }
    if (blufi_sec->aes_key != 0) {
        psa_destroy_key(blufi_sec->aes_key);
        blufi_sec->aes_key = 0;
    }
}

static void blufi_cleanup_negotiation(bool abort_enc, bool abort_dec)
{
    blufi_cleanup_aes_session(abort_enc, abort_dec);
    blufi_cleanup_dh_param();
}

/* ── DH keypair pre-generation (software-only SoCs) ────────────────────────
 * ESP32 has SOC_MPI_SUPPORTED=1 so this entire block compiles out.
 * Kept for portability to ESP32-S2 and similar SoCs without hardware MPI.
 */
#if !SOC_MPI_SUPPORTED

static psa_key_id_t    s_pregen_private_key;
static uint8_t         s_pregen_public_key[DH_SELF_PUB_KEY_LEN];
static size_t          s_pregen_public_key_len;
static SemaphoreHandle_t s_pregen_done;
static bool            s_pregen_ok;
static TaskHandle_t    s_pregen_task_hdl;
static void          (*s_pregen_complete_cb)(void);

static void blufi_wdt_set_timeout(uint32_t timeout_ms)
{
    esp_task_wdt_config_t cfg = {
        .timeout_ms = timeout_ms,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };
    esp_task_wdt_reconfigure(&cfg);
}

static void blufi_dh_pregen_task(void *arg)
{
    (void)arg;
    blufi_wdt_set_timeout(BLUFI_DH_HEAVY_CRYPTO_WDT_MS);

    psa_key_attributes_t attr = psa_key_attributes_init();
    psa_set_key_type(&attr, PSA_KEY_TYPE_DH_KEY_PAIR(PSA_DH_FAMILY_RFC7919));
    psa_set_key_bits(&attr, 3072);
    psa_set_key_algorithm(&attr, PSA_ALG_FFDH);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DERIVE);

    psa_status_t status = psa_generate_key(&attr, &s_pregen_private_key);
    if (status == PSA_SUCCESS) {
        status = psa_export_public_key(s_pregen_private_key,
                                       s_pregen_public_key, DH_SELF_PUB_KEY_LEN,
                                       &s_pregen_public_key_len);
    }
    psa_reset_key_attributes(&attr);

    s_pregen_ok = (status == PSA_SUCCESS);
    if (!s_pregen_ok && s_pregen_private_key != 0) {
        psa_destroy_key(s_pregen_private_key);
        s_pregen_private_key = 0;
    }

    ESP_LOGI(TAG, "DH keypair pre-generation %s", s_pregen_ok ? "done" : "FAILED");
    blufi_wdt_set_timeout(CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000);
    xSemaphoreGive(s_pregen_done);

    if (s_pregen_complete_cb) {
        void (*cb)(void) = s_pregen_complete_cb;
        s_pregen_complete_cb = NULL;
        cb();
    }
    vTaskSuspend(NULL);
}

static void blufi_dh_pregen_start_impl(void (*done_cb)(void))
{
    if (s_pregen_done != NULL) {
        return;
    }
    s_pregen_private_key = 0;
    s_pregen_public_key_len = 0;
    s_pregen_ok = false;
    s_pregen_task_hdl = NULL;
    s_pregen_complete_cb = done_cb;
    s_pregen_done = xSemaphoreCreateBinary();
    if (s_pregen_done == NULL) {
        ESP_LOGE(TAG, "pregen: failed to create semaphore");
        return;
    }
    if (xTaskCreate(blufi_dh_pregen_task, "blufi_pregen",
                    BLUFI_DH_PREGEN_STACK_SIZE, NULL,
                    BLUFI_DH_PREGEN_PRIO, &s_pregen_task_hdl) != pdPASS) {
        ESP_LOGE(TAG, "pregen: failed to create task");
        vSemaphoreDelete(s_pregen_done);
        s_pregen_done = NULL;
    }
}

void blufi_dh_pregen_start(void)
{
    blufi_dh_pregen_start_impl(NULL);
}

void blufi_dh_pregen_start_with_cb(void (*done_cb)(void))
{
    blufi_dh_pregen_start_impl(done_cb);
}

void blufi_dh_pregen_wait(void)
{
    if (s_pregen_done == NULL) {
        return;
    }
    xSemaphoreTake(s_pregen_done, portMAX_DELAY);
    xSemaphoreGive(s_pregen_done);
}

static void blufi_dh_pregen_cleanup(void)
{
    if (s_pregen_done != NULL) {
        xSemaphoreTake(s_pregen_done, portMAX_DELAY);
        vSemaphoreDelete(s_pregen_done);
        s_pregen_done = NULL;
    }
    if (s_pregen_task_hdl != NULL) {
        vTaskDelete(s_pregen_task_hdl);
        s_pregen_task_hdl = NULL;
    }
    if (s_pregen_private_key != 0) {
        psa_destroy_key(s_pregen_private_key);
        s_pregen_private_key = 0;
    }
    s_pregen_ok = false;
}

#endif /* !SOC_MPI_SUPPORTED */

/* ── Public API ─────────────────────────────────────────────────────────── */

esp_err_t blufi_security_init(void)
{
    blufi_sec = (struct blufi_security *)malloc(sizeof(struct blufi_security));
    if (blufi_sec == NULL) {
        ESP_LOGE(TAG, "Failed to allocate security context");
        return ESP_FAIL;
    }
    memset(blufi_sec, 0, sizeof(struct blufi_security));
    blufi_sec->enc_operation = psa_cipher_operation_init();
    blufi_sec->dec_operation = psa_cipher_operation_init();
#if !SOC_MPI_SUPPORTED
    blufi_dh_pregen_start();
#endif
    return ESP_OK;
}

void blufi_security_deinit(void)
{
    if (blufi_sec == NULL) {
        return;
    }
#if !SOC_MPI_SUPPORTED
    blufi_dh_pregen_cleanup();
#endif
    blufi_cleanup_negotiation(true, true);
    memset(blufi_sec, 0, sizeof(struct blufi_security));
    free(blufi_sec);
    blufi_sec = NULL;
}

/* ── DH negotiate_data_handler ──────────────────────────────────────────────
 *
 * The Blufi key-negotiation exchange proceeds in two steps:
 *
 *  1. Phone sends SEC_TYPE_DH_PARAM_LEN — we learn how many bytes the DH
 *     parameters (P, G, phone public key) will occupy and allocate a buffer.
 *
 *  2. Phone sends SEC_TYPE_DH_PARAM_DATA — we parse P/G/phone-pub, generate
 *     our PSA FFDH key pair (RFC7919, 3072-bit), compute the shared secret,
 *     derive a session key via SHA-256, set up AES-CTR cipher operations, and
 *     return our public key so the phone can complete the same derivation.
 *
 * After this exchange all Blufi payload frames are AES-256-CTR encrypted.
 */
// cppcheck-suppress constParameterPointer -- signature fixed by esp_blufi_callbacks_t
// NOLINTNEXTLINE(readability-non-const-parameter) -- signature fixed by esp_blufi_callbacks_t
void blufi_dh_negotiate_data_handler(uint8_t *data, int len,
                                     uint8_t **output_data, int *output_len,
                                     bool *need_free)
{
    if (data == NULL || len < 3) {
        ESP_LOGE(TAG, "negotiate: invalid data format");
        btc_blufi_report_error(ESP_BLUFI_DATA_FORMAT_ERROR);
        return;
    }
    if (output_data == NULL || output_len == NULL || need_free == NULL) {
        ESP_LOGE(TAG, "negotiate: invalid output parameters");
        btc_blufi_report_error(ESP_BLUFI_DATA_FORMAT_ERROR);
        return;
    }
    if (blufi_sec == NULL) {
        ESP_LOGE(TAG, "negotiate: security not initialised");
        btc_blufi_report_error(ESP_BLUFI_INIT_SECURITY_ERROR);
        return;
    }

    uint8_t type = data[0];

    switch (type) {
    case SEC_TYPE_DH_PARAM_LEN: {
        int param_len = ((data[1] << 8) | data[2]);
        if (param_len == 0 || param_len > DH_PARAM_LEN_MAX) {
            ESP_LOGE(TAG, "negotiate: invalid DH param len %d", param_len);
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }
        /* Allow renegotiation — clean up previous state. */
        blufi_cleanup_dh_param();
        blufi_cleanup_aes_session(true, true);

        blufi_sec->dh_param_len = param_len;
        blufi_sec->dh_param = (uint8_t *)malloc((size_t)param_len);
        if (blufi_sec->dh_param == NULL) {
            blufi_sec->dh_param_len = 0;
            ESP_LOGE(TAG, "negotiate: malloc failed for DH params");
            btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
        }
        break;
    }

    case SEC_TYPE_DH_PARAM_DATA: {
        if (blufi_sec->dh_param == NULL) {
            ESP_LOGE(TAG, "negotiate: DH param buffer not allocated");
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }
        if (len < (blufi_sec->dh_param_len + 1)) {
            ESP_LOGE(TAG, "negotiate: DH param data too short");
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        memcpy(blufi_sec->dh_param, &data[1], (size_t)blufi_sec->dh_param_len);
        uint8_t *param = blufi_sec->dh_param;

        /* Parse P (big-endian length-prefixed). */
        if (blufi_sec->dh_param_len < 2) {
            ESP_LOGE(TAG, "negotiate: DH param too short for P len");
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }
        size_t p_len = ((size_t)param[0] << 8) | param[1];
        size_t p_offset = 2u + p_len;
        if (p_offset > (size_t)blufi_sec->dh_param_len) {
            ESP_LOGE(TAG, "negotiate: P length %d exceeds bounds", (int)p_len);
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }
        param += p_offset;

        /* Parse G. */
        if ((param - blufi_sec->dh_param) + 2 > (size_t)blufi_sec->dh_param_len) {
            ESP_LOGE(TAG, "negotiate: DH param too short for G len");
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }
        size_t g_len = ((size_t)param[0] << 8) | param[1];
        param += 2u + g_len;
        if ((size_t)(param - blufi_sec->dh_param) > (size_t)blufi_sec->dh_param_len) {
            ESP_LOGE(TAG, "negotiate: G length exceeds bounds");
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        /* Parse phone's public key. */
        if ((param - blufi_sec->dh_param) + 2 > (size_t)blufi_sec->dh_param_len) {
            ESP_LOGE(TAG, "negotiate: DH param too short for pub key len");
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }
        size_t pub_len = ((size_t)param[0] << 8) | param[1];
        param += 2u;

        if ((size_t)(param - blufi_sec->dh_param) + pub_len > (size_t)blufi_sec->dh_param_len) {
            ESP_LOGE(TAG, "negotiate: pub key length exceeds bounds");
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }
        /* Only 3072-bit (384-byte) keys are supported (matches DH_SELF_PUB_KEY_LEN). */
        if (pub_len * 8u != 3072u || pub_len > DH_SELF_PUB_KEY_LEN) {
            ESP_LOGE(TAG, "negotiate: unsupported DH key size %d bytes", (int)pub_len);
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        psa_key_id_t private_key = 0;
        size_t       pub_key_out_len = 0u;
        psa_status_t status;

#if !SOC_MPI_SUPPORTED
        /* Use pre-generated keypair when available; fall back to inline generation. */
        bool used_pregen = false;
        if (s_pregen_done != NULL &&
            xSemaphoreTake(s_pregen_done,
                           pdMS_TO_TICKS(BLUFI_DH_PREGEN_WAIT_MS)) == pdTRUE) {
            xSemaphoreGive(s_pregen_done);
            if (s_pregen_ok && s_pregen_private_key != 0) {
                private_key = s_pregen_private_key;
                s_pregen_private_key = 0;
                memcpy(blufi_sec->self_public_key,
                       s_pregen_public_key, s_pregen_public_key_len);
                pub_key_out_len = s_pregen_public_key_len;
                used_pregen = true;
                ESP_LOGI(TAG, "negotiate: using pre-generated DH keypair");
            }
        }
        if (!used_pregen) {
            ESP_LOGW(TAG, "negotiate: pre-gen unavailable, generating inline");
            blufi_wdt_set_timeout(BLUFI_DH_HEAVY_CRYPTO_WDT_MS);
            psa_key_attributes_t attrs = psa_key_attributes_init();
            psa_set_key_type(&attrs, PSA_KEY_TYPE_DH_KEY_PAIR(PSA_DH_FAMILY_RFC7919));
            psa_set_key_bits(&attrs, pub_len * 8u);
            psa_set_key_algorithm(&attrs, PSA_ALG_FFDH);
            psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_DERIVE);
            status = psa_generate_key(&attrs, &private_key);
            psa_reset_key_attributes(&attrs);
            if (status != PSA_SUCCESS) {
                ESP_LOGE(TAG, "negotiate: psa_generate_key failed: %d", (int)status);
                blufi_wdt_set_timeout(CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000);
                blufi_cleanup_dh_param();
                btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
                return;
            }
            status = psa_export_public_key(private_key,
                                           blufi_sec->self_public_key,
                                           DH_SELF_PUB_KEY_LEN,
                                           &pub_key_out_len);
            if (status != PSA_SUCCESS) {
                ESP_LOGE(TAG, "negotiate: psa_export_public_key failed: %d", (int)status);
                blufi_wdt_set_timeout(CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000);
                psa_destroy_key(private_key);
                blufi_cleanup_dh_param();
                btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
                return;
            }
        }
        /* Copy peer public key before releasing dh_param buffer. */
        uint8_t peer_pub[DH_SELF_PUB_KEY_LEN];
        memcpy(peer_pub, param, pub_len);
        blufi_cleanup_dh_param();
        if (s_pregen_task_hdl != NULL) {
            vTaskDelete(s_pregen_task_hdl);
            s_pregen_task_hdl = NULL;
        }
        blufi_wdt_set_timeout(BLUFI_DH_HEAVY_CRYPTO_WDT_MS);
        status = psa_raw_key_agreement(PSA_ALG_FFDH, private_key,
                                       peer_pub, pub_len,
                                       blufi_sec->share_key, SHARE_KEY_LEN,
                                       &blufi_sec->share_len);
        psa_destroy_key(private_key);
        blufi_wdt_set_timeout(CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000);
#else /* SOC_MPI_SUPPORTED — hardware MPI available, generate inline without WDT guard */
        {
            psa_key_attributes_t attrs = psa_key_attributes_init();
            psa_set_key_type(&attrs, PSA_KEY_TYPE_DH_KEY_PAIR(PSA_DH_FAMILY_RFC7919));
            psa_set_key_bits(&attrs, pub_len * 8u);
            psa_set_key_algorithm(&attrs, PSA_ALG_FFDH);
            psa_set_key_usage_flags(&attrs, PSA_KEY_USAGE_DERIVE);
            status = psa_generate_key(&attrs, &private_key);
            psa_reset_key_attributes(&attrs);
            if (status != PSA_SUCCESS) {
                ESP_LOGE(TAG, "negotiate: psa_generate_key failed: %d", (int)status);
                blufi_cleanup_dh_param();
                btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
                return;
            }
            status = psa_export_public_key(private_key,
                                           blufi_sec->self_public_key,
                                           DH_SELF_PUB_KEY_LEN,
                                           &pub_key_out_len);
            if (status != PSA_SUCCESS) {
                ESP_LOGE(TAG, "negotiate: psa_export_public_key failed: %d", (int)status);
                psa_destroy_key(private_key);
                blufi_cleanup_dh_param();
                btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
                return;
            }
        }
        status = psa_raw_key_agreement(PSA_ALG_FFDH, private_key,
                                       param, pub_len,
                                       blufi_sec->share_key, SHARE_KEY_LEN,
                                       &blufi_sec->share_len);
        psa_destroy_key(private_key);
        blufi_cleanup_dh_param();
#endif /* !SOC_MPI_SUPPORTED */

        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "negotiate: psa_raw_key_agreement failed: %d", (int)status);
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        /* Sanity-check: PSA should never write more than SHARE_KEY_LEN bytes. */
        if (blufi_sec->share_len > SHARE_KEY_LEN) {
            ESP_LOGE(TAG, "negotiate: share_len %d exceeds buffer", (int)blufi_sec->share_len);
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        /* Derive session key: SHA-256(shared_secret). */
        size_t hash_len = 0u;
        status = psa_hash_compute(PSA_ALG_SHA_256,
                                  blufi_sec->share_key, blufi_sec->share_len,
                                  blufi_sec->psk, PSK_LEN,
                                  &hash_len);
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "negotiate: psa_hash_compute failed: %d", (int)status);
            btc_blufi_report_error(ESP_BLUFI_CALC_SHA_256_ERROR);
            blufi_cleanup_dh_param();
            return;
        }

        /* Import AES key for CTR mode. */
        blufi_cleanup_aes_session(true, true);
        {
            psa_key_attributes_t aes_attrs = psa_key_attributes_init();
            psa_set_key_type(&aes_attrs, PSA_KEY_TYPE_AES);
            psa_set_key_bits(&aes_attrs, PSK_LEN * 8u);
            psa_set_key_algorithm(&aes_attrs, PSA_ALG_CTR);
            psa_set_key_usage_flags(&aes_attrs, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);
            status = psa_import_key(&aes_attrs, blufi_sec->psk, PSK_LEN, &blufi_sec->aes_key);
            psa_reset_key_attributes(&aes_attrs);
        }
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "negotiate: psa_import_key failed: %d", (int)status);
            btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
            return;
        }

        /* Derive domain-separated IVs via SHA-256(domain || shared_secret). */
        uint8_t iv_material[BLUFI_IV_PREFIX_LEN + SHARE_KEY_LEN];
        uint8_t iv_hash[BLUFI_HASH_SIZE];

        memcpy(iv_material, BLUFI_ENC_DOMAIN_STR, BLUFI_IV_PREFIX_LEN);
        memcpy(iv_material + BLUFI_IV_PREFIX_LEN, blufi_sec->share_key, blufi_sec->share_len);
        status = psa_hash_compute(PSA_ALG_SHA_256,
                                  iv_material, BLUFI_IV_PREFIX_LEN + blufi_sec->share_len,
                                  iv_hash, BLUFI_HASH_SIZE, &hash_len);
        mbedtls_platform_zeroize(iv_material, sizeof(iv_material));
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "negotiate: enc IV derivation failed: %d", (int)status);
            blufi_cleanup_negotiation(false, false);
            btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
            return;
        }
        uint8_t iv_enc[BLUFI_IV_SIZE];
        memcpy(iv_enc, iv_hash, BLUFI_IV_SIZE);

        memcpy(iv_material, BLUFI_DEC_DOMAIN_STR, BLUFI_IV_PREFIX_LEN);
        memcpy(iv_material + BLUFI_IV_PREFIX_LEN, blufi_sec->share_key, blufi_sec->share_len);

        uint8_t iv_dec[BLUFI_IV_SIZE];
        status = psa_hash_compute(PSA_ALG_SHA_256,
                                  iv_material, BLUFI_IV_PREFIX_LEN + blufi_sec->share_len,
                                  iv_hash, BLUFI_HASH_SIZE, &hash_len);
        mbedtls_platform_zeroize(iv_material, sizeof(iv_material));
        if (status != PSA_SUCCESS) {
            mbedtls_platform_zeroize(iv_hash, sizeof(iv_hash));
            ESP_LOGE(TAG, "negotiate: dec IV derivation failed: %d", (int)status);
            blufi_cleanup_negotiation(false, false);
            btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
            return;
        }
        memcpy(iv_dec, iv_hash, BLUFI_IV_SIZE);
        mbedtls_platform_zeroize(iv_hash, sizeof(iv_hash));

        /* Set up persistent CTR cipher operations. */
        blufi_sec->enc_operation = psa_cipher_operation_init();
        status = psa_cipher_encrypt_setup(&blufi_sec->enc_operation,
                                          blufi_sec->aes_key, PSA_ALG_CTR);
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "negotiate: enc setup failed: %d", (int)status);
            blufi_cleanup_negotiation(false, false);
            btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
            return;
        }
        status = psa_cipher_set_iv(&blufi_sec->enc_operation, iv_enc, BLUFI_IV_SIZE);
        mbedtls_platform_zeroize(iv_enc, sizeof(iv_enc));
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "negotiate: enc IV set failed: %d", (int)status);
            blufi_cleanup_negotiation(true, false);
            btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
            return;
        }

        blufi_sec->dec_operation = psa_cipher_operation_init();
        /* CTR mode uses encrypt_setup for both enc and dec. */
        status = psa_cipher_encrypt_setup(&blufi_sec->dec_operation,
                                          blufi_sec->aes_key, PSA_ALG_CTR);
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "negotiate: dec setup failed: %d", (int)status);
            blufi_cleanup_negotiation(true, false);
            btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
            return;
        }
        status = psa_cipher_set_iv(&blufi_sec->dec_operation, iv_dec, BLUFI_IV_SIZE);
        mbedtls_platform_zeroize(iv_dec, sizeof(iv_dec));
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "negotiate: dec IV set failed: %d", (int)status);
            blufi_cleanup_negotiation(true, true);
            btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
            return;
        }

        *output_data = blufi_sec->self_public_key;
        *output_len  = (int)pub_key_out_len;
        *need_free   = false;

        ESP_LOGI(TAG, "DH key exchange complete — Blufi AES-256-CTR session active");
        break;
    }

    case SEC_TYPE_DH_P:
    case SEC_TYPE_DH_G:
    case SEC_TYPE_DH_PUBLIC:
        /* Individual P/G/pubkey frames — handled implicitly via PARAM_DATA. */
        break;

    default:
        ESP_LOGW(TAG, "negotiate: unknown type 0x%02x", type);
        btc_blufi_report_error(ESP_BLUFI_DATA_FORMAT_ERROR);
        break;
    }
}

/* ── AES-256-CTR encrypt ─────────────────────────────────────────────────── */

int blufi_aes_encrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len)
{
    (void)iv8;  /* Persistent CTR operation manages the counter automatically. */

    if (!blufi_sec || !crypt_data || crypt_len < 0) {
        return -1;
    }
    if (crypt_len == 0) {
        return 0;
    }
    if (blufi_sec->aes_key == 0) {
        ESP_LOGE(TAG, "encrypt: AES key not initialised");
        return -1;
    }

    size_t  out_len = 0u;
    size_t  out_buf_size = PSA_CIPHER_ENCRYPT_OUTPUT_SIZE(PSA_KEY_TYPE_AES,
                                                          PSA_ALG_CTR,
                                                          (size_t)crypt_len);
    uint8_t *out_buf = (uint8_t *)malloc(out_buf_size);
    if (out_buf == NULL) {
        ESP_LOGE(TAG, "encrypt: malloc failed");
        return -1;
    }

    psa_status_t status = psa_cipher_update(&blufi_sec->enc_operation,
                                            crypt_data, (size_t)crypt_len,
                                            out_buf, out_buf_size, &out_len);
    if (status != PSA_SUCCESS || out_len > (size_t)crypt_len) {
        ESP_LOGE(TAG, "encrypt: psa_cipher_update failed: %d", (int)status);
        free(out_buf);
        return -1;
    }

    memcpy(crypt_data, out_buf, out_len);
    free(out_buf);
    return (int)out_len;
}

/* ── AES-256-CTR decrypt ─────────────────────────────────────────────────── */

int blufi_aes_decrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len)
{
    (void)iv8;

    if (!blufi_sec || !crypt_data || crypt_len < 0) {
        return -1;
    }
    if (crypt_len == 0) {
        return 0;
    }
    if (blufi_sec->aes_key == 0) {
        ESP_LOGE(TAG, "decrypt: AES key not initialised");
        return -1;
    }

    size_t  out_len = 0u;
    size_t  out_buf_size = PSA_CIPHER_DECRYPT_OUTPUT_SIZE(PSA_KEY_TYPE_AES,
                                                          PSA_ALG_CTR,
                                                          (size_t)crypt_len);
    uint8_t *out_buf = (uint8_t *)malloc(out_buf_size);
    if (out_buf == NULL) {
        ESP_LOGE(TAG, "decrypt: malloc failed");
        return -1;
    }

    psa_status_t status = psa_cipher_update(&blufi_sec->dec_operation,
                                            crypt_data, (size_t)crypt_len,
                                            out_buf, out_buf_size, &out_len);
    if (status != PSA_SUCCESS || out_len > (size_t)crypt_len) {
        ESP_LOGE(TAG, "decrypt: psa_cipher_update failed: %d", (int)status);
        free(out_buf);
        return -1;
    }

    memcpy(crypt_data, out_buf, out_len);
    free(out_buf);
    return (int)out_len;
}

/* ── CRC-16 checksum ────────────────────────────────────────────────────── */

uint16_t blufi_crc_checksum(uint8_t iv8, uint8_t *data, int len)
{
    (void)iv8;  /* Blufi passes iv8 but the standard example ignores it. */

    if (data == NULL || len <= 0) {
        return 0u;
    }
    return esp_crc16_be(0u, data, (uint32_t)len);
}
