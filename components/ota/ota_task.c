#include "ota_task.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_system.h"

static const char *TAG = "ota";

/* ── Task constants ─────────────────────────────────────────────────────── */

/* 8 KB stack — esp_https_ota needs TLS buffers (~6 KB) */
#define OTA_STACK_WORDS   (8192u / sizeof(StackType_t))
#define OTA_PRIORITY      2
#define OTA_CORE          0

/* ── Module state ───────────────────────────────────────────────────────── */

static ota_task_config_t s_cfg;

/* ── Task ───────────────────────────────────────────────────────────────── */

static void ota_task(void *arg) {
    (void)arg;

    /* NOTE: ota_task is intentionally NOT registered with the task watchdog.
     *
     * NFR-01 requires WDT on all tasks, but ota_task is a deliberate exception:
     *   - It only runs when the cooking cycle is inactive (IDLE/DONE/ERROR);
     *     a hang here cannot affect actuator safety or Core 1 real-time control.
     *   - esp_https_ota() can legitimately take 30-120 s on slow links, well
     *     beyond the 5 s TWDT timeout. A mid-download WDT reset is
     *     indistinguishable from a real hang and forces an unnecessary restart.
     *   - The HTTP socket timeout (timeout_ms=30000) is the actual hang guard:
     *     if the server stops sending, the socket errors out cleanly within
     *     30 s and the task loops back to wait for a corrected URL.
     *   - Matches Espressif's own OTA examples, which also omit TWDT.
     * See SDD §4 — NFR-01 exception note. */

    for (;;) {
        /* Block indefinitely until comms_task dispatches a URL.
         * comms_task guarantees it only sends when state is IDLE/DONE/ERROR
         * so we never interrupt an active cooking cycle. */
        char url[OTA_URL_MAX_LEN];
        xQueueReceive(s_cfg.ota_url_q, url, portMAX_DELAY);
        url[OTA_URL_MAX_LEN - 1u] = '\0';   /* safety: ensure null-terminated */

        ESP_LOGI(TAG, "OTA: starting download from %s", url);

        esp_http_client_config_t http_cfg = {
            .url               = url,
            .keep_alive_enable = true,
            .buffer_size_tx    = 4096,   /* GitHub response headers are large */
            .timeout_ms        = 30000,  /* socket recv timeout — guards against server hang.
                                          * 30 s allows for TCP retransmissions on poor Wi-Fi.
                                          * Not WDT-constrained: ota_task excluded from TWDT. */
            .crt_bundle_attach = esp_crt_bundle_attach,
        };
        esp_https_ota_config_t ota_cfg = {
            .http_config = &http_cfg,
        };

        esp_err_t err = esp_https_ota(&ota_cfg);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "OTA: download complete — restarting");
            esp_restart();
            /* not reached */
        } else {
            ESP_LOGE(TAG, "OTA: download failed (%s) — will retry on next URL",
                     esp_err_to_name(err));
            /* Loop back — wait for operator to re-publish a corrected URL */
        }
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void ota_task_start(const ota_task_config_t *cfg) {
    configASSERT(cfg != NULL);
    configASSERT(cfg->ota_url_q != NULL);
    s_cfg = *cfg;

    BaseType_t ret = xTaskCreatePinnedToCore(
        ota_task,
        "ota",
        OTA_STACK_WORDS,
        NULL,
        OTA_PRIORITY,
        NULL,
        OTA_CORE
    );
    configASSERT(ret == pdPASS);
}
