#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* Maximum URL length accepted via cooking/ota (includes null terminator). */
#define OTA_URL_MAX_LEN  512u

/* ── ota_task configuration ─────────────────────────────────────────────── */

typedef struct {
    QueueHandle_t ota_url_q;   /* IN: char[OTA_URL_MAX_LEN] from comms_task */
} ota_task_config_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * Spawns ota_task on Core 0, priority 2.
 * Waits for a URL from ota_url_q, then calls esp_https_ota() and restarts.
 * Must be called after the queue is created.
 */
void ota_task_start(const ota_task_config_t *cfg);
