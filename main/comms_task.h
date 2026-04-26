#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* ── comms_task configuration ───────────────────────────────────────────── */

typedef struct {
    QueueHandle_t state_q;   /* IN:  system_state_t ← control_task          */
    QueueHandle_t cmd_q;     /* OUT: mqtt_cmd_t     → control_task           */
} comms_task_config_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * Spawns comms_task on Core 0, priority 3.
 * Initialises Wi-Fi and MQTT client inside the task.
 * Must be called after queues are created.
 */
void comms_task_start(const comms_task_config_t *cfg);
