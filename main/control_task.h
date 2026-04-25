#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* ── control_task configuration ─────────────────────────────────────────── */

typedef struct {
    QueueHandle_t temp_q;     /* IN:  temp_reading_t  from thermal_task   */
    QueueHandle_t cmd_q;      /* IN:  mqtt_cmd_t      from comms_task     */
    QueueHandle_t motor_q;    /* OUT: motor_cmd_t     to   motor_task     */
    QueueHandle_t state_q;    /* OUT: system_state_t  to   comms_task     */
    QueueHandle_t thermal_q;  /* OUT: thermal_cmd_t   to   thermal_task   */
} control_task_config_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * Spawns control_task on Core 1, priority 5, using the supplied queues for
 * control coordination, including thermal command dispatch.
 * Must be called once from app_main before the scheduler is running tasks
 * that write to temp_q or cmd_q.
 */
void control_task_start(const control_task_config_t *cfg);
