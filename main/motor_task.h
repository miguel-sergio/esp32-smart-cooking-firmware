#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "drv8833.h"

/* ── motor_task configuration ───────────────────────────────────────────── */

typedef struct {
    drv8833_config_t  drv_cfg;    /* DRV8833 hardware config (pins, LEDC)    */
    drv8833_channel_t ch;         /* which H-bridge channel drives the motor  */
    QueueHandle_t     motor_q;    /* IN: motor_cmd_t ← control_task          */
} motor_task_config_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * Spawns motor_task on Core 1, priority 4.
 * DRV8833 is initialised inside the task before the loop.
 * Must be called after queues are created.
 */
void motor_task_start(const motor_task_config_t *cfg);
