#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"

/* ── thermal_task configuration ─────────────────────────────────────────── */

typedef struct {
    int                     relay_gpio;  /* GPIO driving the heating relay   */
    i2c_master_bus_handle_t i2c_bus;     /* I2C bus created by app_main      */
    uint8_t                 bme280_addr; /* 7-bit I2C address                */
    QueueHandle_t           temp_q;      /* OUT: temp_reading_t → control    */
    QueueHandle_t           thermal_q;   /* IN:  thermal_cmd_t  ← control    */
} thermal_task_config_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * Spawns thermal_task on Core 1, priority 4.
 * GPIO and BME280 are initialised inside the task before the loop.
 * Must be called after queues are created and before the cooking cycle starts.
 */
void thermal_task_start(const thermal_task_config_t *cfg);
