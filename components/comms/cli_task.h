#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief CLI task configuration.
 *
 * Reads single-character commands from UART0 (the serial monitor port) and
 * injects the corresponding mqtt_cmd_t into cmd_q.
 *
 * Key map:
 *   s  — CMD_START  profile 0 (Standard)
 *   S  — CMD_START  profile 1 (Delicate)
 *   x  — CMD_STOP
 *   e  — CMD_ESTOP
 *   r  — CMD_RESET
 *
 * Only compiled when CONFIG_SMART_COOKING_CLI=y.
 */
typedef struct {
    QueueHandle_t cmd_q;    /**< comms_task → control_task command queue */
} cli_task_config_t;

void cli_task_start(const cli_task_config_t *cfg);
