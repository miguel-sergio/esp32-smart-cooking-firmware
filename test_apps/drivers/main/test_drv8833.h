#pragma once

#include "esp_err.h"

/**
 * @brief Validate the DRV8833 driver against live hardware.
 *
 * Performs a basic runtime sanity check by ramping Channel A forward and
 * then commanding coast. This test does not exercise Channel B, brake or
 * reverse modes, sleep/wake control, or FAULT-pin handling.
 * All invoked API calls must return ESP_OK for the test to pass.
 *
 * @return ESP_OK if all operations succeed, error code on first failure.
 */
esp_err_t test_drv8833_run(void);
