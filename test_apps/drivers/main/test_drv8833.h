#pragma once

#include "esp_err.h"

/**
 * @brief Validate the DRV8833 driver against live hardware.
 *
 * Exercises both motor channels through forward, brake, reverse and coast
 * sequences, then tests the sleep/wake cycle and reads the FAULT pin.
 * All API calls must return ESP_OK for the test to pass.
 *
 * @return ESP_OK if all operations succeed, error code on first failure.
 */
esp_err_t test_drv8833_run(void);
