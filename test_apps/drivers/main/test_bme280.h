#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

/**
 * @brief Validate the BME280 driver against a live sensor.
 *
 * Reads N samples and checks that each reading falls within the sensor's
 * physical operating range. Logs individual results and a final PASS/FAIL.
 *
 * @param bus  Initialised I2C master bus handle.
 * @return ESP_OK if all samples are within range, ESP_FAIL otherwise.
 */
esp_err_t test_bme280_run(i2c_master_bus_handle_t bus);
