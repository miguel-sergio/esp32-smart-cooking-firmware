#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Default I2C address — SDO pulled low. */
#define BME280_I2C_ADDR_DEFAULT  0x76
/** Alternative I2C address — SDO pulled high. */
#define BME280_I2C_ADDR_ALT      0x77

/** Opaque driver handle. Allocated by bme280_init(), freed by bme280_deinit(). */
typedef struct bme280_dev bme280_dev_t;

/** Physical sensor readings after compensation. */
typedef struct {
    float temperature; /**< Degrees Celsius     */
    float humidity;    /**< Relative humidity % */
    float pressure;    /**< Pressure in hPa     */
} bme280_data_t;

/**
 * @brief  Initialise the BME280 and attach it to an existing I2C master bus.
 *
 * Verifies the chip-ID, performs a soft reset, reads factory calibration data
 * and puts the sensor in normal mode (oversampling x1 for T, P, H).
 *
 * @param  bus     I2C master bus handle (created by the caller via i2c_new_master_bus()).
 * @param  addr    7-bit device address: BME280_I2C_ADDR_DEFAULT or BME280_I2C_ADDR_ALT.
 * @param  out_dev Receives the allocated driver handle on success.
 * @return ESP_OK            - sensor found and configured.
 *         ESP_ERR_NOT_FOUND - chip-ID mismatch (wrong device on that address).
 *         ESP_ERR_NO_MEM   - heap allocation failed.
 *         other            - I2C transport error.
 */
esp_err_t bme280_init(i2c_master_bus_handle_t bus, uint8_t addr, bme280_dev_t **out_dev);

/**
 * @brief  Read the latest compensated temperature, humidity and pressure.
 *
 * In normal mode the sensor refreshes its output registers automatically;
 * call this function at any rate up to the configured standby time.
 *
 * @param  dev  Handle returned by bme280_init().
 * @param  out  Struct that receives the readings.
 * @return ESP_OK on success, I2C error code on failure.
 */
esp_err_t bme280_read(bme280_dev_t *dev, bme280_data_t *out);

/**
 * @brief  Remove the device from the I2C bus and release all resources.
 *
 * @param  dev  Handle returned by bme280_init(). No-op if NULL.
 */
void bme280_deinit(bme280_dev_t *dev);

#ifdef __cplusplus
}
#endif
