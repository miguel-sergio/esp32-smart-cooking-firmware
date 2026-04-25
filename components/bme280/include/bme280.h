#pragma once

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Default I2C address — SDO pulled low. */
#define BME280_I2C_ADDR_DEFAULT  0x76
/** Alternative I2C address — SDO pulled high. */
#define BME280_I2C_ADDR_ALT      0x77

/** Calibration coefficients — populated by bme280_init(). Do not modify. */
typedef struct {
    uint16_t T1; int16_t T2; int16_t T3;
    uint16_t P1; int16_t P2; int16_t P3; int16_t P4;
    int16_t  P5; int16_t P6; int16_t P7; int16_t P8; int16_t P9;
    uint8_t  H1; int16_t H2; uint8_t  H3;
    int16_t  H4; int16_t H5; int8_t   H6;
} bme280_calib_t;

/**
 * Driver handle. Declare one instance (e.g. static) and pass its address
 * to bme280_init(). All fields are populated by the driver; do not modify
 * them directly.
 */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    bme280_calib_t          calib;
} bme280_dev_t;

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
 * The caller is responsible for allocating the bme280_dev_t instance. Typical usage:
 * @code
 *   static bme280_dev_t sensor;
 *   ESP_ERROR_CHECK(bme280_init(bus, BME280_I2C_ADDR_DEFAULT, &sensor));
 * @endcode
 *
 * @param  bus  I2C master bus handle (created by the caller via i2c_new_master_bus()).
 * @param  addr 7-bit device address: BME280_I2C_ADDR_DEFAULT or BME280_I2C_ADDR_ALT.
 * @param  dev  Pointer to a caller-allocated bme280_dev_t to initialise.
 * @return ESP_OK            - sensor found and configured.
 *         ESP_ERR_NOT_FOUND - chip-ID mismatch (wrong device on that address).
 *         other            - I2C transport error.
 */
esp_err_t bme280_init(i2c_master_bus_handle_t bus, uint8_t addr, bme280_dev_t *dev);

/**
 * @brief  Read the latest compensated temperature, humidity and pressure.
 *
 * In normal mode the sensor refreshes its output registers automatically;
 * call this function at any rate up to the configured standby time.
 *
 * @param  dev  Pointer to an initialised bme280_dev_t.
 * @param  out  Struct that receives the readings.
 * @return ESP_OK on success, I2C error code on failure.
 */
esp_err_t bme280_read(bme280_dev_t *dev, bme280_data_t *out);

/**
 * @brief  Remove the device from the I2C bus.
 *
 * Does not free the bme280_dev_t — the caller owns the storage.
 *
 * @param  dev  Pointer to an initialised bme280_dev_t. No-op if NULL.
 */
void bme280_deinit(bme280_dev_t *dev);

#ifdef __cplusplus
}
#endif
