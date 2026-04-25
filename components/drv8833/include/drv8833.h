#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Motor channel selector for the dual H-bridge. */
typedef enum {
    DRV8833_CHANNEL_A = 0,
    DRV8833_CHANNEL_B = 1,
} drv8833_channel_t;

/**
 * Initialisation configuration for the DRV8833 dual H-bridge driver.
 *
 * Each motor channel requires two GPIO pins wired to xIN1/xIN2 and two
 * distinct LEDC channels driven by a shared LEDC timer.
 *
 * Set nsleep_gpio / fault_gpio to -1 when those pins are not connected.
 */
typedef struct {
    /* ---- Motor channel A ---- */
    int            ain1_gpio;
    int            ain2_gpio;
    ledc_channel_t ain1_ledc_ch;
    ledc_channel_t ain2_ledc_ch;
    /* ---- Motor channel B ---- */
    int            bin1_gpio;
    int            bin2_gpio;
    ledc_channel_t bin1_ledc_ch;
    ledc_channel_t bin2_ledc_ch;
    /* ---- Shared LEDC timer ---- */
    ledc_timer_t   ledc_timer;
    uint32_t       pwm_freq_hz;   /**< Switching frequency, e.g. 20000 */
    /* ---- Optional control pins ---- */
    int            nsleep_gpio;   /**< Active-low sleep; -1 = not connected */
    int            fault_gpio;    /**< Active-low fault output (input); -1 = not connected */
} drv8833_config_t;

/**
 * Driver handle. Declare one instance (e.g. static) and pass its address
 * to drv8833_init(). All fields are written by the driver; do not modify
 * them directly.
 */
typedef struct {
    ledc_channel_t ain1_ch;
    ledc_channel_t ain2_ch;
    ledc_channel_t bin1_ch;
    ledc_channel_t bin2_ch;
    ledc_timer_t   timer;
    int            nsleep_gpio;
    int            fault_gpio;
    int8_t         cur_speed[2]; /**< Last set speed per channel [0]=A [1]=B */
} drv8833_dev_t;

/**
 * @brief  Initialise the DRV8833, configure the LEDC timer, four PWM channels
 *         and the optional nSLEEP / FAULT GPIO pins.
 *
 * After a successful call the device is awake and both channels coast
 * (all IN pins at 0 %).
 *
 * The caller is responsible for allocating the drv8833_dev_t instance. Typical usage:
 * @code
 *   drv8833_dev_t motor;
 *   ESP_ERROR_CHECK(drv8833_init(&cfg, &motor));
 * @endcode
 *
 * @param  cfg  Pointer to a filled drv8833_config_t.
 * @param  dev  Pointer to a caller-allocated drv8833_dev_t to initialise.
 * @return ESP_OK on success.
 *         ESP_ERR_INVALID_ARG if cfg or dev is NULL.
 *         LEDC or GPIO error code otherwise.
 */
esp_err_t drv8833_init(const drv8833_config_t *cfg, drv8833_dev_t *dev);

/**
 * @brief  Set motor speed and direction for one channel (fast-decay PWM).
 *
 * The active IN pin is driven with the requested duty cycle; the
 * complementary pin is held low.
 *
 * @param  dev       Pointer to an initialised drv8833_dev_t.
 * @param  ch        DRV8833_CHANNEL_A or DRV8833_CHANNEL_B.
 * @param  speed_pct Speed in the range [-100, 100].
 *                   Positive = forward, negative = reverse, 0 = coast.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if dev is NULL or speed is out of range.
 */
esp_err_t drv8833_set_speed(drv8833_dev_t *dev, drv8833_channel_t ch, int8_t speed_pct);

/**
 * @brief  Ramp motor speed smoothly from the current speed to a target.
 *
 * Interpolates linearly from the last speed set on the channel to
 * @p target_pct over @p ramp_ms milliseconds in 10 ms steps.
 * Blocks the calling task for the duration of the ramp.
 * If @p ramp_ms is 0, behaves identically to drv8833_set_speed().
 *
 * @param  dev        Pointer to an initialised drv8833_dev_t.
 * @param  ch         DRV8833_CHANNEL_A or DRV8833_CHANNEL_B.
 * @param  target_pct Target speed in [-100, 100].
 * @param  ramp_ms    Ramp duration in milliseconds.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if dev is NULL or target is out of range.
 */
esp_err_t drv8833_ramp_to_speed(drv8833_dev_t *dev, drv8833_channel_t ch,
                                 int8_t target_pct, uint32_t ramp_ms);

/**
 * @brief  Apply active brake on one channel (both IN pins driven high).
 *
 * @param  dev  Pointer to an initialised drv8833_dev_t.
 * @param  ch   DRV8833_CHANNEL_A or DRV8833_CHANNEL_B.
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if dev is NULL.
 */
esp_err_t drv8833_brake(drv8833_dev_t *dev, drv8833_channel_t ch);

/**
 * @brief  Control the nSLEEP pin.
 *
 * No-op (returns ESP_OK) when nsleep_gpio was configured as -1.
 *
 * @param  dev    Pointer to an initialised drv8833_dev_t.
 * @param  sleep  true  → pull nSLEEP low  (device enters low-power sleep).
 *                false → pull nSLEEP high (device awake, default after init).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if dev is NULL.
 */
esp_err_t drv8833_sleep(drv8833_dev_t *dev, bool sleep);

/**
 * @brief  Read the FAULT pin state.
 *
 * @param  dev  Pointer to an initialised drv8833_dev_t.
 * @return true if a fault is asserted (pin low), false otherwise or when
 *         fault_gpio was configured as -1.
 */
bool drv8833_fault(drv8833_dev_t *dev);

/**
 * @brief  Coast both channels and stop all LEDC outputs.
 *
 * Does not free the drv8833_dev_t — the caller owns the storage.
 *
 * @param  dev  Pointer to an initialised drv8833_dev_t. No-op if NULL.
 */
void drv8833_deinit(drv8833_dev_t *dev);

#ifdef __cplusplus
}
#endif
