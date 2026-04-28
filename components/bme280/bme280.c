#include "bme280.h"

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bme280";

/* --------------------------------------------------------------------------
 * Register map
 * -------------------------------------------------------------------------- */
#define REG_ID          0xD0
#define REG_RESET       0xE0
#define REG_STATUS      0xF3  /* bit0: im_update (NVM loading), bit3: measuring */
#define REG_CTRL_HUM    0xF2
#define REG_CTRL_MEAS   0xF4
#define REG_DATA        0xF7  /* 8 bytes: press(3) + temp(3) + hum(2) */
#define REG_CALIB1      0x88  /* 26 bytes: T1-T3, P1-P9, H1           */
#define REG_CALIB2      0xE1  /* 7  bytes: H2-H6                       */

#define CHIP_ID         0x60
#define RESET_VAL       0xB6
#define I2C_TIMEOUT_MS  (-1)  /* block until done */

/* --------------------------------------------------------------------------
 * I2C helpers
 * -------------------------------------------------------------------------- */
static esp_err_t reg_write(bme280_dev_t *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev->i2c_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t reg_read(bme280_dev_t *dev, uint8_t reg, uint8_t *dst, size_t len) {
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, dst, len, I2C_TIMEOUT_MS);
}

/* --------------------------------------------------------------------------
 * Calibration
 * -------------------------------------------------------------------------- */
/* Little-endian 16-bit helpers */
#define LE16U(b, i)  ((uint16_t)((uint8_t)(b)[(i)+1] << 8 | (uint8_t)(b)[i]))
#define LE16S(b, i)  ((int16_t) LE16U(b, i))

static void load_calib(bme280_calib_t *c, const uint8_t *b1, const uint8_t *b2) {
    c->T1 = LE16U(b1,  0); c->T2 = LE16S(b1,  2); c->T3 = LE16S(b1,  4);
    c->P1 = LE16U(b1,  6); c->P2 = LE16S(b1,  8); c->P3 = LE16S(b1, 10);
    c->P4 = LE16S(b1, 12); c->P5 = LE16S(b1, 14); c->P6 = LE16S(b1, 16);
    c->P7 = LE16S(b1, 18); c->P8 = LE16S(b1, 20); c->P9 = LE16S(b1, 22);
    c->H1 = b1[25]; /* 0xA1 — byte 25 of the 26-byte read from 0x88 */

    c->H2 = LE16S(b2, 0);
    c->H3 = b2[2];
    /* 0xE4[7:0] = H4[11:4], 0xE5[3:0] = H4[3:0] — cast to int8_t first for sign extension.
     * Outer (int16_t) truncates the int result of the OR back to 16 bits — value fits. */
    c->H4 = (int16_t)(((int16_t)(int8_t)b2[3] << 4) | (b2[4] & 0x0F));
    /* 0xE6[7:0] = H5[11:4], 0xE5[7:4] = H5[3:0] */
    c->H5 = (int16_t)(((int16_t)(int8_t)b2[5] << 4) | (b2[4] >> 4));
    c->H6 = (int8_t)b2[6];
}

#undef LE16U
#undef LE16S

/* --------------------------------------------------------------------------
 * Compensation formulas — integer arithmetic, BME280 datasheet §4.2.3
 * -------------------------------------------------------------------------- */

/* Returns temperature in units of 0.01 °C; sets t_fine used by P and H. */
static int32_t comp_temp(const bme280_calib_t *c, int32_t adc, int32_t *t_fine) {
    int32_t v1 = ((((adc >> 3) - ((int32_t)c->T1 << 1))) * c->T2) >> 11;
    int32_t v2 = (((((adc >> 4) - (int32_t)c->T1) *
                    ((adc >> 4) - (int32_t)c->T1)) >> 12) * c->T3) >> 14;
    *t_fine = v1 + v2;
    return (*t_fine * 5 + 128) >> 8;
}

/* Returns pressure in Q24.8 Pa (divide by 256 to get Pa, divide by 25600 for hPa). */
static uint32_t comp_pressure(const bme280_calib_t *c, int32_t adc, int32_t t_fine) {
    int64_t v1 = (int64_t)t_fine - 128000;
    int64_t v2 = v1 * v1 * c->P6;
    v2 += (v1 * c->P5) << 17;
    v2 += (int64_t)c->P4 << 35;
    v1  = ((v1 * v1 * c->P3) >> 8) + ((v1 * c->P2) << 12);
    v1  = ((((int64_t)1 << 47) + v1) * c->P1) >> 33;
    if (v1 == 0) return 0; /* avoid division by zero */
    int64_t p = 1048576 - adc;
    p  = (((p << 31) - v2) * 3125) / v1;
    v1 = ((int64_t)c->P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)c->P8 * p) >> 19;
    return (uint32_t)(((p + v1 + v2) >> 8) + ((int64_t)c->P7 << 4));
}

/* Returns humidity in Q22.10 %RH (divide by 1024 to get %RH). */
static uint32_t comp_humidity(const bme280_calib_t *c, int32_t adc, int32_t t_fine) {
    int32_t v = t_fine - 76800;
    v = (((adc << 14) - ((int32_t)c->H4 << 20) - (c->H5 * v) + 16384) >> 15)
        * (((((((v * c->H6) >> 10)
               * (((v * c->H3) >> 11) + 32768)) >> 10) + 2097152)
             * c->H2 + 8192) >> 14);
    v -= (((((v >> 15) * (v >> 15)) >> 7) * c->H1) >> 4);
    if (v < 0)         v = 0;
    if (v > 419430400) v = 419430400;
    return (uint32_t)(v >> 12);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

/* Configure the sensor after the I2C device handle has been added.
 * All failures propagate upward; cleanup of i2c_dev is handled by the caller's
 * error path (currently bme280_init()), not by this helper. */
static esp_err_t bme280_configure(bme280_dev_t *dev) {
    uint8_t id;
    ESP_RETURN_ON_ERROR(reg_read(dev, REG_ID, &id, 1),
                        TAG, "read chip-id failed");
    ESP_RETURN_ON_FALSE(id == CHIP_ID, ESP_ERR_NOT_FOUND,
                        TAG, "unexpected chip-id 0x%02x (expected 0x%02x)", id, CHIP_ID);

    ESP_RETURN_ON_ERROR(reg_write(dev, REG_RESET, RESET_VAL),
                        TAG, "soft reset failed");

    /* Poll im_update (bit0 of status) until NVM data is fully loaded.
     * Fixed delay is unreliable; polling is the correct approach. */
    uint8_t status;
    int retries = 20;
    do {
        vTaskDelay(pdMS_TO_TICKS(2));
        ESP_RETURN_ON_ERROR(reg_read(dev, REG_STATUS, &status, 1),
                            TAG, "read status failed");
    } while ((status & 0x01) && --retries);
    ESP_RETURN_ON_FALSE(retries, ESP_ERR_TIMEOUT,
                        TAG, "NVM load timeout after reset");

    uint8_t c1[26], c2[7];
    ESP_RETURN_ON_ERROR(reg_read(dev, REG_CALIB1, c1, sizeof(c1)),
                        TAG, "read calib block 1 failed");
    ESP_RETURN_ON_ERROR(reg_read(dev, REG_CALIB2, c2, sizeof(c2)),
                        TAG, "read calib block 2 failed");
    load_calib(&dev->calib, c1, c2);

    /* ctrl_hum changes only take effect after writing ctrl_meas (per datasheet) */
    ESP_RETURN_ON_ERROR(reg_write(dev, REG_CTRL_HUM,  0x01), TAG, "write ctrl_hum");
    ESP_RETURN_ON_ERROR(reg_write(dev, REG_CTRL_MEAS, 0x27), TAG, "write ctrl_meas");
    /* 0x27 = osrs_t×1 | osrs_p×1 | mode=normal */

    /* Wait for the first measurement to complete (~10 ms at ×1 oversampling) */
    vTaskDelay(pdMS_TO_TICKS(15));
    return ESP_OK;
}

esp_err_t bme280_init(i2c_master_bus_handle_t bus, uint8_t addr, bme280_dev_t *dev) {
    ESP_RETURN_ON_FALSE(bus, ESP_ERR_INVALID_ARG, TAG, "bus is NULL");
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");

    dev->i2c_dev = NULL;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &dev->i2c_dev),
                        TAG, "i2c_master_bus_add_device failed");

    esp_err_t ret = bme280_configure(dev);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(dev->i2c_dev);
        dev->i2c_dev = NULL;
        return ret;
    }

    return ESP_OK;
}

esp_err_t bme280_read(bme280_dev_t *dev, bme280_data_t *out) {
    ESP_RETURN_ON_FALSE(dev && out, ESP_ERR_INVALID_ARG, TAG, "Invalid argument");

    uint8_t raw[8];
    ESP_RETURN_ON_ERROR(reg_read(dev, REG_DATA, raw, sizeof(raw)), TAG, "read data failed");

    int32_t adc_p = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_t = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_h = ((int32_t)raw[6] <<  8) |  raw[7];

    int32_t  t_fine;
    int32_t  t_raw = comp_temp    (&dev->calib, adc_t, &t_fine);
    uint32_t p_raw = comp_pressure(&dev->calib, adc_p, t_fine);
    uint32_t h_raw = comp_humidity(&dev->calib, adc_h, t_fine);

    out->temperature = (float)t_raw / 100.0f;           /* 0.01 °C  → °C   */
    out->pressure    = (float)p_raw / 256.0f / 100.0f;  /* Q24.8 Pa → hPa  */
    out->humidity    = (float)h_raw / 1024.0f;          /* Q22.10   → %RH  */
    return ESP_OK;
}

void bme280_deinit(bme280_dev_t *dev) {
    if (!dev || !dev->i2c_dev) return;
    i2c_master_bus_rm_device(dev->i2c_dev);
    dev->i2c_dev = NULL;
}
