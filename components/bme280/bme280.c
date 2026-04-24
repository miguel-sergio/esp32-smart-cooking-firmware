#include "bme280.h"

#include <stdlib.h>
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
 * Internal types
 * -------------------------------------------------------------------------- */
typedef struct {
    uint16_t T1; int16_t T2; int16_t T3;
    uint16_t P1; int16_t P2; int16_t P3; int16_t P4;
    int16_t  P5; int16_t P6; int16_t P7; int16_t P8; int16_t P9;
    uint8_t  H1; int16_t H2; uint8_t  H3;
    int16_t  H4; int16_t H5; int8_t   H6;
} calib_t;

struct bme280_dev {
    i2c_master_dev_handle_t i2c_dev;
    calib_t                 calib;
};

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

static void load_calib(calib_t *c, const uint8_t *b1, const uint8_t *b2) {
    c->T1 = LE16U(b1,  0); c->T2 = LE16S(b1,  2); c->T3 = LE16S(b1,  4);
    c->P1 = LE16U(b1,  6); c->P2 = LE16S(b1,  8); c->P3 = LE16S(b1, 10);
    c->P4 = LE16S(b1, 12); c->P5 = LE16S(b1, 14); c->P6 = LE16S(b1, 16);
    c->P7 = LE16S(b1, 18); c->P8 = LE16S(b1, 20); c->P9 = LE16S(b1, 22);
    c->H1 = b1[25]; /* 0xA1 — byte 25 of the 26-byte read from 0x88 */

    c->H2 = LE16S(b2, 0);
    c->H3 = b2[2];
    /* 0xE4[7:0] = H4[11:4], 0xE5[3:0] = H4[3:0] — cast to int8_t first for sign extension */
    c->H4 = ((int16_t)(int8_t)b2[3] << 4) | (b2[4] & 0x0F);
    /* 0xE6[7:0] = H5[11:4], 0xE5[7:4] = H5[3:0] */
    c->H5 = ((int16_t)(int8_t)b2[5] << 4) | (b2[4] >> 4);
    c->H6 = (int8_t)b2[6];
}

#undef LE16U
#undef LE16S

/* --------------------------------------------------------------------------
 * Compensation formulas — integer arithmetic, BME280 datasheet §4.2.3
 * -------------------------------------------------------------------------- */

/* Returns temperature in units of 0.01 °C; sets t_fine used by P and H. */
static int32_t comp_temp(const calib_t *c, int32_t adc, int32_t *t_fine) {
    int32_t v1 = ((((adc >> 3) - ((int32_t)c->T1 << 1))) * c->T2) >> 11;
    int32_t v2 = (((((adc >> 4) - (int32_t)c->T1) *
                    ((adc >> 4) - (int32_t)c->T1)) >> 12) * c->T3) >> 14;
    *t_fine = v1 + v2;
    return (*t_fine * 5 + 128) >> 8;
}

/* Returns pressure in Q24.8 Pa (divide by 256 to get Pa, divide by 25600 for hPa). */
static uint32_t comp_pressure(const calib_t *c, int32_t adc, int32_t t_fine) {
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
static uint32_t comp_humidity(const calib_t *c, int32_t adc, int32_t t_fine) {
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
esp_err_t bme280_init(i2c_master_bus_handle_t bus, uint8_t addr, bme280_dev_t **out_dev) {
    esp_err_t ret;

    bme280_dev_t *dev = calloc(1, sizeof(*dev));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "heap alloc failed");

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 400000,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &dev->i2c_dev),
                      err_free, TAG, "i2c_master_bus_add_device failed");

    uint8_t id;
    ESP_GOTO_ON_ERROR(reg_read(dev, REG_ID, &id, 1),
                      err_dev, TAG, "read chip-id failed");
    ESP_GOTO_ON_FALSE(id == CHIP_ID, ESP_ERR_NOT_FOUND, err_dev,
                      TAG, "unexpected chip-id 0x%02x (expected 0x%02x)", id, CHIP_ID);

    ESP_GOTO_ON_ERROR(reg_write(dev, REG_RESET, RESET_VAL),
                      err_dev, TAG, "soft reset failed");

    /* Poll im_update (bit0 of status) until NVM data is fully loaded.
     * Fixed delay is unreliable; polling is the correct approach. */
    uint8_t status;
    int retries = 20;
    do {
        vTaskDelay(pdMS_TO_TICKS(2));
        ESP_GOTO_ON_ERROR(reg_read(dev, REG_STATUS, &status, 1),
                          err_dev, TAG, "read status failed");
    } while ((status & 0x01) && --retries);
    ESP_GOTO_ON_FALSE(retries, ESP_ERR_TIMEOUT, err_dev,
                      TAG, "NVM load timeout after reset");

    uint8_t c1[26], c2[7];
    ESP_GOTO_ON_ERROR(reg_read(dev, REG_CALIB1, c1, sizeof(c1)),
                      err_dev, TAG, "read calib block 1 failed");
    ESP_GOTO_ON_ERROR(reg_read(dev, REG_CALIB2, c2, sizeof(c2)),
                      err_dev, TAG, "read calib block 2 failed");
    load_calib(&dev->calib, c1, c2);

    /* ctrl_hum changes only take effect after writing ctrl_meas (per datasheet) */
    ESP_GOTO_ON_ERROR(reg_write(dev, REG_CTRL_HUM,  0x01), err_dev, TAG, "write ctrl_hum");
    ESP_GOTO_ON_ERROR(reg_write(dev, REG_CTRL_MEAS, 0x27), err_dev, TAG, "write ctrl_meas");
    /* 0x27 = osrs_t×1 | osrs_p×1 | mode=normal */

    /* Wait for the first measurement to complete (~10 ms at ×1 oversampling) */
    vTaskDelay(pdMS_TO_TICKS(15));

    *out_dev = dev;
    return ESP_OK;

err_dev:
    i2c_master_bus_rm_device(dev->i2c_dev);
err_free:
    free(dev);
    return ret;
}

esp_err_t bme280_read(bme280_dev_t *dev, bme280_data_t *out) {
    uint8_t raw[8];
    ESP_RETURN_ON_ERROR(reg_read(dev, REG_DATA, raw, sizeof(raw)), TAG, "read data failed");

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = ((int32_t)raw[6] <<  8) |  raw[7];

    int32_t  t_fine;
    int32_t  T = comp_temp    (&dev->calib, adc_T, &t_fine);
    uint32_t P = comp_pressure(&dev->calib, adc_P, t_fine);
    uint32_t H = comp_humidity(&dev->calib, adc_H, t_fine);

    out->temperature = T / 100.0f;           /* 0.01 °C  → °C   */
    out->pressure    = P / 256.0f / 100.0f;  /* Q24.8 Pa → hPa  */
    out->humidity    = H / 1024.0f;          /* Q22.10   → %RH  */
    return ESP_OK;
}

void bme280_deinit(bme280_dev_t *dev) {
    if (!dev) return;
    i2c_master_bus_rm_device(dev->i2c_dev);
    free(dev);
}
