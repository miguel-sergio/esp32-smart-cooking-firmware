#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

/* --- I2C drivers --- */
#include "test_bme280.h"
/* --- PWM / GPIO drivers --- */
#include "test_drv8833.h"

static const char *TAG = "drv-validation";

/* -----------------------------------------------------------------------
 * Board pin config — adjust to match your wiring
 * --------------------------------------------------------------------- */
#define I2C_PORT   I2C_NUM_0
#define PIN_SDA    21
#define PIN_SCL    22

#define RUN_TEST(name, call)                                        \
    do {                                                            \
        ESP_LOGI(TAG, ">> Running: %s", name);                     \
        if ((call) != ESP_OK) {                                     \
            ESP_LOGE(TAG, "   FAIL: %s", name);                    \
            failures++;                                             \
        } else {                                                    \
            ESP_LOGI(TAG, "   PASS: %s", name);                    \
        }                                                           \
    } while (0)

void app_main(void) {
    int failures = 0;

    /* ------------------------------------------------------------------ *
     * I2C tests — shared bus, one init for all I2C devices               *
     * ------------------------------------------------------------------ */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_PORT,
        .sda_io_num        = PIN_SDA,
        .scl_io_num        = PIN_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    RUN_TEST("bme280", test_bme280_run(i2c_bus));

    i2c_del_master_bus(i2c_bus);

    /* ------------------------------------------------------------------ *
     * PWM / GPIO tests — standalone, no shared bus                       *
     * ------------------------------------------------------------------ */
    RUN_TEST("drv8833", test_drv8833_run());

    /* ------------------------------------------------------------------ *
     * Summary                                                             *
     * ------------------------------------------------------------------ */
    if (failures == 0) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  ALL DRIVER TESTS PASSED");
        ESP_LOGI(TAG, "========================================");
    } else {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "  %d DRIVER TEST(S) FAILED", failures);
        ESP_LOGE(TAG, "========================================");
    }
}
