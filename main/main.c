/* PoC: 1 Hz BME280 sensor read + DRV8833 motor at fixed duty via FreeRTOS task */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"

#include "bme280.h"
#include "drv8833.h"

static const char *TAG = "poc";

/* ── Pin config ─────────────────────────────────────────────────────────── */
#define PIN_SDA         21
#define PIN_SCL         22
#define BME280_ADDR     BME280_I2C_ADDR_DEFAULT

#define PIN_AIN1        25
#define PIN_AIN2        26    /* dummy — IN2 hardwired GND on board */
#define PIN_BIN1        27    /* dummy — channel B unused            */
#define PIN_BIN2        14    /* dummy — channel B unused            */
#define MOTOR_DUTY_PCT  50    /* cruise duty for the PoC             */
#define MOTOR_RAMP_MS   500   /* ramp duration per phase             */

/* ── Shared device handles ──────────────────────────────────────────────── */
static bme280_dev_t  s_bme;
static drv8833_dev_t s_motor;

/* ── Sensor task ─────────────────────────────────────────────────────────
 * Reads BME280 at 1 Hz and logs T / H / P to UART.
 * Motor is started once at init and runs independently of this task.
 * ----------------------------------------------------------------------- */
static void sensor_task(void *arg) {
    (void)arg;
    bme280_data_t data;

    while (1) {
        esp_err_t ret = bme280_read(&s_bme, &data);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "T: %.1f °C  H: %.1f %%  P: %.1f hPa",
                     data.temperature, data.humidity, data.pressure);
        } else {
            ESP_LOGW(TAG, "bme280_read failed: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ── app_main ────────────────────────────────────────────────────────────── */
void app_main(void) {
    /* --- I2C bus -------------------------------------------------------- */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = PIN_SDA,
        .scl_io_num        = PIN_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    /* --- BME280 --------------------------------------------------------- */
    ESP_ERROR_CHECK(bme280_init(i2c_bus, BME280_ADDR, &s_bme));
    ESP_LOGI(TAG, "BME280 initialised");

    /* --- DRV8833 -------------------------------------------------------- */
    drv8833_config_t motor_cfg = {
        .ain1_gpio    = PIN_AIN1,
        .ain2_gpio    = PIN_AIN2,
        .ain1_ledc_ch = LEDC_CHANNEL_0,
        .ain2_ledc_ch = LEDC_CHANNEL_1,
        .bin1_gpio    = PIN_BIN1,
        .bin2_gpio    = PIN_BIN2,
        .bin1_ledc_ch = LEDC_CHANNEL_2,
        .bin2_ledc_ch = LEDC_CHANNEL_3,
        .ledc_timer   = LEDC_TIMER_0,
        .pwm_freq_hz  = 20000,
        .nsleep_gpio  = -1,
        .fault_gpio   = -1,
    };
    ESP_ERROR_CHECK(drv8833_init(&motor_cfg, &s_motor));
    /* Kick to 100 % first to overcome static friction, then settle at cruise */
    ESP_ERROR_CHECK(drv8833_ramp_to_speed(&s_motor, DRV8833_CHANNEL_A, 100, MOTOR_RAMP_MS));
    ESP_ERROR_CHECK(drv8833_ramp_to_speed(&s_motor, DRV8833_CHANNEL_A,
                                          MOTOR_DUTY_PCT, MOTOR_RAMP_MS));
    ESP_LOGI(TAG, "Motor at cruise %d %%", MOTOR_DUTY_PCT);

    /* --- Start sensor task --------------------------------------------- */
    BaseType_t ret = xTaskCreatePinnedToCore(
        sensor_task,                 /* task function                    */
        "sensor",                    /* name                             */
        (4096 / sizeof(StackType_t)),/* stack words (4096 bytes total)   */
        NULL,                        /* arg                              */
        5,                           /* priority                         */
        NULL,                        /* handle                           */
        1                            /* core — real-time tasks on Core 1 */
    );
    configASSERT(ret == pdPASS);
}
