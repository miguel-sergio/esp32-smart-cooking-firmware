#include "test_drv8833.h"

#include "drv8833.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "test_drv8833";

/* --------------------------------------------------------------------- *
 * Board pin config — adjust to match your wiring                        *
 * --------------------------------------------------------------------- *
 *  Wiring (M1):                                                         *
 *    GPIO_PWM ──► IN1  (speed, fast-decay PWM)                         *
 *    GND      ──► IN2  (fixed LOW — direction is fixed, no reverse)    *
 *    3.3V     ──► EEP  (always awake — not ESP32-controlled)           *
 *    IN3, IN4, OUT3, OUT4 not connected in M1                           *
 *                                                                       *
 *  ← Set PIN_AIN1 to your actual GPIO_PWM pin number.                  *
 *  PIN_AIN2 is a dummy: IN2 is hardwired to GND on the board, but      *
 *  the driver requires a GPIO for each IN pin. Assign any free          *
 *  unconnected ESP32 GPIO here; it will never be wired to the DRV8833. *
 *  Same applies to PIN_BIN1 / PIN_BIN2 (Channel B unused in M1).       *
 * --------------------------------------------------------------------- */
#define PIN_AIN1    25    /* ← your GPIO_PWM pin                        */
#define PIN_AIN2    26    /* dummy — IN2 is hardwired to GND on board    */
#define PIN_BIN1    27    /* dummy — IN3 not connected in M1             */
#define PIN_BIN2    14    /* dummy — IN4 not connected in M1             */
#define PIN_NSLEEP  (-1)  /* EEP wired to 3.3V — always awake           */
#define PIN_FAULT   (-1)  /* FAULT not connected                         */

#define PWM_FREQ_HZ 20000

esp_err_t test_drv8833_run(void) {
    ESP_LOGI(TAG, "--- DRV8833 validation START ---");

    drv8833_config_t cfg = {
        .ain1_gpio    = PIN_AIN1,
        .ain2_gpio    = PIN_AIN2,
        .ain1_ledc_ch = LEDC_CHANNEL_0,
        .ain2_ledc_ch = LEDC_CHANNEL_1,
        .bin1_gpio    = PIN_BIN1,
        .bin2_gpio    = PIN_BIN2,
        .bin1_ledc_ch = LEDC_CHANNEL_2,
        .bin2_ledc_ch = LEDC_CHANNEL_3,
        .ledc_timer   = LEDC_TIMER_0,
        .pwm_freq_hz  = PWM_FREQ_HZ,
        .nsleep_gpio  = PIN_NSLEEP,
        .fault_gpio   = PIN_FAULT,
    };

    drv8833_dev_t dev = {0};
    esp_err_t ret = drv8833_init(&cfg, &dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ---- Channel A: variable speed forward then coast ---- *
     * IN2 is hardwired to GND — reverse and brake are not    *
     * possible with this wiring. Channel B is not connected. */

    ESP_LOGI(TAG, "Channel A: ramp 0 -> 100%% in 500 ms");
    ret = drv8833_ramp_to_speed(&dev, DRV8833_CHANNEL_A, 100, 500);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP_LOGI(TAG, "Channel A: ramp 100 -> 50%% in 300 ms");
        ret = drv8833_ramp_to_speed(&dev, DRV8833_CHANNEL_A, 50, 300);
    }
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        ESP_LOGI(TAG, "Channel A: ramp 50 -> 0%% (coast) in 500 ms");
        ret = drv8833_ramp_to_speed(&dev, DRV8833_CHANNEL_A, 0, 500);
    }
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    drv8833_deinit(&dev);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "--- DRV8833 validation PASS ---");
    } else {
        ESP_LOGE(TAG, "--- DRV8833 validation FAIL: %s ---", esp_err_to_name(ret));
    }
    return ret;
}
