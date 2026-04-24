#include "test_bme280.h"

#include "bme280.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "test_bme280";

/* BME280 datasheet §1 — absolute operating limits */
#define TEMP_MIN_C    -40.0f
#define TEMP_MAX_C     85.0f
#define HUM_MIN_PCT     0.0f
#define HUM_MAX_PCT   100.0f
#define PRESS_MIN_HPA 300.0f
#define PRESS_MAX_HPA 1100.0f

#define SAMPLE_COUNT  5

esp_err_t test_bme280_run(i2c_master_bus_handle_t bus) {
    ESP_LOGI(TAG, "--- BME280 validation START ---");

    bme280_dev_t *dev = NULL;
    esp_err_t ret = bme280_init(bus, BME280_I2C_ADDR_DEFAULT, &dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_err_t result = ESP_OK;

    /* Discard first sample — sensor output may be unstable immediately
     * after entering normal mode (internal filter not yet settled). */
    bme280_data_t discard;
    bme280_read(dev, &discard);
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        bme280_data_t d;
        ret = bme280_read(dev, &d);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[%d] read failed: %s", i, esp_err_to_name(ret));
            result = ret;
            break;
        }

        ESP_LOGI(TAG, "[%d] T=%.2f°C  H=%.2f%%  P=%.2f hPa",
                 i, d.temperature, d.humidity, d.pressure);

        bool ok = (d.temperature >= TEMP_MIN_C  && d.temperature <= TEMP_MAX_C)
               && (d.humidity    >= HUM_MIN_PCT  && d.humidity    <= HUM_MAX_PCT)
               && (d.pressure    >= PRESS_MIN_HPA && d.pressure   <= PRESS_MAX_HPA);

        if (!ok) {
            ESP_LOGE(TAG, "[%d] reading out of range", i);
            result = ESP_FAIL;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }

    bme280_deinit(dev);

    if (result == ESP_OK) {
        ESP_LOGI(TAG, "--- BME280 validation PASS ---");
    } else {
        ESP_LOGE(TAG, "--- BME280 validation FAIL ---");
    }

    return result;
}
