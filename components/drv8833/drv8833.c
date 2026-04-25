#include "drv8833.h"

#include "esp_log.h"
#include "esp_check.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "drv8833";

/* 10-bit PWM resolution: duty range 0–1023 */
#define PWM_RESOLUTION  LEDC_TIMER_10_BIT
#define PWM_DUTY_MAX    ((1u << 10) - 1)   /* 1023 */

/* Ramp interpolation step interval */
#define RAMP_STEP_MS    10u

/* --------------------------------------------------------------------------
 * LEDC helper
 * -------------------------------------------------------------------------- */
static esp_err_t set_duty(ledc_channel_t ch, uint32_t duty) {
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty),
                        TAG, "ledc_set_duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch),
                        TAG, "ledc_update_duty failed");
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */
esp_err_t drv8833_init(const drv8833_config_t *cfg, drv8833_dev_t *dev) {
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "cfg is NULL");
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");

    dev->ain1_ch     = cfg->ain1_ledc_ch;
    dev->ain2_ch     = cfg->ain2_ledc_ch;
    dev->bin1_ch     = cfg->bin1_ledc_ch;
    dev->bin2_ch     = cfg->bin2_ledc_ch;
    dev->timer       = cfg->ledc_timer;
    dev->nsleep_gpio  = cfg->nsleep_gpio;
    dev->fault_gpio   = cfg->fault_gpio;
    dev->cur_speed[0] = 0;
    dev->cur_speed[1] = 0;

    /* --- LEDC timer --- */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num       = cfg->ledc_timer,
        .freq_hz         = cfg->pwm_freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg),
                        TAG, "ledc_timer_config failed");

    /* --- 4 LEDC channels — all start at duty 0 (coast) --- */
    const struct { int gpio; ledc_channel_t ch; } chs[4] = {
        { cfg->ain1_gpio, cfg->ain1_ledc_ch },
        { cfg->ain2_gpio, cfg->ain2_ledc_ch },
        { cfg->bin1_gpio, cfg->bin1_ledc_ch },
        { cfg->bin2_gpio, cfg->bin2_ledc_ch },
    };
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = chs[i].gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = chs[i].ch,
            .timer_sel  = cfg->ledc_timer,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg),
                            TAG, "ledc_channel_config[%d] failed", i);
    }

    /* --- nSLEEP: output, default high (device awake) --- */
    if (cfg->nsleep_gpio >= 0) {
        gpio_config_t gpio_cfg = {
            .pin_bit_mask = (1ULL << cfg->nsleep_gpio),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&gpio_cfg),
                            TAG, "nsleep gpio_config failed");
        ESP_RETURN_ON_ERROR(gpio_set_level(cfg->nsleep_gpio, 1),
                            TAG, "nsleep set_level failed");
    }

    /* --- FAULT: input, no internal pull (board has external pull-up) --- */
    if (cfg->fault_gpio >= 0) {
        gpio_config_t gpio_cfg = {
            .pin_bit_mask = (1ULL << cfg->fault_gpio),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&gpio_cfg),
                            TAG, "fault gpio_config failed");
    }

    return ESP_OK;
}

esp_err_t drv8833_set_speed(drv8833_dev_t *dev, drv8833_channel_t ch, int8_t speed_pct) {
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");
    ESP_RETURN_ON_FALSE(ch == DRV8833_CHANNEL_A || ch == DRV8833_CHANNEL_B,
                        ESP_ERR_INVALID_ARG, TAG, "ch is invalid");
    ESP_RETURN_ON_FALSE(speed_pct >= -100 && speed_pct <= 100,
                        ESP_ERR_INVALID_ARG, TAG, "speed_pct out of range [-100, 100]");

    ledc_channel_t in1 = (ch == DRV8833_CHANNEL_A) ? dev->ain1_ch : dev->bin1_ch;
    ledc_channel_t in2 = (ch == DRV8833_CHANNEL_A) ? dev->ain2_ch : dev->bin2_ch;
    uint32_t duty = ((uint32_t)(speed_pct < 0 ? -speed_pct : speed_pct) * PWM_DUTY_MAX) / 100;

    if (speed_pct > 0) {
        ESP_RETURN_ON_ERROR(set_duty(in1, duty), TAG, "set in1 duty failed");
        ESP_RETURN_ON_ERROR(set_duty(in2,    0), TAG, "set in2 duty failed");
    } else if (speed_pct < 0) {
        ESP_RETURN_ON_ERROR(set_duty(in1,    0), TAG, "set in1 duty failed");
        ESP_RETURN_ON_ERROR(set_duty(in2, duty), TAG, "set in2 duty failed");
    } else {
        /* coast — both pins low */
        ESP_RETURN_ON_ERROR(set_duty(in1, 0), TAG, "set in1 duty failed");
        ESP_RETURN_ON_ERROR(set_duty(in2, 0), TAG, "set in2 duty failed");
    }
    dev->cur_speed[ch] = speed_pct;
    return ESP_OK;
}

esp_err_t drv8833_ramp_to_speed(drv8833_dev_t *dev, drv8833_channel_t ch,
                                 int8_t target_pct, uint32_t ramp_ms) {
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");
    ESP_RETURN_ON_FALSE(ch == DRV8833_CHANNEL_A || ch == DRV8833_CHANNEL_B,
                        ESP_ERR_INVALID_ARG, TAG, "ch is invalid");
    ESP_RETURN_ON_FALSE(target_pct >= -100 && target_pct <= 100,
                        ESP_ERR_INVALID_ARG, TAG, "target_pct out of range [-100, 100]");

    if (ramp_ms == 0) {
        return drv8833_set_speed(dev, ch, target_pct);
    }

    int8_t  start     = dev->cur_speed[ch];
    int     diff      = (int)target_pct - (int)start;
    uint32_t intervals = (ramp_ms + RAMP_STEP_MS - 1u) / RAMP_STEP_MS;
    uint32_t steps     = intervals + 1u;

    for (uint32_t step = 1; step <= steps; step++) {
        int8_t v = (int8_t)(start + diff * (int)step / (int)steps);
        ESP_RETURN_ON_ERROR(drv8833_set_speed(dev, ch, v),
                            TAG, "ramp set_speed failed at step %"PRIu32, step);
        if (step < steps) {
            uint32_t elapsed_before = (ramp_ms * (step - 1u)) / intervals;
            uint32_t elapsed_after  = (ramp_ms * step) / intervals;
            vTaskDelay(pdMS_TO_TICKS(elapsed_after - elapsed_before));
        }
    }
    return ESP_OK;
}

esp_err_t drv8833_brake(drv8833_dev_t *dev, drv8833_channel_t ch) {
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");
    ESP_RETURN_ON_FALSE(ch == DRV8833_CHANNEL_A || ch == DRV8833_CHANNEL_B,
                        ESP_ERR_INVALID_ARG, TAG, "ch is invalid");

    ledc_channel_t in1 = (ch == DRV8833_CHANNEL_A) ? dev->ain1_ch : dev->bin1_ch;
    ledc_channel_t in2 = (ch == DRV8833_CHANNEL_A) ? dev->ain2_ch : dev->bin2_ch;
    ESP_RETURN_ON_ERROR(set_duty(in1, PWM_DUTY_MAX), TAG, "set in1 duty failed");
    ESP_RETURN_ON_ERROR(set_duty(in2, PWM_DUTY_MAX), TAG, "set in2 duty failed");
    return ESP_OK;
}

esp_err_t drv8833_sleep(drv8833_dev_t *dev, bool sleep) {
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "dev is NULL");
    if (dev->nsleep_gpio < 0) return ESP_OK;
    return gpio_set_level(dev->nsleep_gpio, sleep ? 0 : 1);
}

bool drv8833_fault(drv8833_dev_t *dev) {
    if (!dev || dev->fault_gpio < 0) return false;
    return gpio_get_level(dev->fault_gpio) == 0; /* active low */
}

void drv8833_deinit(drv8833_dev_t *dev) {
    if (!dev) return;
    /* Coast all channels before releasing */
    ledc_channel_t all_chs[4] = { dev->ain1_ch, dev->ain2_ch,
                                   dev->bin1_ch, dev->bin2_ch };
    for (int i = 0; i < 4; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, all_chs[i], 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, all_chs[i]);
        ledc_stop(LEDC_LOW_SPEED_MODE, all_chs[i], 0);
    }
    if (dev->nsleep_gpio >= 0) {
        gpio_set_level(dev->nsleep_gpio, 0); /* pull low before releasing */
    }
}
