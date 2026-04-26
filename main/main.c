#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "bme280.h"
#include "app_types.h"
#include "control_task.h"
#include "thermal_task.h"
#include "motor_task.h"
#include "comms_task.h"
#if CONFIG_SMART_COOKING_CLI
#include "cli_task.h"
#endif

static const char *TAG = "main";

/* ── Pin configuration ──────────────────────────────────────────────────
 * Centralised pin map for the whole application.
 * ----------------------------------------------------------------------- */
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_RELAY      32
#define BME280_ADDR    BME280_I2C_ADDR_DEFAULT

/* DRV8833 — Channel A drives the stirrer motor
 * AIN2 is hardwired to GND on the PCB — motor is unidirectional (forward/coast only).
 * No active brake or reverse is available on this channel.
 * AIN1 is driven by LEDC for speed control; AIN2 is not driven by the ESP32. */
#define PIN_AIN1       25
#define PIN_AIN2       26   /* dummy — IN2 is hardwired to GND on board; not wired to this GPIO */
#define PIN_BIN1       27   /* unused channel B — reserved */
#define PIN_BIN2       14   /* unused channel B — reserved */
#define PIN_NSLEEP     (-1) /* not connected */
#define PIN_FAULT      (-1) /* not connected */

/* ── Inter-task queues ───────────────────────────────────────────────────
 * Created once in app_main and passed by pointer to each task at start.
 * Queue ownership stays in main; tasks never create or delete queues.
 * ----------------------------------------------------------------------- */
static QueueHandle_t s_temp_q;    /* thermal_task → control_task            */
static QueueHandle_t s_cmd_q;     /* comms_task   → control_task            */
static QueueHandle_t s_motor_q;   /* control_task → motor_task              */
static QueueHandle_t s_state_q;   /* control_task → comms_task              */
static QueueHandle_t s_thermal_q; /* control_task → thermal_task            */

/* ── app_main ─────────────────────────────────────────────────────────── */
void app_main(void) {
    /* ── NVS (required by Wi-Fi) ────────────────────────────────────────── */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    /* ── I2C bus ─────────────────────────────────────────────────── */
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
    ESP_LOGI(TAG, "I2C bus initialised");

    /* ── Create queues ─────────────────────────────────────────────────── */
    s_temp_q    = xQueueCreate(4, sizeof(temp_reading_t));
    s_cmd_q     = xQueueCreate(8, sizeof(mqtt_cmd_t));
    s_motor_q   = xQueueCreate(4, sizeof(motor_cmd_t));
    s_state_q   = xQueueCreate(4, sizeof(system_state_t));
    s_thermal_q = xQueueCreate(4, sizeof(thermal_cmd_t));
    configASSERT(s_temp_q && s_cmd_q && s_motor_q && s_state_q && s_thermal_q);

    ESP_LOGI(TAG, "Queues created");

    /* ── Start control_task ─────────────────────────────────────────────── */
    control_task_config_t ctrl_cfg = {
        .temp_q     = s_temp_q,
        .cmd_q      = s_cmd_q,
        .motor_q    = s_motor_q,
        .state_q    = s_state_q,
        .thermal_q  = s_thermal_q,
    };
    control_task_start(&ctrl_cfg);

    /* ── Start thermal_task ──────────────────────────────────────── */
    thermal_task_config_t therm_cfg = {
        .relay_gpio  = PIN_RELAY,
        .i2c_bus     = i2c_bus,
        .bme280_addr = BME280_ADDR,
        .temp_q      = s_temp_q,
        .thermal_q   = s_thermal_q,
    };
    thermal_task_start(&therm_cfg);

    /* ── Init DRV8833 & start motor_task ────────────────────────── */
    motor_task_config_t motor_cfg = {
        .drv_cfg = {
            .ain1_gpio    = PIN_AIN1,
            .ain2_gpio    = PIN_AIN2,
            .ain1_ledc_ch = LEDC_CHANNEL_0,
            .ain2_ledc_ch = LEDC_CHANNEL_1,
            .bin1_gpio    = PIN_BIN1,
            .bin2_gpio    = PIN_BIN2,
            .bin1_ledc_ch = LEDC_CHANNEL_2,
            .bin2_ledc_ch = LEDC_CHANNEL_3,
            .ledc_timer   = LEDC_TIMER_0,
            .pwm_freq_hz  = 20000u,
            .nsleep_gpio  = PIN_NSLEEP,
            .fault_gpio   = PIN_FAULT,
        },
        .ch      = DRV8833_CHANNEL_A,
        .motor_q = s_motor_q,
    };
    motor_task_start(&motor_cfg);

    /* ── Start comms_task ────────────────────────────────────────── */
    comms_task_config_t comms_cfg = {
        .state_q = s_state_q,
        .cmd_q   = s_cmd_q,
    };
    comms_task_start(&comms_cfg);

#if CONFIG_SMART_COOKING_CLI
    /* ── Start CLI task (M3 testing only) ──────────────────────────── */
    cli_task_config_t cli_cfg = {
        .cmd_q = s_cmd_q,
    };
    cli_task_start(&cli_cfg);
#endif

    ESP_LOGI(TAG, "Firmware started — control_task + thermal_task + motor_task + comms_task running");
}
