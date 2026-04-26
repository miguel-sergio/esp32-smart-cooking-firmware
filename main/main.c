#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "app_types.h"
#include "control_task.h"
#include "thermal_task.h"
#include "bme280.h"

static const char *TAG = "main";

/* ── Pin configuration ──────────────────────────────────────────────────
 * Centralised pin map for the whole application.
 * ----------------------------------------------------------------------- */
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_RELAY      32
#define BME280_ADDR    BME280_I2C_ADDR_DEFAULT

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

    ESP_LOGI(TAG, "Firmware started — control_task + thermal_task running");
}
