#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "app_types.h"
#include "control_task.h"

static const char *TAG = "main";

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

    ESP_LOGI(TAG, "Firmware started — control_task running");
}
