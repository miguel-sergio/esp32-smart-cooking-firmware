#include "cli_task.h"
#include "app_types.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "cli";

/* ── Task constants ─────────────────────────────────────────────────────── */

#define CLI_STACK_WORDS     (2048u / sizeof(StackType_t))
#define CLI_PRIORITY        2
#define CLI_CORE            0

/* ── Module state ───────────────────────────────────────────────────────── */

static cli_task_config_t s_cfg;

/* ── Task ───────────────────────────────────────────────────────────────── */

static void cli_task(void *arg) {
    (void)arg;

    /* Disable stdin buffering so each keystroke is delivered immediately. */
    setvbuf(stdin, NULL, _IONBF, 0);

    ESP_LOGI(TAG, "CLI ready — s=START-p0  S=START-p1  x=STOP  e=ESTOP  r=RESET");

    for (;;) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        mqtt_cmd_t cmd = { .type = CMD_STOP, .profile_id = 0u };
        bool valid = true;

        switch ((char)c) {
        case 's':
            cmd.type       = CMD_START;
            cmd.profile_id = 0u;
            ESP_LOGI(TAG, "CMD_START profile=0 (Standard)");
            break;
        case 'S':
            cmd.type       = CMD_START;
            cmd.profile_id = 1u;
            ESP_LOGI(TAG, "CMD_START profile=1 (Delicate)");
            break;
        case 'x':
            cmd.type = CMD_STOP;
            ESP_LOGI(TAG, "CMD_STOP");
            break;
        case 'e':
            cmd.type = CMD_ESTOP;
            ESP_LOGI(TAG, "CMD_ESTOP");
            break;
        case 'r':
            cmd.type = CMD_RESET;
            ESP_LOGI(TAG, "CMD_RESET");
            break;
        default:
            valid = false;
            break;
        }

        if (valid) {
            if (xQueueSend(s_cfg.cmd_q, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
                ESP_LOGW(TAG, "cmd_q full — command dropped");
            }
        }
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void cli_task_start(const cli_task_config_t *cfg) {
    configASSERT(cfg != NULL);
    configASSERT(cfg->cmd_q != NULL);
    s_cfg = *cfg;

    BaseType_t ret = xTaskCreatePinnedToCore(
        cli_task,
        "cli",
        CLI_STACK_WORDS,
        NULL,
        CLI_PRIORITY,
        NULL,
        CLI_CORE
    );
    configASSERT(ret == pdPASS);
}
