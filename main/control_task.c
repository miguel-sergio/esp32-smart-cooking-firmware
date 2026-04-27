#include "control_task.h"
#include "app_types.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "nvs.h"

static const char *TAG = "ctrl";

/* ── Generic helpers ───────────────────────────────────────────────────── */
#define ARRAY_SIZE(a)  (sizeof(a) / sizeof((a)[0]))

/* ── Task constants ─────────────────────────────────────────────────────── */

#define CTRL_STACK_WORDS    (4096u / sizeof(StackType_t))
#define CTRL_PRIORITY       5
#define CTRL_CORE           1
#define CTRL_LOOP_MS        100u        /* 10 Hz control loop              */

#define SENSOR_TIMEOUT_MS   3000u       /* fault if no reading for 3 s     */
#define DONE_AUTORETURN_MS  60000u      /* SRD: auto-return IDLE 60 s after DONE */

/* HEAT_RISE_MS: fault if temp stays below target for this long.
 * In fault-inject builds reduce to 15 s so the test completes quickly. */
#if CONFIG_SMART_COOKING_FAULT_INJECT
#define HEAT_RISE_MS        15000u
#else
#define HEAT_RISE_MS        120000u
#endif

/* ── NVS namespace and keys ────────────────────────────────────────────── */
#define NVS_NAMESPACE       "cooking"
#define NVS_KEY_ACTIVE_PROF "active_profile"
#define NVS_KEY_PROFILE_FMT "profile_%u"

/* ── Cooking profiles ───────────────────────────────────────────────────── */
/* Compile-time defaults.  On first boot these are written as blobs to NVS  *
 * (namespace "cooking", keys "profile_0" / "profile_1") so a future        *
 * milestone can update them over MQTT without a firmware rebuild.           */

typedef struct {
    float    preheat_target;    /* °C — transition PREHEAT → COOKING   */
    float    cook_target;       /* °C — setpoint sent to thermal_task  */
    float    safety_cutoff;     /* °C — OVERTEMP fault threshold       */
    uint32_t cook_duration_ms;  /* ms  — COOKING → DONE timer          */
    int8_t   motor_duty_pct;    /* %   — motor duty during cook cycle  */
} cooking_profile_t;

static const cooking_profile_t s_profiles[] = {
    [0] = {                         /* Profile 0 — Standard              */
#if CONFIG_SMART_COOKING_FAULT_INJECT
        .preheat_target   = 26.0f,
        .cook_target      = 28.0f,
        /* In fault-inject builds lower safety_cutoff to 24 °C so OVERTEMP
         * triggers at ambient without real heating hardware. */
        .safety_cutoff    = 30.0f,
#else
        .preheat_target   = 60.0f,
        .cook_target      = 80.0f,
        .safety_cutoff    = 95.0f,
#endif
        .cook_duration_ms = 30u * 60u * 1000u,
        .motor_duty_pct   = 60,
    },
    [1] = {                         /* Profile 1 — Delicate              */
        .preheat_target   = 45.0f,
        .cook_target      = 60.0f,
        .safety_cutoff    = 75.0f,
        .cook_duration_ms = 20u * 60u * 1000u,
        .motor_duty_pct   = 40,
    },
};

/* ── Module state ───────────────────────────────────────────────────────── */

static control_task_config_t s_cfg;

/* ── Helpers ────────────────────────────────────────────────────────────── */

static inline uint32_t now_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static const char *state_name(cooking_state_t s) {
    switch (s) {
    case COOKING_STATE_IDLE:    return "IDLE";
    case COOKING_STATE_PREHEAT: return "PREHEAT";
    case COOKING_STATE_COOKING: return "COOKING";
    case COOKING_STATE_DONE:    return "DONE";
    case COOKING_STATE_ERROR:   return "ERROR";
    default:                    return "UNKNOWN";
    }
}

static const char *fault_name(fault_type_t f) {
    switch (f) {
    case FAULT_NONE:           return "NONE";
    case FAULT_OVERTEMP:       return "OVERTEMP";
    case FAULT_SENSOR_TIMEOUT: return "SENSOR_TIMEOUT";
    case FAULT_ESTOP:          return "ESTOP";
    case FAULT_HEATER_FAIL:    return "HEATER_FAIL";
    default:                   return "UNKNOWN";
    }
}

static void send_thermal_cmd(bool enabled, float setpoint) {
    if (s_cfg.thermal_q == NULL) {
        return;
    }

    thermal_cmd_t cmd = { .enabled = enabled, .setpoint = setpoint };
    if (xQueueSend(s_cfg.thermal_q, &cmd, 0) != pdTRUE) {
        /*
         * Keep the latest thermal command instead of dropping it. This is
         * especially important for OFF transitions, which may only be sent
         * once on state entry.
         */
        ESP_LOGW(TAG, "thermal_q full — replacing stale thermal command");
        xQueueReset(s_cfg.thermal_q);
        if (xQueueSend(s_cfg.thermal_q, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
            ESP_LOGE(TAG, "failed to enqueue thermal command after queue reset");
        }
    }
}

static void send_motor_cmd(int8_t duty_pct, bool brake) {
    if (s_cfg.motor_q == NULL) {
        return;
    }

    motor_cmd_t cmd = { .duty_pct = duty_pct, .brake = brake };
    const bool critical_stop_cmd = brake || (duty_pct == 0);
    const TickType_t send_ticks = critical_stop_cmd ? pdMS_TO_TICKS(20) : 0;

    if (xQueueSend(s_cfg.motor_q, &cmd, send_ticks) != pdTRUE) {
        if (critical_stop_cmd) {
            ESP_LOGW(TAG, "motor_q full — stop/brake command could not be queued");
        } else {
            ESP_LOGW(TAG, "motor_q full — motor command dropped");
        }
    }
}

/**
 * Consolidation helper: returns true when @p last_temp has been more than
 * 1 °C below @p target for HEAT_RISE_MS consecutive milliseconds.
 * Resets the window whenever temperature recovers above the threshold.
 */
static bool check_heater_fail(float last_temp, float target, uint32_t tick,
                               uint32_t *below_target_since_ms) {
    if (last_temp < (target - 1.0f)) {
        if (*below_target_since_ms == 0u) {
            *below_target_since_ms = tick;   /* start consolidation window */
        } else if ((tick - *below_target_since_ms) >= HEAT_RISE_MS) {
            return true;
        }
    } else {
        *below_target_since_ms = 0u;         /* recovered — reset window */
    }
    return false;
}

static void publish_state(cooking_state_t state, float temp, fault_type_t fault) {
    if (s_cfg.state_q == NULL) {
        return;
    }
    system_state_t s = { .state = state, .temperature = temp, .fault = fault };
    if (xQueueSend(s_cfg.state_q, &s, 0) != pdTRUE) {
        /* comms_task drains this queue; a full queue means it is behind — acceptable */
    }
}

/* ── State machine ──────────────────────────────────────────────────────── */

static void control_task(void *arg) {
    (void)arg;

    cooking_state_t          state        = COOKING_STATE_IDLE;
    cooking_state_t          prev_state   = (cooking_state_t)-1;
    fault_type_t             active_fault = FAULT_NONE;
    const cooking_profile_t *profile      = &s_profiles[0];

    uint32_t cook_start_ms        = 0;
    uint32_t done_start_ms        = 0;
    uint32_t last_temp_ms         = 0;
    uint32_t below_target_since_ms = 0;   /* start of current below-target window, 0 = at target */
    float    last_temp            = 0.0f;

    TickType_t last_wake  = xTaskGetTickCount();
#if CONFIG_SMART_COOKING_STABILITY_TEST
    TickType_t last_diag  = 0u; /* anchor for 30 s diagnostic log */
#endif

    /* ── NVS: open namespace, seed profile blobs, load active profile ───── */
    nvs_handle_t nvs          = 0;
    bool         nvs_ok       = false;
    uint8_t      persisted_pid = 0u;   /* last value committed to NVS */
    {
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "NVS open failed (%s) — profile persistence disabled",
                     esp_err_to_name(err));
        } else {
            nvs_ok     = true;
            bool dirty = false;

            /* Seed compile-time profile blobs on first boot.
             * Also overwrite if the stored blob length differs from the
             * current struct size — this handles schema changes across
             * firmware updates (fields added, removed, or reordered). */
            for (uint8_t i = 0u; i < (uint8_t)ARRAY_SIZE(s_profiles); i++) {
                char              key[12];
                snprintf(key, sizeof(key), NVS_KEY_PROFILE_FMT, (unsigned)i);
                size_t            len      = sizeof(cooking_profile_t);
                cooking_profile_t tmp;
                esp_err_t         blob_err = nvs_get_blob(nvs, key, &tmp, &len);

                bool needs_write = false;
                if (blob_err == ESP_ERR_NVS_NOT_FOUND) {
                    ESP_LOGI(TAG, "NVS: %s not found — seeding default", key);
                    needs_write = true;
                } else if (blob_err == ESP_ERR_NVS_INVALID_LENGTH ||
                           (blob_err == ESP_OK && len != sizeof(cooking_profile_t))) {
                    /* Stored blob is a different size: schema changed across
                     * firmware update.  Reset to compile-time default so NVS
                     * stays consistent with the current struct layout. */
                    ESP_LOGW(TAG,
                             "NVS: %s size mismatch (stored=%zu expected=%zu)"
                             " — resetting to default",
                             key, len, sizeof(cooking_profile_t));
                    needs_write = true;
                } else if (blob_err != ESP_OK) {
                    ESP_LOGW(TAG, "NVS: %s read error (%s) — skipping",
                             key, esp_err_to_name(blob_err));
                }

                if (needs_write) {
                    esp_err_t set_err = nvs_set_blob(nvs, key, &s_profiles[i],
                                                     sizeof(cooking_profile_t));
                    if (set_err != ESP_OK) {
                        ESP_LOGW(TAG, "NVS: failed to write %s (%s) — persistence disabled",
                                 key, esp_err_to_name(set_err));
                        nvs_ok = false;
                        break;
                    }
                    dirty = true;
                }
            }

            /* Read active_profile; write default 0 if key is absent. */
            uint8_t pid = 0u;
            err = nvs_get_u8(nvs, NVS_KEY_ACTIVE_PROF, &pid);
            if (err == ESP_ERR_NVS_NOT_FOUND) {
                if (nvs_ok) {
                    esp_err_t set_err = nvs_set_u8(nvs, NVS_KEY_ACTIVE_PROF, 0u);
                    if (set_err != ESP_OK) {
                        ESP_LOGW(TAG, "NVS: failed to write active_profile default (%s) — persistence disabled",
                                 esp_err_to_name(set_err));
                        nvs_ok = false;
                    } else {
                        dirty = true;
                        ESP_LOGI(TAG, "NVS: active_profile not found — default 0 written");
                    }
                }
            } else if (err == ESP_OK) {
                if (pid >= (uint8_t)ARRAY_SIZE(s_profiles)) { pid = 0u; }
                persisted_pid = pid;
                ESP_LOGI(TAG, "NVS: active_profile = %u", (unsigned)pid);
            } else {
                ESP_LOGW(TAG, "NVS: active_profile read error (%s) — using 0",
                         esp_err_to_name(err));
                pid = 0u;
            }

            if (nvs_ok && dirty) {
                esp_err_t commit_err = nvs_commit(nvs);
                if (commit_err != ESP_OK) {
                    ESP_LOGW(TAG, "NVS: commit failed (%s) — persistence may be incomplete",
                             esp_err_to_name(commit_err));
                    nvs_ok = false;
                }
            }

            profile = &s_profiles[pid];
        }
    }

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    for (;;) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        uint32_t tick = now_ms();

        /* ── 1. Drain temp queue — keep the most recent valid reading ───── */
        temp_reading_t reading;
        while (xQueueReceive(s_cfg.temp_q, &reading, 0) == pdTRUE) {
            if (reading.valid) {
                last_temp    = reading.temperature;
                last_temp_ms = tick;
            }
        }

        /* ── 2. Read one pending command (non-blocking) ─────────────────── */
        mqtt_cmd_t cmd     = {0};
        bool       got_cmd = (xQueueReceive(s_cfg.cmd_q, &cmd, 0) == pdTRUE);

        /* ESTOP is immediate and unconditional — always latch FAULT_ESTOP so
         * telemetry reflects the most recent safety event, even if already
         * in ERROR due to a different fault. */
        if (got_cmd && cmd.type == CMD_ESTOP) {
            active_fault = FAULT_ESTOP;
            state        = COOKING_STATE_ERROR;
        }

        /* ── 3. State entry actions — run once per transition ───────────── */
        if (state != prev_state) {
            ESP_LOGI(TAG, "State \xe2\x86\x92 %s", state_name(state));

            switch (state) {
            case COOKING_STATE_IDLE:
                send_thermal_cmd(false, 0.0f);
                send_motor_cmd(0, false);
                break;

            case COOKING_STATE_PREHEAT:
                last_temp_ms          = tick;  /* reset timeout grace period */
                below_target_since_ms = 0u;
                send_thermal_cmd(true, profile->preheat_target);
                send_motor_cmd(0, false);
                break;

            case COOKING_STATE_COOKING:
                cook_start_ms         = tick;
                below_target_since_ms = 0u;
                send_thermal_cmd(true, profile->cook_target);
                send_motor_cmd(profile->motor_duty_pct, false);
                break;

            case COOKING_STATE_DONE:
                done_start_ms = tick;
                send_thermal_cmd(false, 0.0f);
                send_motor_cmd(0, false);
                break;

            case COOKING_STATE_ERROR:
                send_thermal_cmd(false, 0.0f);
                send_motor_cmd(0, true);       /* brake = true */
                ESP_LOGE(TAG, "Fault: %s", fault_name(active_fault));
                break;
            }

            prev_state = state;
        }

        /* ── 4. Per-state logic and transitions ─────────────────────────── */
        switch (state) {
        case COOKING_STATE_IDLE:
            if (got_cmd && cmd.type == CMD_START) {
                uint8_t pid = (cmd.profile_id < (uint8_t)ARRAY_SIZE(s_profiles))
                              ? cmd.profile_id : 0u;
                /* Persist selected profile only when it actually changes to
                 * avoid unnecessary NVS writes and flash wear. */
                if (nvs_ok && pid != persisted_pid) {
                    esp_err_t err = nvs_set_u8(nvs, NVS_KEY_ACTIVE_PROF, pid);
                    if (err == ESP_OK) { err = nvs_commit(nvs); }
                    if (err == ESP_OK) {
                        persisted_pid = pid;
                    } else {
                        ESP_LOGW(TAG, "NVS: failed to persist active_profile (%s)",
                                 esp_err_to_name(err));
                    }
                }
                profile      = &s_profiles[pid];
                active_fault = FAULT_NONE;
                state        = COOKING_STATE_PREHEAT;
            }
            break;

        case COOKING_STATE_PREHEAT:
            /* Fault: sensor timeout */
            if ((tick - last_temp_ms) > SENSOR_TIMEOUT_MS) {
                active_fault = FAULT_SENSOR_TIMEOUT;
                state        = COOKING_STATE_ERROR;
                break;
            }
            /* Fault: overtemp */
            if (last_temp > profile->safety_cutoff) {
                active_fault = FAULT_OVERTEMP;
                state        = COOKING_STATE_ERROR;
                break;
            }
            /* Fault: temperature below preheat target for 2 consecutive minutes.
             * The consolidation window resets whenever temperature recovers.
             * This detects heater failure at any point during the preheat cycle. */
            if (check_heater_fail(last_temp, profile->preheat_target, tick,
                                  &below_target_since_ms)) {
                active_fault = FAULT_HEATER_FAIL;
                state        = COOKING_STATE_ERROR;
                break;
            }
            /* Operator abort */
            if (got_cmd && cmd.type == CMD_STOP) {
                state = COOKING_STATE_IDLE;
                break;
            }
            /* Transition: preheat target reached */
            if (last_temp >= profile->preheat_target) {
                state = COOKING_STATE_COOKING;
            }
            break;

        case COOKING_STATE_COOKING:
            /* Fault: sensor timeout */
            if ((tick - last_temp_ms) > SENSOR_TIMEOUT_MS) {
                active_fault = FAULT_SENSOR_TIMEOUT;
                state        = COOKING_STATE_ERROR;
                break;
            }
            /* Fault: overtemp */
            if (last_temp > profile->safety_cutoff) {
                active_fault = FAULT_OVERTEMP;
                state        = COOKING_STATE_ERROR;
                break;
            }
            /* Fault: temperature below cook target for 2 consecutive minutes.
             * The consolidation window resets whenever temperature recovers.
             * This detects heater failure at any point during the cook cycle. */
            if (check_heater_fail(last_temp, profile->cook_target, tick,
                                  &below_target_since_ms)) {
                active_fault = FAULT_HEATER_FAIL;
                state        = COOKING_STATE_ERROR;
                break;
            }
            /* Operator abort */
            if (got_cmd && cmd.type == CMD_STOP) {
                state = COOKING_STATE_IDLE;
                break;
            }
            /* Transition: cook duration elapsed */
            if ((tick - cook_start_ms) >= profile->cook_duration_ms) {
                state = COOKING_STATE_DONE;
            }
            break;

        case COOKING_STATE_DONE:
            /* SRD: return to IDLE on explicit STOP or after 60 s */
            if ((got_cmd && cmd.type == CMD_STOP) ||
                    (tick - done_start_ms) >= DONE_AUTORETURN_MS) {
                state = COOKING_STATE_IDLE;
            }
            break;

        case COOKING_STATE_ERROR:
            if (got_cmd && cmd.type == CMD_RESET) {
                active_fault = FAULT_NONE;
                state        = COOKING_STATE_IDLE;
            }
            break;
        }

        /* ── 5. Publish current state to comms_task ─────────────────────── */
        publish_state(state, last_temp, active_fault);

#if CONFIG_SMART_COOKING_STABILITY_TEST
        /* ── 6. Periodic diagnostics (every 30 s) ──────────────────────── */
        if ((xTaskGetTickCount() - last_diag) >= pdMS_TO_TICKS(30000u)) {
            ESP_LOGI(TAG, "DIAG heap_free=%" PRIu32 " heap_min=%" PRIu32 " stack_hwm=%u words",
                     (uint32_t)esp_get_free_heap_size(),
                     (uint32_t)esp_get_minimum_free_heap_size(),
                     uxTaskGetStackHighWaterMark(NULL));
            last_diag = xTaskGetTickCount();
        }
#endif

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_LOOP_MS));
    }
}

/* ── Public API ─────────────────────────────────────────────────────────── */

void control_task_start(const control_task_config_t *cfg) {
    configASSERT(cfg != NULL);
    configASSERT(cfg->temp_q != NULL);
    configASSERT(cfg->cmd_q != NULL);
    s_cfg = *cfg;

    BaseType_t ret = xTaskCreatePinnedToCore(
        control_task,
        "ctrl",
        CTRL_STACK_WORDS,
        NULL,
        CTRL_PRIORITY,
        NULL,
        CTRL_CORE
    );
    configASSERT(ret == pdPASS);
}
