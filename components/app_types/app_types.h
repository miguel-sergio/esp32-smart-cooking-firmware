#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ── Cooking state machine ──────────────────────────────────────────────── */

typedef enum {
    COOKING_STATE_INVALID = -1,
    COOKING_STATE_IDLE,
    COOKING_STATE_PREHEAT,
    COOKING_STATE_COOKING,
    COOKING_STATE_DONE,
    COOKING_STATE_ERROR,
} cooking_state_t;

/* ── Fault codes ────────────────────────────────────────────────────────── */

typedef enum {
    FAULT_NONE,
    FAULT_OVERTEMP,         /* temperature exceeded safety cutoff    */
    FAULT_SENSOR_TIMEOUT,   /* no valid BME280 reading for >3 s      */
    FAULT_ESTOP,            /* emergency stop received via MQTT      */
    FAULT_HEATER_FAIL,      /* temp not rising within HEAT_RISE_MS   */
} fault_type_t;

/* ── Inter-task message types ───────────────────────────────────────────── */

/* thermal_task → control_task (temp_q) */
typedef struct {
    float    temperature;   /* °C                                    */
    float    humidity;      /* % RH                                  */
    bool     valid;         /* false = sensor read failed            */
    uint32_t timestamp_ms;  /* now_ms() at time of reading           */
} temp_reading_t;

/* control_task → thermal_task (thermal_q) */
typedef struct {
    bool  enabled;    /* false = relay must stay OFF regardless         */
    float setpoint;   /* °C target; thermal_task applies bang-bang      */
} thermal_cmd_t;

/* comms_task → control_task (cmd_q) */
typedef enum {
    CMD_START,
    CMD_STOP,
    CMD_ESTOP,
    CMD_RESET,
} mqtt_cmd_type_t;

typedef struct {
    mqtt_cmd_type_t type;
    uint8_t         profile_id;  /* 0 = Standard, 1 = Delicate (for CMD_START) */
} mqtt_cmd_t;

/* control_task → motor_task (motor_q) */
typedef struct {
    int8_t duty_pct;   /* 0–100 % (negative = reverse)              */
    bool   brake;      /* true = active brake (ESTOP / ERROR)       */
} motor_cmd_t;

/* control_task → comms_task (state_q) */
typedef struct {
    cooking_state_t state;
    float           temperature;    /* °C                                    */
    float           humidity;       /* % RH — from thermal_task              */
    fault_type_t    fault;
    uint8_t         active_profile; /* 0 = Standard, 1 = Delicate            */
    int8_t          motor_duty_pct; /* 0 = stopped, >0 = running             */
} system_state_t;

/* ── Shared helpers ─────────────────────────────────────────────────────── */

/** Convenience macro — number of elements in a stack-allocated array. */
#define ARRAY_SIZE(a)  (sizeof(a) / sizeof((a)[0]))

/** Human-readable name for a cooking_state_t value (for logs/telemetry). */
static inline const char *state_name(cooking_state_t s) {
    switch (s) {
    case COOKING_STATE_IDLE:    return "IDLE";
    case COOKING_STATE_PREHEAT: return "PREHEAT";
    case COOKING_STATE_COOKING: return "COOKING";
    case COOKING_STATE_DONE:    return "DONE";
    case COOKING_STATE_ERROR:   return "ERROR";
    default:                    return "UNKNOWN";
    }
}

/** Human-readable name for a fault_type_t value (for logs/telemetry). */
static inline const char *fault_name(fault_type_t f) {
    switch (f) {
    case FAULT_NONE:           return "NONE";
    case FAULT_OVERTEMP:       return "OVERTEMP";
    case FAULT_SENSOR_TIMEOUT: return "SENSOR_TIMEOUT";
    case FAULT_ESTOP:          return "ESTOP";
    case FAULT_HEATER_FAIL:    return "HEATER_FAIL";
    default:                   return "UNKNOWN";
    }
}
