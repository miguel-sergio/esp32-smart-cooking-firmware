#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "app_types.h"

/* ── Cooking profile ────────────────────────────────────────────────────── */

typedef struct {
    float    preheat_target;    /* °C — transition PREHEAT → COOKING   */
    float    cook_target;       /* °C — setpoint sent to thermal_task  */
    float    safety_cutoff;     /* °C — OVERTEMP fault threshold       */
    uint32_t cook_duration_ms;  /* ms  — COOKING → DONE timer          */
    int8_t   motor_duty_pct;    /* %   — motor duty during cook cycle  */
} cooking_profile_t;

/* ── Heater fail check ──────────────────────────────────────────────────── */

/**
 * Returns true when @p last_temp has been more than 1 °C below @p target
 * for at least @p heat_rise_ms consecutive milliseconds.
 * Resets the consolidation window whenever temperature recovers.
 *
 * @p below_target_since_ms  In-out: 0 = no active window; non-zero = start
 *                           of current below-target window in ms ticks.
 */
bool cooking_logic_check_heater_fail(float     last_temp,
                                     float     target,
                                     uint32_t  tick_ms,
                                     uint32_t *below_target_since_ms,
                                     uint32_t  heat_rise_ms);

/* ── Safe-state / interruptibility check ────────────────────────────────────── */

/**
 * Returns true when @p state indicates no active cooking cycle is running,
 * i.e., it is safe to perform interruptive operations such as an OTA update
 * or a deferred Wi-Fi restart (IDLE, DONE, or ERROR).
 */
bool cooking_logic_cycle_inactive(cooking_state_t state);

/* ── State transition ───────────────────────────────────────────────────── */

typedef struct {
    cooking_state_t  current_state;
    fault_type_t     current_fault;

    bool             got_cmd;
    mqtt_cmd_type_t  cmd_type;

    float            last_temp;
    uint32_t         tick_ms;
    uint32_t         last_temp_ms;
    uint32_t         cook_start_ms;
    uint32_t         done_start_ms;
    uint32_t         below_target_since_ms;

    float            preheat_target;
    float            cook_target;
    float            safety_cutoff;
    uint32_t         cook_duration_ms;

    uint32_t         sensor_timeout_ms;
    uint32_t         done_autoreturn_ms;
    uint32_t         heat_rise_ms;
} cooking_transition_input_t;

typedef struct {
    cooking_state_t  next_state;
    fault_type_t     next_fault;
    uint32_t         below_target_since_ms;
    bool             cmd_rejected;   /* true when got_cmd and cmd was not valid in state */
} cooking_transition_output_t;

/**
 * Pure state transition: applies ESTOP, per-state fault checks, and
 * command handling.  Has no side effects; all ESP-IDF / FreeRTOS calls
 * remain in control_task.c.
 */
cooking_transition_output_t cooking_logic_next_state(
    const cooking_transition_input_t *in);

/* ── Profile validation ─────────────────────────────────────────────────── */

/**
 * Returns true when all profile fields are within valid operating ranges:
 *   0 < preheat_target < cook_target < safety_cutoff
 *   cook_duration_ms > 0
 *   0 <= motor_duty_pct <= 100
 */
bool cooking_logic_profile_is_valid(const cooking_profile_t *p);
