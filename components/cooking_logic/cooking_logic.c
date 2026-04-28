#include "cooking_logic.h"

#include <stddef.h>

/* ── Heater fail ────────────────────────────────────────────────────────── */

bool cooking_logic_check_heater_fail(float     last_temp,
                                     float     target,
                                     uint32_t  tick_ms,
                                     uint32_t *below_target_since_ms,
                                     uint32_t  heat_rise_ms) {
    if (last_temp < (target - 1.0f)) {
        if (*below_target_since_ms == 0u) {
            *below_target_since_ms = tick_ms;   /* start consolidation window */
        } else if ((tick_ms - *below_target_since_ms) >= heat_rise_ms) {
            return true;
        }
    } else {
        *below_target_since_ms = 0u;            /* recovered — reset window   */
    }
    return false;
}

/* ── Safe-state / interruptibility check ────────────────────────────────────── */

bool cooking_logic_cycle_inactive(cooking_state_t state) {
    return state == COOKING_STATE_IDLE  ||
           state == COOKING_STATE_DONE  ||
           state == COOKING_STATE_ERROR;
}

/* ── State transition ───────────────────────────────────────────────────── */

cooking_transition_output_t cooking_logic_next_state(
    const cooking_transition_input_t *in) {

    cooking_transition_output_t out = {
        .next_state            = in->current_state,
        .next_fault            = in->current_fault,
        .below_target_since_ms = in->below_target_since_ms,
        .cmd_rejected          = false,
    };

    /* ESTOP is unconditional — processed first, no further evaluation */
    if (in->got_cmd && in->cmd_type == CMD_ESTOP) {
        out.next_state = COOKING_STATE_ERROR;
        out.next_fault = FAULT_ESTOP;
        return out;
    }

    switch (in->current_state) {

    case COOKING_STATE_IDLE:
        if (in->got_cmd && in->cmd_type == CMD_START) {
            out.next_state = COOKING_STATE_PREHEAT;
            out.next_fault = FAULT_NONE;
        } else if (in->got_cmd) {
            out.cmd_rejected = true;
        }
        break;

    case COOKING_STATE_PREHEAT:
        /* Fault: sensor timeout */
        if ((in->tick_ms - in->last_temp_ms) > in->sensor_timeout_ms) {
            out.next_state = COOKING_STATE_ERROR;
            out.next_fault = FAULT_SENSOR_TIMEOUT;
            break;
        }
        /* Fault: overtemp */
        if (in->last_temp > in->safety_cutoff) {
            out.next_state = COOKING_STATE_ERROR;
            out.next_fault = FAULT_OVERTEMP;
            break;
        }
        /* Fault: heater not rising */
        {
            uint32_t win = in->below_target_since_ms;
            if (cooking_logic_check_heater_fail(in->last_temp, in->preheat_target,
                                                in->tick_ms, &win,
                                                in->heat_rise_ms)) {
                out.next_state            = COOKING_STATE_ERROR;
                out.next_fault            = FAULT_HEATER_FAIL;
                out.below_target_since_ms = win;
                break;
            }
            out.below_target_since_ms = win;
        }
        /* Operator abort */
        if (in->got_cmd && in->cmd_type == CMD_STOP) {
            out.next_state = COOKING_STATE_IDLE;
            break;
        } else if (in->got_cmd) {
            out.cmd_rejected = true;
        }
        /* Transition: preheat target reached */
        if (in->last_temp >= in->preheat_target) {
            out.next_state = COOKING_STATE_COOKING;
        }
        break;

    case COOKING_STATE_COOKING:
        /* Fault: sensor timeout */
        if ((in->tick_ms - in->last_temp_ms) > in->sensor_timeout_ms) {
            out.next_state = COOKING_STATE_ERROR;
            out.next_fault = FAULT_SENSOR_TIMEOUT;
            break;
        }
        /* Fault: overtemp */
        if (in->last_temp > in->safety_cutoff) {
            out.next_state = COOKING_STATE_ERROR;
            out.next_fault = FAULT_OVERTEMP;
            break;
        }
        /* Fault: heater not maintaining temperature */
        {
            uint32_t win = in->below_target_since_ms;
            if (cooking_logic_check_heater_fail(in->last_temp, in->cook_target,
                                                in->tick_ms, &win,
                                                in->heat_rise_ms)) {
                out.next_state            = COOKING_STATE_ERROR;
                out.next_fault            = FAULT_HEATER_FAIL;
                out.below_target_since_ms = win;
                break;
            }
            out.below_target_since_ms = win;
        }
        /* Operator abort */
        if (in->got_cmd && in->cmd_type == CMD_STOP) {
            out.next_state = COOKING_STATE_IDLE;
            break;
        } else if (in->got_cmd) {
            out.cmd_rejected = true;
        }
        /* Transition: cook duration elapsed */
        if ((in->tick_ms - in->cook_start_ms) >= in->cook_duration_ms) {
            out.next_state = COOKING_STATE_DONE;
        }
        break;

    case COOKING_STATE_DONE:
        if ((in->got_cmd && in->cmd_type == CMD_STOP) ||
                (in->tick_ms - in->done_start_ms) >= in->done_autoreturn_ms) {
            out.next_state = COOKING_STATE_IDLE;
        } else if (in->got_cmd) {
            out.cmd_rejected = true;
        }
        break;

    case COOKING_STATE_ERROR:
        if (in->got_cmd && in->cmd_type == CMD_RESET) {
            out.next_state = COOKING_STATE_IDLE;
            out.next_fault = FAULT_NONE;
        } else if (in->got_cmd) {
            out.cmd_rejected = true;
        }
        break;

    default:
        break;
    }

    return out;
}

/* ── Profile validation ─────────────────────────────────────────────────── */

bool cooking_logic_profile_is_valid(const cooking_profile_t *p) {
    if (p == NULL) {
        return false;
    }
    return p->preheat_target > 0.0f
        && p->preheat_target < p->cook_target
        && p->cook_target    < p->safety_cutoff
        && p->cook_duration_ms > 0u
        && p->motor_duty_pct   >= 0
        && p->motor_duty_pct   <= 100;
}
