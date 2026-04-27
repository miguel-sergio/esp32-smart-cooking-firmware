#include "unity.h"
#include "cooking_logic.h"

/* ── Helpers ────────────────────────────────────────────────────────────── */

/* Returns a fully initialised input that keeps the current state stable
 * (temp at target, no cmd, timers not elapsed). Tests override only the
 * fields relevant to the scenario under test. */
static cooking_transition_input_t stable_input(cooking_state_t state) {
    return (cooking_transition_input_t){
        .current_state         = state,
        .current_fault         = FAULT_NONE,
        .got_cmd               = false,
        .cmd_type              = CMD_STOP,       /* irrelevant — got_cmd=false */
        .last_temp             = 25.0f,
        .tick_ms               = 1000u,
        .last_temp_ms          = 1000u,          /* same tick → no timeout     */
        .cook_start_ms         = 0u,
        .done_start_ms         = 0u,
        .below_target_since_ms = 0u,
        .preheat_target        = 60.0f,
        .cook_target           = 80.0f,
        .safety_cutoff         = 95.0f,
        .cook_duration_ms      = 1800000u,       /* 30 min                     */
        .sensor_timeout_ms     = 3000u,
        .done_autoreturn_ms    = 60000u,
        .heat_rise_ms          = 120000u,
    };
}

/* ── IDLE ───────────────────────────────────────────────────────────────── */

static void test_sm_idle_start_goes_preheat(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_IDLE);
    in.got_cmd  = true;
    in.cmd_type = CMD_START;

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_PREHEAT, out.next_state);
    TEST_ASSERT_EQUAL_INT(FAULT_NONE,            out.next_fault);
    TEST_ASSERT_FALSE(out.cmd_rejected);
}

static void test_sm_idle_stop_rejected(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_IDLE);
    in.got_cmd  = true;
    in.cmd_type = CMD_STOP;

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_IDLE, out.next_state);
    TEST_ASSERT_TRUE(out.cmd_rejected);
}

static void test_sm_idle_reset_rejected(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_IDLE);
    in.got_cmd  = true;
    in.cmd_type = CMD_RESET;

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_IDLE, out.next_state);
    TEST_ASSERT_TRUE(out.cmd_rejected);
}

/* ── PREHEAT ────────────────────────────────────────────────────────────── */

static void test_sm_preheat_temp_reached_goes_cooking(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_PREHEAT);
    in.last_temp = 60.0f;   /* == preheat_target */

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_COOKING, out.next_state);
}

static void test_sm_preheat_stop_goes_idle(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_PREHEAT);
    in.got_cmd  = true;
    in.cmd_type = CMD_STOP;

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_IDLE, out.next_state);
    TEST_ASSERT_FALSE(out.cmd_rejected);
}

static void test_sm_preheat_sensor_timeout_goes_error(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_PREHEAT);
    in.tick_ms      = 5000u;
    in.last_temp_ms = 1000u;  /* elapsed = 4000 > sensor_timeout_ms = 3000 */

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_ERROR,   out.next_state);
    TEST_ASSERT_EQUAL_INT(FAULT_SENSOR_TIMEOUT,  out.next_fault);
}

static void test_sm_preheat_overtemp_goes_error(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_PREHEAT);
    in.last_temp = 96.0f;   /* > safety_cutoff = 95 */

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_ERROR, out.next_state);
    TEST_ASSERT_EQUAL_INT(FAULT_OVERTEMP,      out.next_fault);
}

static void test_sm_preheat_start_rejected(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_PREHEAT);
    in.got_cmd  = true;
    in.cmd_type = CMD_START;

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_PREHEAT, out.next_state);
    TEST_ASSERT_TRUE(out.cmd_rejected);
}

/* ── COOKING ────────────────────────────────────────────────────────────── */

static void test_sm_cooking_duration_elapsed_goes_done(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_COOKING);
    in.last_temp       = 80.0f;       /* at cook_target — no heater fail    */
    in.tick_ms         = 1800001u;
    in.last_temp_ms    = 1800001u;    /* same tick — no sensor timeout      */
    in.cook_start_ms   = 0u;          /* elapsed = 1800001 >= 1800000 ms    */

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_DONE, out.next_state);
}

static void test_sm_cooking_stop_goes_idle(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_COOKING);
    in.last_temp = 80.0f;
    in.got_cmd   = true;
    in.cmd_type  = CMD_STOP;

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_IDLE, out.next_state);
}

/* ── DONE ───────────────────────────────────────────────────────────────── */

static void test_sm_done_stop_goes_idle(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_DONE);
    in.got_cmd  = true;
    in.cmd_type = CMD_STOP;

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_IDLE, out.next_state);
}

static void test_sm_done_autoreturn_goes_idle(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_DONE);
    in.tick_ms       = 61001u;
    in.done_start_ms = 0u;   /* elapsed = 61001 >= done_autoreturn_ms = 60000 */

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_IDLE, out.next_state);
}

/* ── ERROR ──────────────────────────────────────────────────────────────── */

static void test_sm_error_reset_goes_idle(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_ERROR);
    in.current_fault = FAULT_OVERTEMP;
    in.got_cmd       = true;
    in.cmd_type      = CMD_RESET;

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_IDLE, out.next_state);
    TEST_ASSERT_EQUAL_INT(FAULT_NONE,         out.next_fault);
}

/* ── ESTOP (any state) ──────────────────────────────────────────────────── */

static void test_sm_estop_from_idle_goes_error(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_IDLE);
    in.got_cmd  = true;
    in.cmd_type = CMD_ESTOP;

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_ERROR, out.next_state);
    TEST_ASSERT_EQUAL_INT(FAULT_ESTOP,         out.next_fault);
}

static void test_sm_estop_from_cooking_goes_error(void) {
    cooking_transition_input_t in = stable_input(COOKING_STATE_COOKING);
    in.last_temp = 80.0f;
    in.got_cmd   = true;
    in.cmd_type  = CMD_ESTOP;

    cooking_transition_output_t out = cooking_logic_next_state(&in);
    TEST_ASSERT_EQUAL_INT(COOKING_STATE_ERROR, out.next_state);
    TEST_ASSERT_EQUAL_INT(FAULT_ESTOP,         out.next_fault);
}

/* ── runner ─────────────────────────────────────────────────────────────── */

void run_state_machine_tests(void) {
    RUN_TEST(test_sm_idle_start_goes_preheat);
    RUN_TEST(test_sm_idle_stop_rejected);
    RUN_TEST(test_sm_idle_reset_rejected);
    RUN_TEST(test_sm_preheat_temp_reached_goes_cooking);
    RUN_TEST(test_sm_preheat_stop_goes_idle);
    RUN_TEST(test_sm_preheat_sensor_timeout_goes_error);
    RUN_TEST(test_sm_preheat_overtemp_goes_error);
    RUN_TEST(test_sm_preheat_start_rejected);
    RUN_TEST(test_sm_cooking_duration_elapsed_goes_done);
    RUN_TEST(test_sm_cooking_stop_goes_idle);
    RUN_TEST(test_sm_done_stop_goes_idle);
    RUN_TEST(test_sm_done_autoreturn_goes_idle);
    RUN_TEST(test_sm_error_reset_goes_idle);
    RUN_TEST(test_sm_estop_from_idle_goes_error);
    RUN_TEST(test_sm_estop_from_cooking_goes_error);
}
