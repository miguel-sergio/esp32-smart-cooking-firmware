#include "unity.h"
#include "cooking_logic.h"

#include <stdint.h>
#include <limits.h>

/* ── test_heater_fail_triggers_after_timeout ────────────────────────────── */

static void test_heater_fail_triggers_after_timeout(void) {
    uint32_t window = 0u;

    /* First call: temp is below target, window opens at tick=1000 */
    bool r1 = cooking_logic_check_heater_fail(25.0f, 60.0f, 1000u, &window, 120000u);
    TEST_ASSERT_FALSE(r1);
    TEST_ASSERT_EQUAL_UINT32(1000u, window);

    /* Intermediate call: still below target, not yet timed out */
    bool r2 = cooking_logic_check_heater_fail(25.0f, 60.0f, 100000u, &window, 120000u);
    TEST_ASSERT_FALSE(r2);

    /* Final call: elapsed = 121000 >= heat_rise_ms = 120000 → triggers */
    bool r3 = cooking_logic_check_heater_fail(25.0f, 60.0f, 121000u, &window, 120000u);
    TEST_ASSERT_TRUE(r3);
}

/* ── test_heater_fail_no_trigger_before_timeout ─────────────────────────── */

static void test_heater_fail_no_trigger_before_timeout(void) {
    uint32_t window = 0u;

    cooking_logic_check_heater_fail(25.0f, 60.0f, 1000u, &window, 120000u);

    /* elapsed = 119999 < 120000 — must not trigger */
    bool result = cooking_logic_check_heater_fail(25.0f, 60.0f, 120999u, &window, 120000u);
    TEST_ASSERT_FALSE(result);
}

/* ── test_heater_fail_resets_on_recovery ────────────────────────────────── */

static void test_heater_fail_resets_on_recovery(void) {
    uint32_t window = 0u;

    /* Open window at tick=1000 */
    cooking_logic_check_heater_fail(25.0f, 60.0f, 1000u, &window, 120000u);
    TEST_ASSERT_EQUAL_UINT32(1000u, window);

    /* Temperature recovers: window resets */
    cooking_logic_check_heater_fail(60.0f, 60.0f, 5000u, &window, 120000u);
    TEST_ASSERT_EQUAL_UINT32(0u, window);

    /* Start a new window after recovery; elapsed from 5000 is only 1000 */
    cooking_logic_check_heater_fail(25.0f, 60.0f, 5000u, &window, 120000u);
    bool result = cooking_logic_check_heater_fail(25.0f, 60.0f, 6000u, &window, 120000u);
    TEST_ASSERT_FALSE(result);   /* new window, not timed out yet */
}

/* ── test_heater_fail_boundary_exactly_at_threshold ────────────────────── */

static void test_heater_fail_boundary_exactly_at_threshold(void) {
    /* temp == target - 1.0f: condition is strictly <, so NOT below threshold */
    uint32_t window = 0u;
    bool result = cooking_logic_check_heater_fail(
        59.0f,   /* target - 1.0f exactly */
        60.0f,
        1000u,
        &window,
        120000u
    );
    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_EQUAL_UINT32(0u, window);  /* window must NOT have opened */
}

/* ── test_heater_fail_boundary_one_tick_before ──────────────────────────── */

static void test_heater_fail_boundary_one_tick_before(void) {
    /* elapsed == heat_rise_ms - 1: must not trigger */
    uint32_t window = 1000u;  /* window already open */
    bool result = cooking_logic_check_heater_fail(
        25.0f,
        60.0f,
        1000u + 99u,  /* elapsed = 99 < heat_rise_ms = 100 */
        &window,
        100u
    );
    TEST_ASSERT_FALSE(result);
}

/* ── test_heater_fail_no_window_if_above_target ─────────────────────────── */

static void test_heater_fail_no_window_if_above_target(void) {
    uint32_t window = 0u;
    /* temp well above target → window never opens, no fault */
    bool result = cooking_logic_check_heater_fail(
        80.0f, 60.0f, 999999u, &window, 1u
    );
    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_EQUAL_UINT32(0u, window);
}

/* ── test_heater_fail_tick_overflow ─────────────────────────────────────── */

static void test_heater_fail_tick_overflow(void) {
    /* Window opened near UINT32_MAX; tick_ms has wrapped to a small value.
     * Unsigned subtraction handles wrap-around correctly:
     *   60 - (UINT32_MAX - 50) wraps to 111 >= heat_rise_ms=100 → trigger. */
    uint32_t window = UINT32_MAX - 50u;
    bool result = cooking_logic_check_heater_fail(
        25.0f, 60.0f, 60u, &window, 100u
    );
    TEST_ASSERT_TRUE(result);
}

/* ── runner ─────────────────────────────────────────────────────────────── */

void run_heater_fail_tests(void) {
    RUN_TEST(test_heater_fail_triggers_after_timeout);
    RUN_TEST(test_heater_fail_no_trigger_before_timeout);
    RUN_TEST(test_heater_fail_resets_on_recovery);
    RUN_TEST(test_heater_fail_boundary_exactly_at_threshold);
    RUN_TEST(test_heater_fail_boundary_one_tick_before);
    RUN_TEST(test_heater_fail_no_window_if_above_target);
    RUN_TEST(test_heater_fail_tick_overflow);
}
