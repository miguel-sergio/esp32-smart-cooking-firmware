#include "unity.h"
#include "cooking_logic.h"

/* ── Helpers ────────────────────────────────────────────────────────────── */

static cooking_profile_t valid_profile(void) {
    return (cooking_profile_t){
        .preheat_target  = 60.0f,
        .cook_target     = 80.0f,
        .safety_cutoff   = 95.0f,
        .cook_duration_ms = 1800000u,
        .motor_duty_pct  = 60,
    };
}

/* ── test_profile_standard_valid ────────────────────────────────────────── */

static void test_profile_standard_valid(void) {
    cooking_profile_t p = valid_profile();
    TEST_ASSERT_TRUE(cooking_logic_profile_is_valid(&p));
}

/* ── test_profile_delicate_valid ────────────────────────────────────────── */

static void test_profile_delicate_valid(void) {
    cooking_profile_t p = {
        .preheat_target   = 45.0f,
        .cook_target      = 60.0f,
        .safety_cutoff    = 75.0f,
        .cook_duration_ms = 1200000u,
        .motor_duty_pct   = 40,
    };
    TEST_ASSERT_TRUE(cooking_logic_profile_is_valid(&p));
}

/* ── test_profile_preheat_exceeds_cook_invalid ──────────────────────────── */

static void test_profile_preheat_exceeds_cook_invalid(void) {
    cooking_profile_t p = valid_profile();
    p.preheat_target = 85.0f;  /* > cook_target=80 */
    TEST_ASSERT_FALSE(cooking_logic_profile_is_valid(&p));
}

/* ── test_profile_cook_exceeds_safety_invalid ───────────────────────────── */

static void test_profile_cook_exceeds_safety_invalid(void) {
    cooking_profile_t p = valid_profile();
    p.cook_target = 96.0f;  /* > safety_cutoff=95 */
    TEST_ASSERT_FALSE(cooking_logic_profile_is_valid(&p));
}

/* ── test_profile_zero_duration_invalid ─────────────────────────────────── */

static void test_profile_zero_duration_invalid(void) {
    cooking_profile_t p = valid_profile();
    p.cook_duration_ms = 0u;
    TEST_ASSERT_FALSE(cooking_logic_profile_is_valid(&p));
}

/* ── test_profile_motor_duty_out_of_range_invalid ───────────────────────── */

static void test_profile_motor_duty_out_of_range_invalid(void) {
    cooking_profile_t p = valid_profile();
    p.motor_duty_pct = 101;
    TEST_ASSERT_FALSE(cooking_logic_profile_is_valid(&p));
}

/* ── test_profile_null_invalid ──────────────────────────────────────────── */

static void test_profile_null_invalid(void) {
    TEST_ASSERT_FALSE(cooking_logic_profile_is_valid(NULL));
}

/* ── runner ─────────────────────────────────────────────────────────────── */

void run_profile_tests(void) {
    RUN_TEST(test_profile_standard_valid);
    RUN_TEST(test_profile_delicate_valid);
    RUN_TEST(test_profile_preheat_exceeds_cook_invalid);
    RUN_TEST(test_profile_cook_exceeds_safety_invalid);
    RUN_TEST(test_profile_zero_duration_invalid);
    RUN_TEST(test_profile_motor_duty_out_of_range_invalid);
    RUN_TEST(test_profile_null_invalid);
}
