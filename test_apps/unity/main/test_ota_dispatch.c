#include "unity.h"
#include "cooking_logic.h"

/* ── test_ota_dispatch_idle ─────────────────────────────────────────────── */

static void test_ota_dispatch_idle(void) {
    TEST_ASSERT_TRUE(cooking_logic_ota_should_dispatch(COOKING_STATE_IDLE));
}

/* ── test_ota_dispatch_done ─────────────────────────────────────────────── */

static void test_ota_dispatch_done(void) {
    TEST_ASSERT_TRUE(cooking_logic_ota_should_dispatch(COOKING_STATE_DONE));
}

/* ── test_ota_dispatch_error ────────────────────────────────────────────── */

static void test_ota_dispatch_error(void) {
    TEST_ASSERT_TRUE(cooking_logic_ota_should_dispatch(COOKING_STATE_ERROR));
}

/* ── test_ota_defer_preheat ─────────────────────────────────────────────── */

static void test_ota_defer_preheat(void) {
    TEST_ASSERT_FALSE(cooking_logic_ota_should_dispatch(COOKING_STATE_PREHEAT));
}

/* ── test_ota_defer_cooking ─────────────────────────────────────────────── */

static void test_ota_defer_cooking(void) {
    TEST_ASSERT_FALSE(cooking_logic_ota_should_dispatch(COOKING_STATE_COOKING));
}

/* ── runner ─────────────────────────────────────────────────────────────── */

void run_ota_dispatch_tests(void) {
    RUN_TEST(test_ota_dispatch_idle);
    RUN_TEST(test_ota_dispatch_done);
    RUN_TEST(test_ota_dispatch_error);
    RUN_TEST(test_ota_defer_preheat);
    RUN_TEST(test_ota_defer_cooking);
}
