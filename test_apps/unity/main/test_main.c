#include <stdlib.h>
#include "unity.h"

/* Forward declarations — each test file exports a run_*_tests() function */
void run_heater_fail_tests(void);
void run_ota_dispatch_tests(void);
void run_state_machine_tests(void);
void run_profile_tests(void);

void setUp(void)    {}
void tearDown(void) {}

void app_main(void) {
    UNITY_BEGIN();

    run_heater_fail_tests();
    run_ota_dispatch_tests();
    run_state_machine_tests();
    run_profile_tests();

    exit(UNITY_END());
}
