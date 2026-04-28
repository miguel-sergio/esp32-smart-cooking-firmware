idf_component_register(
    SRCS
        "test_main.c"
        "test_heater_fail.c"
        "test_ota_dispatch.c"
        "test_state_machine.c"
        "test_profile.c"
    INCLUDE_DIRS "."
    REQUIRES unity cooking_logic
)
