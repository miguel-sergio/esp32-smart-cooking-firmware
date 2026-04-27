idf_component_register(
    SRCS
        "main.c"
        "control_task.c"
        "thermal_task.c"
        "motor_task.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log bme280 drv8833 nvs_flash esp_system
             esp_driver_i2c esp_driver_gpio esp_driver_ledc
             app_types provisioning comms
)
