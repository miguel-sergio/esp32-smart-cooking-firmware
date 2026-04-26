idf_component_register(
    SRCS
        "main.c"
        "control_task.c"
        "thermal_task.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log bme280 esp_driver_i2c esp_driver_gpio
)
