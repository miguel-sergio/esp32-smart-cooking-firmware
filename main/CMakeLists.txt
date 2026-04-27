idf_component_register(
    SRCS
        "main.c"
        "control_task.c"
        "thermal_task.c"
        "motor_task.c"
        "comms_task.c"
        "provisioning.c"
        "provisioning_security.c"
        "cli_task.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log bme280 drv8833 nvs_flash esp_system
             esp_driver_i2c esp_driver_gpio esp_driver_ledc
             esp_wifi esp_event esp_netif espressif__mqtt
             bt mbedtls esp_hw_support
)
