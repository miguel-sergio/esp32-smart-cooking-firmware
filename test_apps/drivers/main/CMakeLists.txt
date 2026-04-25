idf_component_register(
    SRCS
        "app_main.c"
        "test_bme280.c"
        "test_drv8833.c"
    INCLUDE_DIRS "."
    REQUIRES bme280 drv8833 esp_driver_i2c esp_driver_ledc log freertos
)
