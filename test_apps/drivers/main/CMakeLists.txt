idf_component_register(
    SRCS
        "app_main.c"
        "test_bme280.c"
    INCLUDE_DIRS "."
    REQUIRES bme280 esp_driver_i2c log freertos
)
