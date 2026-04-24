idf_component_register(
    SRCS "bme280.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c log freertos
)
