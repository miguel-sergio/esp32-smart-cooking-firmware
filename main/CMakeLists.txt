idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES bme280 drv8833 esp_driver_i2c freertos log
)
