idf_component_register(
    SRCS "drv8833.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_ledc esp_driver_gpio log freertos
)
