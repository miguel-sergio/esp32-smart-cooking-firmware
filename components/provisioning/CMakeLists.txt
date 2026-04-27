idf_component_register(
    SRCS
        "provisioning.c"
        "provisioning_security.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log nvs_flash esp_system bt mbedtls esp_hw_support
)
