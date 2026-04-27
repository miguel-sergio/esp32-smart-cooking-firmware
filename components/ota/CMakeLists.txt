idf_component_register(
    SRCS "ota_task.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log esp_https_ota app_update esp_http_client mbedtls
)
