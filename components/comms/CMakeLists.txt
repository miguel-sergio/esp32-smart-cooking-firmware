idf_component_register(
    SRCS
        "comms_task.c"
        "cli_task.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log nvs_flash esp_wifi esp_event esp_netif
             espressif__mqtt mbedtls esp_timer espressif__cjson
             provisioning app_types cooking_logic ota app_update
    EMBED_TXTFILES
        "../../main/certs/AmazonRootCA1.pem"
        "../../main/certs/esp32-smart-cooking.cert.pem"
        "../../main/certs/esp32-smart-cooking.private.key"
)
