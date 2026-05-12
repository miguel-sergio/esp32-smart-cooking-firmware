# Verify that AWS IoT certificate files exist before embedding them.
# These files are device-specific and are NOT committed to source control
# (.gitignored). For CI/CD builds they are injected from GitHub Secrets;
# for local builds they must be placed manually under main/certs/.
# A clear FATAL_ERROR here is better than an obscure linker failure later.
foreach(cert_file
        "${PROJECT_DIR}/main/certs/AmazonRootCA1.pem"
        "${PROJECT_DIR}/main/certs/esp32-smart-cooking.cert.pem"
        "${PROJECT_DIR}/main/certs/esp32-smart-cooking.private.key")
    if(NOT EXISTS "${cert_file}")
        message(FATAL_ERROR
            "AWS IoT certificate file not found:\n  ${cert_file}\n"
            "For local builds, copy your AWS IoT Core device certificates to main/certs/.\n"
            "For CI/CD builds, ensure the GitHub Actions workflow injects them from Secrets.")
    endif()
endforeach()

idf_component_register(
    SRCS
        "comms_task.c"
        "cli_task.c"
    INCLUDE_DIRS "."
    REQUIRES freertos log nvs_flash esp_wifi esp_event esp_netif
             espressif__mqtt mbedtls esp_timer espressif__cjson
             provisioning app_types cooking_logic ota app_update
    EMBED_TXTFILES
        "${PROJECT_DIR}/main/certs/AmazonRootCA1.pem"
        "${PROJECT_DIR}/main/certs/esp32-smart-cooking.cert.pem"
        "${PROJECT_DIR}/main/certs/esp32-smart-cooking.private.key"
)
