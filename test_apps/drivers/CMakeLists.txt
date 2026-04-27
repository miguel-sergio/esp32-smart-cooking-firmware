cmake_minimum_required(VERSION 3.22)

# Pull only the driver components needed for this test app.
# Do NOT point at ../../components as a whole — that would pull in comms,
# ota, and provisioning, which have managed-component dependencies
# (espressif__mqtt, esp_https_ota PSA crypto) that are not installed in
# the test app tree.
set(EXTRA_COMPONENT_DIRS
    "../../components/bme280"
    "../../components/drv8833"
    "../../components/app_types"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(driver-validation)
