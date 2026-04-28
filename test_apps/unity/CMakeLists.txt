cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS
    "../../components/app_types"
    "../../components/cooking_logic"
)

# Restrict the build to only components we actually use — prevents pulling in
# ESP32-only drivers that are not ported to the linux target.
set(COMPONENTS main unity cooking_logic app_types)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(unity_tests)
