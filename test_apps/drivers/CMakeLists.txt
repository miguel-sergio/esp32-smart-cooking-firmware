cmake_minimum_required(VERSION 3.22)

# Pull components from the main firmware tree
set(EXTRA_COMPONENT_DIRS "../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(driver-validation)
