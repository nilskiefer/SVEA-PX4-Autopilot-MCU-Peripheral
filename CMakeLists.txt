cmake_minimum_required(VERSION 3.16)

# Limit project components explicitly so ESP-IDF does not try to treat
# vendored MAVLink source trees in ./components as ESP-IDF components.
# Keep the full ESP-IDF component set available.
set(COMPONENT_DIRS
    "$ENV{IDF_PATH}/components"
    "main"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(svea_px4_module_encoder)
