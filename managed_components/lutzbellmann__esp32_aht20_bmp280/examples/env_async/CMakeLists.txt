cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Trim the build to main + its dependencies, instead of every component in the SDK.
idf_build_set_property(MINIMAL_BUILD ON)
project(esp32_aht20_bmp280_async_example)
