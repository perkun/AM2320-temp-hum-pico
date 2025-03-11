cmake_minimum_required(VERSION 3.20)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
pico_sdk_init()

add_library(AM2320-temp-hum-pico INTERFACE)

file(GLOB_RECURSE SRC_FILES "src/*.cpp")

target_sources(AM2320-temp-hum-pico INTERFACE ${SRC_FILES})
target_include_directories(AM2320-temp-hum-pico INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

