# PING firmware: the Arduino core for ESP32 as an ESP-IDF component, on
# FreeRTOS, built with Espressif's LLVM/Clang.
#
# This is an ordinary ESP-IDF project, so CMake drives the whole build and
# `idf.py` is optional. It needs ESP-IDF v5.5, the release the Arduino core
# 3.3 is built on. Configure and build with plain CMake:
#
#     cmake --preset esp32s3        # or: cmake -B build -G Ninja
#     cmake --build --preset esp32s3
#
# See README.md for setting up ESP-IDF and flashing.

cmake_minimum_required(VERSION 3.22)

# Build for the ESP32-S3 with Clang unless told otherwise. ESP-IDF reads both
# from the environment first and the CMake cache second, so an exported
# IDF_TARGET or IDF_TOOLCHAIN still wins.
if(NOT DEFINED ENV{IDF_TARGET})
    set(IDF_TARGET "esp32s3" CACHE STRING "ESP-IDF target chip")
endif()
if(NOT DEFINED ENV{IDF_TOOLCHAIN})
    set(IDF_TOOLCHAIN "clang" CACHE STRING "ESP-IDF toolchain: clang or gcc")
endif()

# The vendored third-party code: the Arduino core and the sensor libraries
# written for it, each directory an ESP-IDF component. Kept apart from our
# own components.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/vendor")

# Only pull in the ESP-IDF components the firmware uses, rather than all of
# them; this keeps configure and build times down. Everything these depend on
# comes in automatically.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ping)

# The Arduino core as upstream ships it pulls the mdns component from the
# ESP-IDF registry, and gets esp_netif onto its public include path through
# it; IPAddress.h, which Arduino.h includes, needs it. Without the package
# manager, link it here, which puts it on the path of everything that uses
# the core.
idf_component_get_property(arduino_lib arduino-esp32 COMPONENT_LIB)
target_link_libraries(${arduino_lib} PUBLIC idf::esp_netif)

# Keep upstream's code as it is and switch off just the checks it trips.
#
# Clang flags a few things in the vendored code that GCC accepts silently:
# variable-length arrays (a GNU extension), a GCC-only warning named in a
# pragma, and brace elision. ESP-IDF builds with -Werror, so these would
# otherwise stop the build.
if(IDF_TOOLCHAIN STREQUAL "clang")
    foreach(vendored arduino-esp32 Adafruit_BNO08x Adafruit_BusIO Adafruit_Sensor
                     SparkFun_u-blox_GNSS_v3 TFMini-Plus)
        idf_component_get_property(vendored_lib ${vendored} COMPONENT_LIB)
        target_compile_options(${vendored_lib} PRIVATE
            -Wno-vla-cxx-extension
            -Wno-unknown-warning-option
            -Wno-missing-braces)
    endforeach()
else()
    # GCC, for its part, warns about ESP-IDF's own flash headers when the
    # core's Esp.cpp compiles them as C++.
    target_compile_options(${arduino_lib} PRIVATE -Wno-missing-field-initializers)
endif()
