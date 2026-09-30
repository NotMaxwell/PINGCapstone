# PING firmware: ESP-IDF on FreeRTOS, built with Espressif's LLVM/Clang.
#
# This is an ordinary ESP-IDF project, so CMake drives the whole build and
# `idf.py` is optional. Configure and build with plain CMake:
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

# The vendored third-party libraries, kept apart from our own components.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/vendor/sh2"
    "${CMAKE_CURRENT_LIST_DIR}/vendor/ubxlib")

# Only pull in the ESP-IDF components the firmware uses, rather than all of
# them; this keeps configure and build times down. Everything these depend on
# comes in automatically.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ping)
