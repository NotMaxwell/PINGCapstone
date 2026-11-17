# The C++ PING firmware: one ESP-IDF project, several applications.
#
# Two ways in:
#
#   * Firmware (ESP-IDF, FreeRTOS). Pick an application with PING_APP (see
#     main/CMakeLists.txt); each gets its own build directory:
#         idf.py -B build/imu_demo -DPING_APP=imu_demo flash monitor
#   * Host tests (plain CMake, the system clang, no ESP-IDF):
#         cmake --preset host && cmake --build --preset host && ctest --preset host
#
# BUILD.md has every command, including the clang toolchain and debugging.
cmake_minimum_required(VERSION 3.22)

option(PING_HOST_TESTS "Build the host unit tests instead of the firmware" OFF)

if(PING_HOST_TESTS)
    project(ping_host_tests LANGUAGES C CXX)
    enable_testing()
    add_subdirectory(test/host)
    return()
endif()

if(NOT DEFINED ENV{IDF_PATH})
    message(FATAL_ERROR "IDF_PATH is not set: source ESP-IDF's export.sh first, "
                        "or configure with --preset host for the host tests. See BUILD.md.")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ping_capstone)
