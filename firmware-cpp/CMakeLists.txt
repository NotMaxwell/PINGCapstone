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
    # ESP-IDF's export script puts esp-clang first on PATH, and esp-clang only
    # targets Espressif chips. Pick the first clang on PATH that is not
    # ESP-IDF's, so the host tests build from an exported shell too.
    if(NOT CMAKE_CXX_COMPILER)
        if(CMAKE_HOST_WIN32)
            set(path_dirs "$ENV{PATH}")
        else()
            string(REPLACE ":" ";" path_dirs "$ENV{PATH}")
        endif()
        list(FILTER path_dirs EXCLUDE REGEX "[.]espressif")
        find_program(PING_HOST_CXX NAMES clang++ PATHS ${path_dirs} NO_DEFAULT_PATH)
        find_program(PING_HOST_CC NAMES clang PATHS ${path_dirs} NO_DEFAULT_PATH)
        if(PING_HOST_CXX AND PING_HOST_CC)
            set(CMAKE_CXX_COMPILER "${PING_HOST_CXX}" CACHE FILEPATH "" FORCE)
            set(CMAKE_C_COMPILER "${PING_HOST_CC}" CACHE FILEPATH "" FORCE)
        endif()
    endif()
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
