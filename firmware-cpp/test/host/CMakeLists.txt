# Host-side tests for the parts of the firmware that do not touch hardware:
# the TF03 wire protocol and the geometry. Everything else is a thin layer over
# ESP-IDF and the libraries, and is checked on the board with the demos.
#
#   cmake -S test/host -B build/host && cmake --build build/host && ctest --test-dir build/host
cmake_minimum_required(VERSION 3.22)
project(ping_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(PING_DIR ${CMAKE_CURRENT_LIST_DIR}/../../components/ping)

add_executable(ping_host_tests
    ${PING_DIR}/src/geo.cpp
    ${PING_DIR}/src/tf03_protocol.cpp
    test_geo.cpp
    test_tf03_protocol.cpp)
target_include_directories(ping_host_tests PRIVATE ${PING_DIR}/include)
target_compile_options(ping_host_tests PRIVATE -Wall -Wextra -Werror)
target_link_libraries(ping_host_tests PRIVATE GTest::gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(ping_host_tests)
