# Host-side checks of the code that does not touch hardware: the geometry,
# and the TF03 protocol parser. Plain CMake and the host's own C++23
# compiler; no ESP-IDF and no package manager.
#
#     cmake -S test/host -B build-host -G Ninja -DCMAKE_CXX_COMPILER=clang++
#     cmake --build build-host
#     ctest --test-dir build-host --output-on-failure

cmake_minimum_required(VERSION 3.22)
project(ping_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(components ${CMAKE_CURRENT_LIST_DIR}/../../components)

enable_testing()

add_executable(geo_test geo_test.cpp ${components}/geo/src/geo.cpp)
target_include_directories(geo_test PRIVATE ${components}/geo/include)
target_compile_options(geo_test PRIVATE -Wall -Wextra -Wfloat-conversion)
add_test(NAME geo COMMAND geo_test)

# The TF03 parser's checks are static_asserts, so building them is the test.
add_library(tf03_protocol_checks OBJECT ${components}/tf03/src/protocol_checks.cpp)
target_include_directories(tf03_protocol_checks PRIVATE ${components}/tf03/include)
target_compile_options(tf03_protocol_checks PRIVATE -Wall -Wextra)
