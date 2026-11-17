# Host-side tests for the parts of the firmware that do not touch hardware:
# wire protocols (TF03, UBX, SHTP transport over a fake bus), the geometry and
# the compass maths. Everything else is a thin layer over ESP-IDF and is
# checked on the board with the demos.
#
# Built through the top-level CMakeLists.txt with -DPING_HOST_TESTS=ON; see
# CMakePresets.json ("host", "host-asan") and BUILD.md.
cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The firmware uses std::expected. clang before 19 cannot use libstdc++'s
# <expected> (it gates on a concepts level clang 18 does not report), so fall
# back to LLVM's libc++ when the default library does not provide it.
include(CheckCXXSourceCompiles)
set(expected_probe "#include <expected>\nint main() { std::expected<int, int> e = 1; return *e - 1; }")
check_cxx_source_compiles("${expected_probe}" PING_STDLIB_HAS_EXPECTED)
if(NOT PING_STDLIB_HAS_EXPECTED)
    set(CMAKE_REQUIRED_FLAGS "-stdlib=libc++")
    check_cxx_source_compiles("${expected_probe}" PING_LIBCXX_HAS_EXPECTED)
    unset(CMAKE_REQUIRED_FLAGS)
    if(NOT PING_LIBCXX_HAS_EXPECTED)
        message(FATAL_ERROR "No C++ standard library with std::expected: use clang 19+ or install libc++ "
                            "(Debian/Ubuntu: libc++-dev libc++abi-dev). See BUILD.md.")
    endif()
    string(APPEND CMAKE_CXX_FLAGS " -stdlib=libc++")
    string(APPEND CMAKE_EXE_LINKER_FLAGS " -stdlib=libc++")
endif()

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz
    URL_HASH SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(PING_DIR ${CMAKE_CURRENT_LIST_DIR}/../../components/ping)

# Only sources with no ESP-IDF dependency belong here. A source listed before
# it exists is skipped, so modules can land one at a time.
set(PING_HOST_SOURCES
    geo.cpp
    mag_cal.cpp
    tf03_protocol.cpp
    ubx.cpp
    shtp_spi.cpp)
set(sources "")
foreach(src IN LISTS PING_HOST_SOURCES)
    if(EXISTS ${PING_DIR}/src/${src})
        list(APPEND sources ${PING_DIR}/src/${src})
    endif()
endforeach()
file(GLOB tests CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/test_*.cpp)

add_executable(ping_host_tests ${sources} ${tests})
target_include_directories(ping_host_tests PRIVATE ${PING_DIR}/include)
target_compile_options(ping_host_tests PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Werror)
target_link_libraries(ping_host_tests PRIVATE GTest::gtest_main)

option(PING_SANITIZE "Build the host tests with AddressSanitizer and UBSan" OFF)
if(PING_SANITIZE)
    target_compile_options(ping_host_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(ping_host_tests PRIVATE -fsanitize=address,undefined)
endif()

include(GoogleTest)
gtest_discover_tests(ping_host_tests)
