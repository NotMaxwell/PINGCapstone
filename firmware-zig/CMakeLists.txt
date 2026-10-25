# ESP-IDF project for the Zig port of the PING firmware.
#
# One project, several applications: the main program and one demo per
# peripheral, as the Rust crate has one [[bin]] each. Pick one with PING_APP
# (see main/CMakeLists.txt); each gets its own build directory so switching
# does not rebuild the world:
#
#   idf.py -B build/imu_demo -DPING_APP=imu_demo flash monitor
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ping_capstone_zig)
