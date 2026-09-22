# ESP-IDF project for the P.I.N.G. sensor node: BNO085 IMU, Benewake TF03
# LIDAR and u-blox NEO-M9N GPS, published to ROS 2 over micro-ROS.
#
# Build with ESP-IDF v5.5:   idf.py set-target esp32s3 && idf.py build
# Dependencies are fetched by the ESP-IDF component manager (see
# main/idf_component.yml) and the git submodules under external/.
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# arduino-esp32 unconditionally depends on espressif/esp-modbus, whose TCP port
# trips GCC 14's -Wstringop-truncation, which ESP-IDF builds as an error. We
# don't use Modbus and can't drop a transitive dependency, so keep that one
# diagnostic a warning. Everything else stays -Werror.
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=stringop-truncation" APPEND)
project(ping_sensor_node)
