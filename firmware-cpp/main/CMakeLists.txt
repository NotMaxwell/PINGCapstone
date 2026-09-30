# The application, or one of the single-sensor demos: PING_APP, set in the
# project's CMakeLists.txt, says which. All of them share the sensor tasks.
#
# ESP-IDF first reads this file in script mode, with no cache, just to learn
# the requirements; only the real configure pass sees PING_APP, and the
# requirements are the same either way.
if(NOT DEFINED PING_APP OR PING_APP STREQUAL "ping")
    set(app_srcs "main.cpp" "sighting.cpp")
elseif(PING_APP MATCHES "^(gps|imu|lidar)_demo$")
    set(app_srcs "demos/${PING_APP}.cpp")
else()
    message(FATAL_ERROR "PING_APP must be ping, gps_demo, imu_demo or lidar_demo, not '${PING_APP}'")
endif()

idf_component_register(
    SRCS ${app_srcs} "sensors.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        arduino-esp32
        Adafruit_BNO08x
        SparkFun_u-blox_GNSS_v3
        TFMini-Plus
        geo
        esp_timer)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ping_cxx.cmake)
ping_cxx_component(${COMPONENT_LIB})
