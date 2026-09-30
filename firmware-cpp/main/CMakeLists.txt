# The application: sensor tasks, the button, and the sighting it reports.
idf_component_register(
    SRCS "main.cpp" "sensors.cpp" "sighting.cpp"
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
