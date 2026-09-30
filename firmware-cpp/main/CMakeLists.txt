# The application: sensor tasks, the button, and the sighting it reports.
idf_component_register(
    SRCS "main.cpp" "sensors.cpp" "sighting.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES bno085 neo_m9n tf03 geo esp_driver_gpio esp_driver_uart esp_driver_i2c esp_timer)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ping_cxx.cmake)
ping_cxx_component(${COMPONENT_LIB})
