# The BNO085 IMU: an ESP-IDF HAL and C++ wrapper for CEVA's SH-2 driver.
idf_component_register(
    SRCS "src/bno085.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sh2 esp_driver_i2c esp_driver_gpio freertos
    PRIV_REQUIRES esp_timer)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/ping_cxx.cmake)
ping_cxx_component(${COMPONENT_LIB})
