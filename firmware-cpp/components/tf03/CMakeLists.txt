# The Benewake TF03 LiDAR on ESP-IDF's UART driver. Benewake ships no host
# driver, so this one is ours; protocol_checks.cpp verifies the parser at
# compile time.
idf_component_register(
    SRCS "src/tf03.cpp" "src/protocol_checks.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart freertos)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/ping_cxx.cmake)
ping_cxx_component(${COMPONENT_LIB})
