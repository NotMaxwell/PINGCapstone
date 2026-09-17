# Builds two ways. Under idf.py (ESP_PLATFORM) this is an ESP-IDF component
# carrying the real peripheral wrappers; under a plain CMake configure it is
# a header-only interface library exposing just the concepts in bus.hpp, so
# the host test suite can compile the drivers against its own fakes.
if(ESP_PLATFORM)
    idf_component_register(
        SRCS "src/esp_i2c.cpp"
             "src/esp_uart.cpp"
             "src/esp_gpio.cpp"
        INCLUDE_DIRS "include"
        REQUIRES driver esp_timer freertos)
    target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_23)
else()
    add_library(ping_hal INTERFACE)
    target_include_directories(ping_hal INTERFACE include)
    target_compile_features(ping_hal INTERFACE cxx_std_23)
endif()
