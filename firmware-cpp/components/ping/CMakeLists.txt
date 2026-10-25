idf_component_register(
    SRCS "src/display.cpp"
         "src/geo.cpp"
         "src/gps.cpp"
         "src/imu.cpp"
         "src/tf03.cpp"
         "src/tf03_protocol.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_spi esp_driver_uart esp_timer freertos log
             esp32_BNO08x sparkfun__sparkfun_u-blox_gnss_v4 u8g2)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_23)

# The display driver finds its way back from u8g2's callbacks to the Display
# through u8g2's user pointer, which u8g2 only compiles in on request.
idf_component_get_property(u8g2_lib u8g2 COMPONENT_LIB)
target_compile_definitions(${u8g2_lib} PUBLIC U8X8_WITH_USER_PTR)
