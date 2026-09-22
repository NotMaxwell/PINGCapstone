idf_component_register(
    SRCS "main.cpp" "imu.cpp" "lidar.cpp" "gnss.cpp" "ros_node.cpp"
    INCLUDE_DIRS "."
    REQUIRES arduino_libs esp_timer)

# Newest standard GCC 14 (ESP-IDF 5.5's toolchain) accepts. ESP-IDF's own
# default is gnu++2b; a later -std flag wins, and this only affects our code.
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++26)
