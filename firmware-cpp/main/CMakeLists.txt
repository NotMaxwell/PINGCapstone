# Which application to build: the main program, or one of the demos that
# brings up a single peripheral and shows what it reads on the display. Each
# file in apps/ is a complete program with its own app_main, as each Rust
# [[bin]] is.
#
#   idf.py -B build/gps_demo -DPING_APP=gps_demo flash monitor
set(PING_APP "main" CACHE STRING "Application to build: main, gps_demo, imu_demo, lidar_demo or lcd_demo")
set_property(CACHE PING_APP PROPERTY STRINGS main gps_demo imu_demo lidar_demo lcd_demo)

if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/apps/${PING_APP}.cpp")
    message(FATAL_ERROR "PING_APP=${PING_APP}: there is no main/apps/${PING_APP}.cpp")
endif()

idf_component_register(
    SRCS "apps/${PING_APP}.cpp"
    REQUIRES ping esp_driver_gpio esp_timer)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_23)
