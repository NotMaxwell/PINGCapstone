# See components/ping_hal/CMakeLists.txt for why every component builds both
# as an ESP-IDF component and as a plain library.
if(ESP_PLATFORM)
    idf_component_register(
        SRCS "src/reports.cpp"
        INCLUDE_DIRS "include"
        REQUIRES ping_hal)
    target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_23)
else()
    add_library(ping_bno085 STATIC src/reports.cpp)
    target_include_directories(ping_bno085 PUBLIC include)
    target_link_libraries(ping_bno085 PUBLIC ping_hal)
    target_compile_features(ping_bno085 PUBLIC cxx_std_23)
endif()
