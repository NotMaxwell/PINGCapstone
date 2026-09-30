# Turning a range and an orientation into a coordinate. Plain C++ with no
# ESP-IDF dependencies, so it also builds on a host.
idf_component_register(
    SRCS "src/geo.cpp"
    INCLUDE_DIRS "include")

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/ping_cxx.cmake)
ping_cxx_component(${COMPONENT_LIB})
