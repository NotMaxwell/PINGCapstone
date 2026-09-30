# The NEO-M9N GNSS receiver: a C++ wrapper for u-blox's ubxlib.
idf_component_register(
    SRCS "src/neo_m9n.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos
    PRIV_REQUIRES ubxlib)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/ping_cxx.cmake)
ping_cxx_component(${COMPONENT_LIB})
