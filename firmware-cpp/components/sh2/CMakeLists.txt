# CEVA's SH-2 reference driver for the BNO08x sensor hubs, vendored unchanged
# from https://github.com/ceva-dsp/sh2 at v1.4.0
# (b514b1e2586ddc195e553dac89fc94c637b25298). Apache-2.0; see NOTICE.txt.
#
# Vendored rather than fetched so the build never depends on a third-party
# wrapper: ping/src/bno085_hal.cpp implements sh2_Hal_t on ESP-IDF's SPI
# master with the same INT/WAKE/RST handling as the Rust driver.
# euler.c is left out; geo.cpp has the conversion the firmware uses.
idf_component_register(
    SRCS "sh2.c" "sh2_SensorValue.c" "sh2_util.c" "shtp.c"
    INCLUDE_DIRS ".")
# Vendor code: keep it unchanged, keep its warnings out of our -Werror.
target_compile_options(${COMPONENT_LIB} PRIVATE -w)
