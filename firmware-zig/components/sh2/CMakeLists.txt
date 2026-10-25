# CEVA's SH-2 reference driver for the BNO08x sensor hubs, vendored unchanged
# from https://github.com/ceva-dsp/sh2 at v1.4.0
# (b514b1e2586ddc195e553dac89fc94c637b25298). Apache-2.0; see NOTICE.txt.
#
# The driver is platform independent: it talks to the hub through the
# sh2_Hal_t callbacks, which src/imu.zig implements on ESP-IDF's SPI master.
# euler.c is left out; geo.zig has the conversion the firmware uses.
idf_component_register(
    SRCS "sh2.c" "sh2_SensorValue.c" "sh2_util.c" "shtp.c"
    INCLUDE_DIRS ".")
