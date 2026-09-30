# CEVA's SH-2 sensor hub driver, vendored unmodified in ceva-sh2/.
# See README.md for where it came from.
#
# The library is portable C with no platform code of its own; the ESP-IDF
# half, an sh2_Hal_t over the I2C master driver, lives in components/bno085.

idf_component_register(
    SRCS
        "ceva-sh2/sh2.c"
        "ceva-sh2/sh2_SensorValue.c"
        "ceva-sh2/sh2_util.c"
        "ceva-sh2/shtp.c"
        "ceva-sh2/euler.c"
    INCLUDE_DIRS "ceva-sh2")
