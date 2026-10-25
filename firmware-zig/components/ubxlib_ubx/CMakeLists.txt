# u-blox's UBX protocol encoder and decoder, and the GNSS configuration keys
# and NAV-PVT definitions, vendored unchanged from ubxlib
# (https://github.com/u-blox/ubxlib) at v1.5.0
# (0209dcf03ebe4d54a6b601f9f7a2741dbf56c785). Apache-2.0; see LICENSE.
#
# Only these files are taken. The rest of ubxlib brings its own port layer,
# UART handling and tasks, where src/gps.zig uses ESP-IDF's UART driver
# directly.
#
#   src/u_ubx_protocol.c           common/ubx_protocol/src/
#   include/u_ubx_protocol.h       common/ubx_protocol/api/
#   include/u_error_common.h       common/error/api/
#   include/u_gnss_cfg_val_key.h   gnss/api/
#   include/u_gnss_dec_ubx_nav_pvt.h  gnss/api/
idf_component_register(
    SRCS "src/u_ubx_protocol.c"
    INCLUDE_DIRS "include")
