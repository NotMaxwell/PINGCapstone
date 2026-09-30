# u-blox's ubxlib, vendored in ubxlib/ and cut down to what a GNSS receiver on
# a UART needs. See README.md for where it came from and what was left out.
#
# Upstream builds ubxlib for ESP-IDF through port/ubxlib.cmake, which compiles
# every "base" module (MQTT, HTTP, sockets, security, SPARTN...) whatever
# features are asked for. Listing the sources here instead keeps the build to
# the GNSS path.

set(U ubxlib)

set(srcs
    # Core utilities every module leans on.
    ${U}/common/assert/src/u_assert.c
    ${U}/common/timeout/src/u_timeout.c
    ${U}/common/ubx_protocol/src/u_ubx_protocol.c
    ${U}/common/utils/src/u_hex_bin_convert.c
    ${U}/common/utils/src/u_interface.c
    ${U}/common/utils/src/u_linked_list.c
    ${U}/common/utils/src/u_ringbuffer.c
    ${U}/common/utils/src/u_time.c
    # GNSS inside a cellular module is reached over AT commands; the GNSS
    # code links against the AT client whether or not that is used.
    ${U}/common/at_client/src/u_at_client.c
    ${U}/common/at_client/src/u_at_client_stub_short_range.c
    # Bookkeeping shared by the device, network, location and geofence
    # layers, which the GNSS code calls into even when unused.
    ${U}/common/location/src/u_location_shared.c
    ${U}/common/network/src/u_network_shared.c
    ${U}/common/geofence/src/u_geofence.c

    # The generic device API (uDeviceOpen() and friends), with the cellular
    # and short-range halves stubbed out.
    ${U}/common/device/src/u_device.c
    ${U}/common/device/src/u_device_serial.c
    ${U}/common/device/src/u_device_serial_wrapped.c
    ${U}/common/device/src/u_device_shared.c
    ${U}/common/device/src/u_device_private.c
    ${U}/common/device/src/u_device_private_gnss.c
    ${U}/common/device/src/u_device_private_cell_stub.c
    ${U}/common/device/src/u_device_private_cell_link.c
    ${U}/common/device/src/u_device_private_short_range_stub.c
    ${U}/common/device/src/u_device_private_short_range_link.c

    # The GNSS module itself.
    ${U}/gnss/src/u_gnss.c
    ${U}/gnss/src/u_gnss_cfg.c
    ${U}/gnss/src/u_gnss_dec.c
    ${U}/gnss/src/u_gnss_dec_ubx_nav_hpposllh.c
    ${U}/gnss/src/u_gnss_dec_ubx_nav_pvt.c
    ${U}/gnss/src/u_gnss_info.c
    ${U}/gnss/src/u_gnss_msg.c
    ${U}/gnss/src/u_gnss_pos.c
    ${U}/gnss/src/u_gnss_private.c
    ${U}/gnss/src/u_gnss_pwr.c
    ${U}/gnss/src/u_gnss_stub_cell.c
    ${U}/gnss/src/u_gnss_util.c

    # Platform-neutral port pieces and weak defaults.
    ${U}/port/u_port_heap.c
    ${U}/port/u_port_timezone.c
    ${U}/port/u_port_resource.c
    ${U}/port/u_port_gpio_default.c
    ${U}/port/u_port_i2c_default.c
    ${U}/port/u_port_spi_default.c
    ${U}/port/u_port_named_pipe_default.c
    ${U}/port/u_port_ppp_default.c
    ${U}/port/u_port_board_cfg.c
    ${U}/port/clib/u_port_clib_mktime64.c
    ${U}/port/platform/common/event_queue/u_port_event_queue.c
    ${U}/port/platform/common/mutex_debug/u_mutex_debug.c
    ${U}/port/platform/common/log_ram/u_log_ram.c

    # The ESP-IDF port: OS, UART, GPIO and debug output. Its I2C, SPI and PPP
    # files are replaced by port/u_port_unsupported.c; see there for why.
    ${U}/port/platform/esp-idf/src/u_port.c
    ${U}/port/platform/esp-idf/src/u_port_debug.c
    ${U}/port/platform/esp-idf/src/u_port_gpio.c
    ${U}/port/platform/esp-idf/src/u_port_os.c
    ${U}/port/platform/esp-idf/src/u_port_private.c
    ${U}/port/platform/esp-idf/src/u_port_uart.c
    port/u_port_unsupported.c)

# Every module's public headers, whether or not its sources are built: the
# device and network layers include the headers of the modules they stub.
file(GLOB api_dirs LIST_DIRECTORIES true
    ${CMAKE_CURRENT_LIST_DIR}/${U}/common/*/api
    ${CMAKE_CURRENT_LIST_DIR}/${U}/gnss/api
    ${CMAKE_CURRENT_LIST_DIR}/${U}/cell/api
    ${CMAKE_CURRENT_LIST_DIR}/${U}/ble/api
    ${CMAKE_CURRENT_LIST_DIR}/${U}/wifi/api)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS
        ${U}
        ${U}/cfg
        ${U}/port/api
        ${U}/port/clib
        ${U}/port/platform/esp-idf
        ${U}/port/platform/esp-idf/mcu/esp32/cfg
        ${U}/port/platform/common/debug_utils/api
        ${api_dirs}
    PRIV_INCLUDE_DIRS
        ${U}/common/at_client/src
        ${U}/common/device/src
        ${U}/common/location/src
        ${U}/common/network/src
        ${U}/common/geofence/src
        ${U}/gnss/src
        ${U}/gnss/src/lib_mga
        ${U}/cell/src
        ${U}/port/platform/esp-idf/src
        ${U}/port/platform/common/event_queue
        ${U}/port/platform/common/mutex_debug
        ${U}/port/platform/common/log_ram
    PRIV_REQUIRES
        esp_driver_uart
        esp_driver_gpio
        esp_timer
        esp_system
        freertos)

# ubxlib is C99 written for GCC; keep Clang's newer warnings about it quiet
# rather than touching the vendored sources.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-parameter
    -Wno-unused-but-set-variable
    -Wno-unused-function)
