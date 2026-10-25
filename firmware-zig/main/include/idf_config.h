/// @file
/// The ESP-IDF configuration structs that Zig cannot fill in itself.
///
/// Each of `uart_config_t`, `i2c_master_bus_config_t` and
/// `i2c_device_config_t` has a bitfield member (their `flags`), and Zig's
/// translate-c turns any struct with a bitfield into an opaque type. These
/// functions take plain arguments, build the struct in C and make the one
/// ESP-IDF call that needs it. Everything else the firmware calls in ESP-IDF
/// it calls from Zig directly.

#pragma once

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <driver/uart.h>
#include <esp_err.h>

#include <stddef.h>
#include <stdint.h>

/// `uart_param_config` for 8N1 at `baud`, without flow control.
esp_err_t ping_uart_config_8n1(uart_port_t port, int baud);

/// `i2c_new_master_bus` on `port` with the given pins and the internal
/// pull-ups enabled.
esp_err_t ping_i2c_new_master_bus(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl,
                                  i2c_master_bus_handle_t *bus);

/// `i2c_master_bus_add_device` for a device with a 7-bit address.
esp_err_t ping_i2c_add_device(i2c_master_bus_handle_t bus, uint16_t address, uint32_t speed_hz,
                              i2c_master_dev_handle_t *device);

/// `spi_bus_initialize` on `host` with the three signal pins, DMA on, nothing
/// else (`spi_bus_config_t` is a nest of anonymous unions).
esp_err_t ping_spi_bus_initialize(spi_host_device_t host, gpio_num_t mosi, gpio_num_t miso,
                                  gpio_num_t sclk);

/// `spi_bus_add_device` for a full-duplex device in SPI `mode` at `speed_hz`
/// without a hardware chip select: the caller drives CS from a GPIO, so it
/// can stay asserted across transfers.
esp_err_t ping_spi_add_device(spi_host_device_t host, int speed_hz, uint8_t mode,
                              spi_device_handle_t *device);

/// One full-duplex polling transfer of `len` bytes. `tx` may be NULL to clock
/// out nothing in particular, `rx` may be NULL to discard what comes back.
esp_err_t ping_spi_transfer(spi_device_handle_t device, const uint8_t *tx, uint8_t *rx, size_t len);
