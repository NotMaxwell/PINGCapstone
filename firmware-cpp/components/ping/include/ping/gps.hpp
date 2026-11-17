/// @file
/// The u-blox NEO-M9N GNSS receiver (as on the SparkFun GPS-15733 breakout),
/// over an ESP-IDF UART.
///
/// The protocol lives in @ref ping::ubx, which the host tests cover; this file
/// is the UART plumbing around it, mirroring the Rust firmware's `neo_m9n`
/// module: @ref Gps::configure writes one `UBX-CFG-VALSET` (UBX only on UART1,
/// one `UBX-NAV-PVT` per solution, the rate) to the receiver's RAM and
/// battery-backed RAM layers, never flash, and @ref Gps::read waits for the
/// next solution.
///
/// Nothing here aborts. A UART FIFO overflow or a full receive ring buffer
/// flushes the input and surfaces as @ref Error::kOverflow, after which
/// reading can simply continue.

#pragma once

#include "ping/ubx.hpp"

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>

namespace ping::gps {

using ubx::FixType;
using ubx::kMaxNavRateHz;
using ubx::Solution;
using ubx::UtcTime;
using ubx::Version;
using ubx::to_string;

/// What the driver can fail at.
enum class Error : std::uint8_t {
    /// @ref Gps::begin has not succeeded.
    kNotStarted,
    /// An ESP-IDF UART call failed; the code is in the log.
    kUart,
    /// The receiver did not answer in time.
    kTimeout,
    /// The receiver refused the configuration (`UBX-ACK-NAK`).
    kRejected,
    /// The rate asked for is not one the receiver supports.
    kUnsupported,
    /// Received bytes were lost (UART FIFO overflow or full ring buffer). The
    /// input was flushed; call again.
    kOverflow,
};

const char* to_string(Error error);

/// A NEO-M9N on one of the ESP32's UARTs.
class Gps {
  public:
    explicit Gps(uart_port_t port) : port_(port) {}

    /// Installs the ESP-IDF UART driver on the port and routes it to the
    /// given pins, 8N1 at `baud`. Nothing is sent.
    esp_err_t begin(gpio_num_t tx, gpio_num_t rx, int baud);

    /// Switches UART1 to UBX-only output and sets the receiver to send one
    /// `UBX-NAV-PVT` per solution, `rate_hz` solutions a second, then asks
    /// what the receiver is. `rate_hz` must be between 1 and
    /// @ref kMaxNavRateHz. Each of the two exchanges gets one second. Stale
    /// overflow errors are ignored while waiting: the bytes lost were sent
    /// before the request, so the reply can still arrive.
    std::expected<Version, Error> configure(std::uint8_t rate_hz);

    /// Blocks until the next navigation solution or `timeout_ms`.
    ///
    /// Solutions arrive once per navigation epoch whether or not there is a
    /// fix; check @ref Solution::has_position before using the position.
    std::expected<Solution, Error> read(std::uint32_t timeout_ms);

  private:
    enum class Wait : std::uint8_t { kFrame, kTimeout, kOverflow, kUartError };

    Wait next_frame(std::int64_t deadline_us, ubx::Frame& out);
    /// Drains the driver's event queue; true if bytes were lost, in which
    /// case the input is flushed and the parser reset.
    bool handle_events();
    bool send(std::span<const std::uint8_t> bytes);

    uart_port_t port_;
    bool started_ = false;
    QueueHandle_t events_ = nullptr;
    ubx::Parser parser_;
    /// The last chunk taken from the driver, of which [rx_pos_, rx_len_) has
    /// not been handed to the parser yet.
    std::array<std::uint8_t, 64> rx_{};
    std::size_t rx_pos_ = 0;
    std::size_t rx_len_ = 0;
};

} // namespace ping::gps
