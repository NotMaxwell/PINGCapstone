/// @file
/// The narrow hardware interface the sensor drivers are written against.
///
/// The Rust original is generic over `embedded_hal_async`'s `I2c` and
/// `DelayNs` traits, so a driver can be exercised against a mock bus on the
/// host and against the real peripheral on the chip without changing a line.
/// These concepts are that idea in C++: a driver takes its bus as a template
/// parameter, the concept states what the driver needs, and the compiler
/// checks the requirement at the point of instantiation rather than at the
/// first confusing error deep inside the driver.
///
/// Nothing here includes an ESP-IDF header. @ref ping/hal/esp_bus.hpp holds
/// the implementations that do.

#pragma once

#include <concepts>
#include <cstdint>
#include <expected>
#include <span>

namespace ping::hal {

/// A bus level failure, carried verbatim from the underlying driver.
///
/// On the ESP32 `code` is an `esp_err_t`; the host test doubles use their own
/// small integers. Keeping it opaque here is what lets the pure driver code
/// stay free of ESP-IDF headers.
struct BusError {
    std::int32_t code = 0;

    friend bool operator==(const BusError&, const BusError&) = default;
};

/// The result of a transfer that returns nothing but can fail.
using BusResult = std::expected<void, BusError>;

/// An I²C controller that can address a 7-bit peripheral.
///
/// `read` and `write` correspond to a single bus transaction each; a driver
/// that needs a repeated start uses `write_read`.
template <typename T>
concept I2cBus =
    requires(T& bus, std::uint8_t address, std::span<std::uint8_t> in, std::span<const std::uint8_t> out) {
        { bus.read(address, in) } -> std::same_as<BusResult>;
        { bus.write(address, out) } -> std::same_as<BusResult>;
        { bus.write_read(address, out, in) } -> std::same_as<BusResult>;
    };

/// A byte stream, i.e. a UART.
///
/// `read` returns how many bytes actually arrived, which may be fewer than
/// asked for if the timeout expired first — that is the normal case when
/// polling a sensor that streams at its own rate.
template <typename T>
concept ByteStream =
    requires(T& port, std::span<std::uint8_t> in, std::span<const std::uint8_t> out, std::uint32_t timeout_ms) {
        { port.read(in, timeout_ms) } -> std::same_as<std::expected<std::size_t, BusError>>;
        { port.write(out) } -> std::same_as<BusResult>;
        { port.flush_input() } -> std::same_as<void>;
    };

/// Somewhere to sleep, and somewhere to read a monotonic clock.
///
/// `delay_ms` must yield to the scheduler rather than spin, so a driver that
/// waits on a slow sensor does not starve the rest of the firmware.
template <typename T>
concept Clock = requires(T& clock, std::uint32_t ms) {
    { clock.delay_ms(ms) } -> std::same_as<void>;
    { clock.now_us() } -> std::same_as<std::int64_t>;
};

/// Something that blocks until a sensor's interrupt line asserts.
///
/// The BNO085 pulls its `INT` line low when it has a packet waiting. On the
/// chip this is a GPIO ISR handing a semaphore to the reading task — the
/// equivalent of the Rust original's `int.wait_for_low().await`.
template <typename T>
concept InterruptLine = requires(T& line, std::uint32_t timeout_ms) {
    { line.wait(timeout_ms) } -> std::same_as<bool>;
};

} // namespace ping::hal
