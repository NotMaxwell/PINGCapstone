/// @file
/// Driver for the Benewake TF03 long-range LiDAR (TF03-100 and TF03-180,
/// which differ only in maximum range), on ESP-IDF's UART driver.
///
/// The TF03 measures distance by pulsed time of flight and, unless told
/// otherwise, streams a nine byte data frame per measurement at 100 Hz
/// without being asked. Configuration commands share the same line under a
/// separate framing; see protocol.hpp.
///
/// Benewake publishes no driver of its own, only the protocol, so unlike
/// the IMU and GPS there is no vendor library underneath: this is the Rust
/// driver's logic on top of driver/uart.h.
///
/// # Interface
///
/// The standard TF03 talks UART or CAN, one at a time, and leaves the
/// factory on UART at 115200 8N1. Its UART is 3.3 V LVTTL, so it wires
/// straight to ESP32-S3 GPIOs.
///
/// # Keeping up
///
/// A data frame is 90 bits on the wire, so 115200 baud tops out near
/// 1.2 kHz of frames. The driver's receive buffer holds about a second of
/// frames at 100 Hz; a task that stops reading for longer loses bytes, and
/// the next read reports the overflow. Reading can carry on afterwards,
/// since the parser resynchronises on the next sound frame.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ping/tf03/protocol.hpp"

namespace ping::tf03 {

/// The distance the TF03 reports when nothing is in range, until configured
/// otherwise with Tf03::set_out_of_range_threshold.
inline constexpr std::uint16_t kDefaultOutOfRangeCm = 18'000;

/// Below this signal strength the TF03 does not trust its return, and sends
/// the out-of-range distance in place of a measurement.
inline constexpr std::uint16_t kMinReliableStrength = 40;

/// The TF03's factory baud rate.
inline constexpr int kDefaultBaudRate = 115'200;

/// How the TF03 is wired.
struct Config {
    uart_port_t uart = UART_NUM_1;
    /// ESP32 pin to the TF03's RxD (blue wire).
    int tx_pin = -1;
    /// ESP32 pin from the TF03's TxD (brown wire).
    int rx_pin = -1;
    int baud_rate = kDefaultBaudRate;
    /// An out-of-range threshold configured on the TF03 earlier, so readings
    /// at that distance are reported as no reading. The TF03 keeps it in
    /// flash and has no command to read it back.
    std::uint16_t out_of_range_cm = kDefaultOutOfRangeCm;
};

/// What the driver can fail at.
struct Error {
    enum class Kind : std::uint8_t {
        /// The UART driver returned an error.
        kUart,
        /// The receive buffer overflowed and bytes were lost. Reading can
        /// carry on.
        kOverflow,
        /// Nothing arrived in time.
        kTimeout,
        /// The TF03 answered a command with a nonzero status; see `status`.
        kRejected,
        /// The TF03 answered with something other than the manual describes.
        kProtocol,
        /// The TF03 does not support the value asked for.
        kUnsupported,
    };

    Kind kind;
    /// The TF03's status code, for kRejected.
    std::uint8_t status = 0;

    /// A static, human-readable description, for log lines.
    [[nodiscard]] const char* message() const;
};

template <typename T = void>
using Result = std::expected<T, Error>;

/// One distance reading.
struct Measurement {
    /// Distance to the target in centimetres, or nothing when nothing was in
    /// range or the return was too weak to trust.
    std::optional<std::uint16_t> distance_cm;
    /// The distance field as sent, which reads as the out-of-range threshold
    /// when there was no usable return.
    std::uint16_t raw_distance_cm;
    /// Strength of the return, from 0 to 3500. Readings between 40 and 1200
    /// are the most reliable; highly reflective targets read above 1500.
    std::uint16_t strength;
};

/// The firmware version, which the TF03 reports as major.minor.patch.
struct FirmwareVersion {
    std::uint8_t major;
    std::uint8_t minor;
    std::uint8_t patch;
};

class Tf03 {
public:
    /// Installs the UART driver. Nothing is sent. Aborts on a configuration
    /// ESP-IDF rejects, such as a pin that does not exist.
    explicit Tf03(const Config& config);
    ~Tf03();

    Tf03(const Tf03&) = delete;
    Tf03& operator=(const Tf03&) = delete;

    /// Waits up to `timeout` for the next measurement. Command responses
    /// that arrive in the meantime are dropped.
    Result<Measurement> read(TickType_t timeout = portMAX_DELAY);

    /// Reads the firmware version. Also a cheap way to check the TF03 is
    /// there and the baud rate matches.
    Result<FirmwareVersion> firmware_version();

    /// Reboots the TF03. Settings in flash are kept.
    Result<> system_reset();

    /// Sets how many measurements per second the TF03 streams: `n × 10^k`
    /// for n from 1 to 9 and k from 0 to 3. Anything else is refused here,
    /// since the TF03 itself would quietly fall back to 100 Hz.
    Result<> set_frame_rate(std::uint16_t hz);

    /// Starts or stops the automatic stream of measurements. With output
    /// stopped, the TF03 measures only when trigger() asks.
    Result<> set_output_enabled(bool enabled);

    /// Takes a single measurement. Meant for when output has been stopped;
    /// otherwise it returns whichever streamed measurement arrives next.
    Result<Measurement> trigger();

    /// Changes the TF03's UART baud rate. Switch the ESP32's side over with
    /// set_host_baud_rate() afterwards. The manual does not say which of the
    /// two rates the TF03 answers at, so a timeout here does not mean the
    /// change failed; confirm with firmware_version() at the new rate.
    Result<> set_baud_rate(std::uint32_t baud);

    /// Changes the ESP32's side of the link to `baud`.
    Result<> set_host_baud_rate(std::uint32_t baud);

    /// Sets the distance the TF03 reports when nothing is in range, which
    /// read() then reports as no reading.
    Result<> set_out_of_range_threshold(std::uint16_t threshold_cm);

    /// Writes the current settings to flash.
    Result<> save_settings();

    /// Restores factory settings, including 115200 baud and 100 Hz.
    Result<> restore_defaults();

private:
    Result<protocol::Packet> read_packet(TickType_t timeout);
    Result<protocol::Response> transact(const protocol::Command& command);
    Result<> send(const protocol::Command& command);
    [[nodiscard]] Measurement measurement(const protocol::DataFrame& frame) const;
    /// Drains the UART's event queue, reporting whether the receive side
    /// overflowed since last time.
    bool overflowed();

    uart_port_t uart_;
    QueueHandle_t events_ = nullptr;
    protocol::Parser parser_;
    /// The last chunk read from the UART, and how far through it the parser
    /// has got. A packet can end part way through a chunk; the rest waits
    /// here for the next read.
    std::array<std::uint8_t, 64> rx_buf_{};
    std::size_t rx_start_ = 0;
    std::size_t rx_end_ = 0;
    std::uint16_t out_of_range_cm_;
};

}  // namespace ping::tf03
