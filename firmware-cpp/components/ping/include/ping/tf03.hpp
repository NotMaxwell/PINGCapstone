/// @file
/// UART driver for the Benewake TF03 long-range LiDAR (TF03-100 and TF03-180,
/// which differ only in maximum range).
///
/// The TF03 measures distance by pulsed time of flight and, unless told
/// otherwise, streams a nine byte data frame per measurement at 100 Hz
/// without being asked. Configuration commands share the same line under a
/// separate framing; see @ref tf03_protocol.hpp.
///
/// # Interface
///
/// The standard TF03 talks UART or CAN, one at a time, and leaves the factory
/// on UART. It has no I²C. Its UART is 3.3 V LVTTL, so it wires straight to
/// ESP32-S3 GPIOs. This driver sits on ESP-IDF's own UART driver, which
/// buffers what arrives in the background, so a task that is slow to read
/// loses nothing until the 2 KiB ring buffer fills, about two seconds of
/// frames.
///
/// @ref Tf03::read needs only the TF03's output line. Everything else sends a
/// command and also needs the board's TX wired to the TF03.
///
/// # Example
///
/// @code
/// ping::tf03::Tf03 lidar{ping::kLidarUart};
/// ESP_ERROR_CHECK(lidar.begin(ping::kLidarTx, ping::kLidarRx, ping::kLidarBaudRate));
/// for (;;) {
///     if (auto measurement = lidar.read(1000); measurement && measurement->distance_cm) {
///         // ...
///     }
/// }
/// @endcode

#pragma once

#include "ping/tf03_protocol.hpp"

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_err.h>

#include <array>
#include <cstdint>
#include <expected>
#include <optional>

namespace ping::tf03 {

/// The distance the TF03 reports when nothing is in range, until configured
/// otherwise with @ref Tf03::set_out_of_range_threshold.
inline constexpr std::uint16_t kDefaultOutOfRangeCm = 18'000;

/// Below this signal strength the TF03 does not trust its return, and sends
/// the out-of-range distance in place of a measurement.
inline constexpr std::uint16_t kMinReliableStrength = 40;

/// How long the TF03 may take to answer a command. The manual gives a
/// command no response within one second as failed.
inline constexpr std::uint32_t kResponseTimeoutMs = 1'000;

/// What arrived on the TF03's line while the driver waited for an answer that
/// never came, from least to most of the link proven working.
///
/// The TF03 streams measurements without being asked from the moment it is
/// powered, so listening alone shows whether it is powered and its output
/// reaches the board.
enum class Heard : std::uint8_t {
    /// Not one byte. The TF03 has no power, its TxD wire is not on the
    /// board's RX pin, or the two do not share a ground.
    kNothing,
    /// Bytes, but no sound packet: the baud rates differ, the ground is
    /// loose, or the RX wire is picking up interference.
    kNoise,
    /// Only the command just sent, copied back. The board's TX is reaching
    /// its own RX: through a short, or through an RX wire with nothing
    /// driving it picking up the TX wire beside it. Nothing else answered.
    kOwnCommand,
    /// Measurements, but no answer. The TF03 is powered and its TxD reaches
    /// the board, but the board's TX does not reach the TF03's RxD wire.
    kMeasurements,
};

const char* to_string(Heard heard);

/// What the driver can fail at.
struct Error {
    enum class Kind : std::uint8_t {
        /// The ESP-IDF UART driver returned an error; see @ref esp.
        kUart,
        /// No measurement arrived within the time asked for.
        kTimeout,
        /// The TF03 did not answer a command in time. @ref heard says what
        /// came over the line instead, which narrows down why.
        kNoReply,
        /// The TF03 answered a command with the nonzero status in @ref status.
        kRejected,
        /// The TF03 answered a command with something other than the manual
        /// describes.
        kProtocol,
        /// The TF03 does not support the value asked for.
        kUnsupported,
    };

    Kind kind;
    esp_err_t esp = ESP_OK;
    Heard heard = Heard::kNothing;
    std::uint8_t status = 0;
};

/// A human readable description of an error, for logs and the display.
const char* to_string(const Error& error);

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

/// The firmware version, which the TF03 reports as `major.minor.patch`.
struct FirmwareVersion {
    std::uint8_t major;
    std::uint8_t minor;
    std::uint8_t patch;
};

/// A TF03 on one of the ESP32's UARTs.
class Tf03 {
  public:
    explicit Tf03(uart_port_t port) : port_(port) {}

    /// Installs the ESP-IDF UART driver on the port and routes it to the
    /// given pins, 8N1 at `baud`. Nothing is sent.
    esp_err_t begin(gpio_num_t tx, gpio_num_t rx, int baud);

    /// Tells the driver about an out-of-range threshold configured on the
    /// TF03 earlier, so readings at that distance are reported as nothing.
    /// The TF03 keeps the threshold in flash and has no command to read it
    /// back.
    void set_known_out_of_range_threshold(std::uint16_t threshold_cm) { out_of_range_cm_ = threshold_cm; }

    /// Waits up to `timeout_ms` for the next measurement. Any command
    /// responses that arrive in the meantime are dropped.
    std::expected<Measurement, Error> read(std::uint32_t timeout_ms);

    /// Reads the firmware version. Also a cheap way to check the TF03 is
    /// there and both wires work.
    std::expected<FirmwareVersion, Error> firmware_version();

    /// Reboots the TF03. Settings in flash are kept.
    std::expected<void, Error> system_reset();

    /// Sets how many measurements per second the TF03 streams.
    ///
    /// The TF03 supports rates of the form `n × 10^k` for `n` from 1 to 9 and
    /// `k` from 0 to 3; Benewake rates it for up to 1000 Hz. Anything else is
    /// refused here, since the TF03 itself would quietly fall back to 100 Hz.
    std::expected<void, Error> set_frame_rate(std::uint16_t hz);

    /// Starts or stops the automatic stream of measurements. With output
    /// stopped, the TF03 measures only when @ref trigger asks.
    std::expected<void, Error> set_output_enabled(bool enabled);

    /// Takes a single measurement. Meant for when output has been stopped;
    /// otherwise it returns whichever streamed measurement arrives next.
    std::expected<Measurement, Error> trigger();

    /// Changes the TF03's UART baud rate.
    ///
    /// The host UART has to be reconfigured to match afterwards, and the
    /// manual does not say which of the two rates the TF03 answers this
    /// command at. A no-reply error here therefore does not mean the change
    /// failed: switch the host UART over and confirm with
    /// @ref firmware_version.
    std::expected<void, Error> set_baud_rate(std::uint32_t baud);

    /// Sets the distance the TF03 reports when nothing is in range, and which
    /// the driver will then report as nothing.
    std::expected<void, Error> set_out_of_range_threshold(std::uint16_t threshold_cm);

    /// Writes the current settings to flash.
    std::expected<void, Error> save_settings();

    /// Restores factory settings, including a baud rate of 115200 and a
    /// frame rate of 100 Hz.
    std::expected<void, Error> restore_defaults();

  private:
    /// Waits for the next packet of either kind until `deadline_us`, noting
    /// in `heard_` what it was.
    std::expected<Packet, Error> read_packet(std::int64_t deadline_us);
    [[nodiscard]] bool is_own_command(const Response& response) const;
    std::expected<Response, Error> transact(const Command& command, bool echoed);
    std::expected<void, Error> send(const Command& command);
    [[nodiscard]] Measurement measurement(const DataFrame& frame) const;

    uart_port_t port_;
    Parser parser_;
    std::array<std::uint8_t, 64> rx_buf_{};
    std::size_t rx_start_ = 0;
    std::size_t rx_end_ = 0;
    std::uint16_t out_of_range_cm_ = kDefaultOutOfRangeCm;
    /// The last command sent, to recognise it if it comes back.
    std::optional<Command> sent_;
    /// The most telling thing received since the last command was sent.
    Heard heard_ = Heard::kNothing;
};

} // namespace ping::tf03
