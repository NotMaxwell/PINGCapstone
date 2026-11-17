/// @file
/// The CEVA/Hillcrest BNO085 nine axis sensor hub, over SPI.
///
/// The SH-2 protocol is CEVA's own sh2 reference library (vendored unchanged
/// in `components/sh2`), running on CEVA's own reference SPI HAL ported to
/// ESP-IDF (@ref ping/shtp_spi.hpp, @ref ping/bno085_hal.hpp), so the whole
/// path to the hub is the manufacturer's code. Everything runs in the caller's
/// task: @ref Imu::next services sh2, which decodes reports into a small ring
/// buffer, and hands them out one at a time. Nothing is allocated after
/// construction.
///
/// sh2 keeps a single global instance, so there can be only one @ref Imu.

#pragma once

#include "ping/bno085_hal.hpp"
#include "ping/geo.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>

extern "C" {
#include <sh2.h>
}

namespace ping::imu {

/// The hub's identification, from its first product ID entry (the SH-2
/// application).
struct ProductId {
    std::uint8_t sw_major;
    std::uint8_t sw_minor;
    std::uint16_t sw_patch;
    std::uint32_t part_number;
    std::uint32_t build_number;
    /// 1 power-on, 2 internal system reset, 3 watchdog, 4 external reset.
    std::uint8_t reset_cause;
};

enum class SensorId : std::uint8_t {
    kGameRotationVector,
    kMagneticFieldUncalibrated,
    kRotationVector,
    kGeomagneticRotationVector,
    kMagneticField,
    kAccelerometer,
    kLinearAcceleration,
    kGravity,
    kGyroscope,
    kGyroscopeUncalibrated,
};

/// One decoded report. Which fields carry data depends on @ref sensor:
/// rotation vectors fill @ref quaternion (and @ref heading_accuracy_rad, except
/// the game rotation vector, which has no heading); three axis sensors fill
/// @ref vector (m/s², rad/s or µT); the uncalibrated ones also fill @ref bias.
struct Event {
    SensorId sensor;
    geo::Accuracy accuracy;
    geo::Quaternion quaternion;
    float heading_accuracy_rad;
    geo::Vec3 vector;
    geo::Vec3 bias;
    /// Microseconds on the HAL's clock (`esp_timer`, 32 bits extended by sh2).
    std::uint64_t timestamp_us;
};

struct Error {
    enum class Kind : std::uint8_t {
        /// ESP-IDF refused to set up the SPI bus or pins; @ref code is the
        /// `esp_err_t`.
        kInit,
        /// No INT from the hub after a hardware reset: unpowered, unwired or
        /// not in SPI mode.
        kBootTimeout,
        /// The hub booted but never announced that its reset was complete.
        kNoResetNotice,
        /// The hub did not collect a write after WAKE was asserted.
        kWakeTimeout,
        /// An SPI transfer failed.
        kBus,
        /// The hub did not answer a request in time.
        kNoResponse,
        /// The hub answered a command with an error status.
        kRejected,
        /// The hub's calibration settings are not what was asked for.
        kMismatch,
        /// The hub reset on its own; reports have stopped. Call
        /// @ref Imu::start.
        kHubReset,
        /// @ref Imu::start has not succeeded since the last failure.
        kNotStarted,
        /// sh2 returned the status in @ref code (see `sh2_err.h`).
        kSh2,
    };
    Kind kind;
    int code = 0;
};

/// A short description of an error, for logs and the display.
const char* to_string(const Error& error);

/// Counters for the SPI link and sh2 above it, to tell a quiet hub from a
/// broken link. Counts are since construction and wrap at 32 bits; times are
/// the low 32 bits of `esp_timer` microseconds.
struct LinkStats {
    std::uint32_t int_edges;
    std::uint32_t last_int_us;
    std::uint32_t transfers_ok;
    std::uint32_t transfers_failed;
    /// `esp_err_t` of the last failed transfer, ESP_OK if none.
    int last_spi_error;
    /// Cargos with a payload.
    std::uint32_t cargos_read;
    /// Header reads of length 0 (INTN with nothing to send).
    std::uint32_t empty_headers;
    /// Header reads of length 1 to 4, or all ones.
    std::uint32_t invalid_headers;
    /// Cargos longer than the 1024-byte receive buffer, truncated.
    std::uint32_t oversized_cargos;
    /// Cargos dropped because sh2's buffer was too small.
    std::uint32_t discarded_cargos;
    /// Writes abandoned because the hub never answered WAKE.
    std::uint32_t wake_timeouts;
    /// SH2_RESET events from the hub (start-up resets included).
    std::uint32_t hub_resets;
    /// SHTP errors sh2 reported (bad sequence numbers, short fragments, ...).
    std::uint32_t shtp_errors;
    /// Reports dropped because the queue was full.
    std::uint32_t dropped_events;
    std::uint32_t sensor_events;
    std::uint32_t last_event_us;
};

/// A BNO085 on the board's IMU pins (see @ref ping/constants.hpp).
class Imu {
  public:
    Imu();
    Imu(const Imu&) = delete;
    Imu& operator=(const Imu&) = delete;

    /// (Re)initialises the bus and sh2, hard-resets the hub, waits for it to
    /// announce the reset and reads its product IDs. No reports run after
    /// this. Safe to call again after any failure.
    std::expected<ProductId, Error> start();

    /// Starts `sensor` reporting every `interval_us` (0 stops it).
    std::expected<void, Error> enable_report(SensorId sensor, std::uint32_t interval_us);

    /// Chooses which sensors the hub keeps calibrating, and checks that the
    /// hub took the setting.
    std::expected<void, Error> configure_calibration(bool accel, bool gyro, bool mag);

    /// Saves the hub's running calibration to its flash.
    std::expected<void, Error> save_calibration();

    /// Zeroes the rotation vector's orientation about all three axes, until
    /// the next reset.
    std::expected<void, Error> tare();

    /// Services the hub until a report is available or `timeout_ms` passes,
    /// and returns one report, or nullopt on timeout. A hub that resets on
    /// its own, or a failed transfer, returns an error: start again.
    std::expected<std::optional<Event>, Error> next(std::uint32_t timeout_ms);

    [[nodiscard]] LinkStats link_stats() const;

  private:
    static void on_event(void* cookie, sh2_AsyncEvent_t* event);
    static void on_sensor(void* cookie, sh2_SensorEvent_t* event);

    bool wait_for_reset(std::uint32_t timeout_ms);
    /// Runs one blocking sh2 operation under the response deadline and turns
    /// what went wrong, if anything, into an Error.
    template <typename Op> std::expected<void, Error> run(Op op);
    std::optional<Error> pending_error();
    std::optional<Error> link_error();
    void push(const Event& event);
    std::optional<Event> pop();

    static constexpr std::size_t kQueueLen = 16;

    bno085::Hal hal_;
    std::array<Event, kQueueLen> queue_{};
    std::size_t head_ = 0;
    std::size_t count_ = 0;
    std::uint32_t dropped_ = 0;
    std::uint32_t hub_resets_ = 0;
    std::uint32_t shtp_errors_ = 0;
    std::uint32_t sensor_events_ = 0;
    std::uint32_t last_event_us_ = 0;
    // Link counters already turned into errors.
    std::uint32_t seen_transfers_failed_ = 0;
    std::uint32_t seen_wake_timeouts_ = 0;
    bool started_ = false;
    bool reset_seen_ = false;
};

} // namespace ping::imu
