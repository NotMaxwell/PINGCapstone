/// @file
/// The u-blox NEO-M9N GNSS receiver (SparkFun GPS-15733), through u-blox's
/// own ubxlib.
///
/// The receiver speaks NMEA text from the factory and u-blox's binary UBX on
/// the same line. This asks it for one `UBX-NAV-PVT` per navigation
/// solution, which carries position, velocity, time and their accuracy
/// estimates in one 100 byte frame, and turns NMEA off so the line carries
/// nothing else. ubxlib, vendored in vendor/ubxlib, does the UART, the UBX
/// framing, the `UBX-CFG-VALSET` configuration and the NAV-PVT decoding; this
/// class wraps its C API and turns the decoded message into a plain
/// Solution.
///
/// # Interface
///
/// UART1 of the receiver, which the SparkFun board breaks out as TX/RX at
/// 3.3 V, at its factory 38400 8N1. A NAV-PVT frame is 100 bytes, so that
/// carries even the receiver's full 25 Hz with room to spare.
///
/// # Configuration
///
/// The M9 generation is configured with key/value pairs. configure() writes
/// only the RAM layer, so nothing is left in the receiver's flash, and a
/// fresh board and a reused one behave the same after a power cycle. It is
/// also lost on a power cycle, so call it again if the receiver goes quiet.

#pragma once

#include <array>
#include <cstdint>
#include <expected>
#include <optional>

#include "freertos/FreeRTOS.h"

namespace ping::neo_m9n {

/// The baud rate the NEO-M9N's UART1 uses from the factory.
inline constexpr std::int32_t kDefaultBaudRate = 38'400;

/// The fastest navigation rate the NEO-M9N supports.
inline constexpr std::uint8_t kMaxNavRateHz = 25;

/// How the receiver is wired.
struct Config {
    /// The ESP32's UART block, 0 to 2. UART0 usually carries the console.
    std::int32_t uart = 2;
    /// ESP32 pin to the receiver's RX1.
    std::int32_t tx_pin = -1;
    /// ESP32 pin from the receiver's TX1.
    std::int32_t rx_pin = -1;
    std::int32_t baud_rate = kDefaultBaudRate;
};

/// A failure reported by ubxlib: one of its negative error codes.
struct Error {
    std::int32_t code;

    /// A static, human-readable description, for log lines.
    [[nodiscard]] const char* message() const;
    /// Nothing arrived in time.
    [[nodiscard]] bool timed_out() const;
};

template <typename T = void>
using Result = std::expected<T, Error>;

/// What kind of solution the receiver has.
enum class FixType : std::uint8_t {
    kNoFix = 0,
    /// Position from dead reckoning alone. The NEO-M9N has no sensor
    /// fusion, so this should not appear.
    kDeadReckoningOnly = 1,
    kFix2D = 2,
    kFix3D = 3,
    kGnssPlusDeadReckoning = 4,
    /// Time is known but position is not trusted.
    kTimeOnly = 5,
};

[[nodiscard]] const char* to_string(FixType fix);

/// UTC date and time of a solution.
struct UtcTime {
    std::uint16_t year;
    std::uint8_t month;
    std::uint8_t day;
    std::uint8_t hour;
    std::uint8_t minute;
    std::uint8_t second;
    /// Fraction of the second, which can be negative: the receiver rounds
    /// the other fields to the nearest second and puts the remainder here.
    std::int32_t nanosecond;
};

/// Latitude and longitude, degrees.
struct LatLon {
    double latitude_deg;
    double longitude_deg;
};

/// One navigation solution, decoded from `UBX-NAV-PVT`.
struct Solution {
    /// GPS time of week of the navigation epoch, milliseconds.
    std::uint32_t itow_ms;
    FixType fix_type;
    /// The fix is within the receiver's DOP and accuracy masks. Treat the
    /// position as usable only when this is set; position() does.
    bool fix_ok;
    /// Differential corrections (e.g. SBAS) were applied.
    bool differential;
    /// Satellites used in the solution.
    std::uint8_t satellites;
    /// Degrees, positive north.
    double latitude_deg;
    /// Degrees, positive east.
    double longitude_deg;
    /// Height above the WGS-84 ellipsoid, metres.
    float height_ellipsoid_m;
    /// Height above mean sea level, metres.
    float height_msl_m;
    /// 1σ horizontal position accuracy estimate, metres.
    float horizontal_accuracy_m;
    /// 1σ vertical position accuracy estimate, metres.
    float vertical_accuracy_m;
    /// Velocity north, east and down, m/s.
    std::array<float, 3> velocity_ned_mps;
    /// Horizontal speed, m/s.
    float ground_speed_mps;
    /// 1σ speed accuracy estimate, m/s.
    float speed_accuracy_mps;
    /// Direction of travel, degrees clockwise from north. Meaningless when
    /// standing still.
    float heading_of_motion_deg;
    /// 1σ heading accuracy estimate, degrees.
    float heading_accuracy_deg;
    /// Position dilution of precision.
    float pdop;
    /// UTC time of the solution, once the receiver has resolved it fully.
    std::optional<UtcTime> utc;
    /// The receiver flagged latitude, longitude and heights as invalid
    /// despite a fix.
    bool invalid_position;

    /// Latitude and longitude, when the receiver vouches for them.
    [[nodiscard]] std::optional<LatLon> position() const;
};

class NeoM9n {
public:
    /// Starts ubxlib. Nothing is sent to the receiver yet.
    explicit NeoM9n(const Config& config);
    ~NeoM9n();

    NeoM9n(const NeoM9n&) = delete;
    NeoM9n& operator=(const NeoM9n&) = delete;

    /// Opens the UART and has ubxlib bring the receiver up: it checks the
    /// receiver answers and starts its GNSS if stopped.
    Result<> open();

    /// Closes the UART. The receiver keeps running.
    void close();

    /// Switches UART1 to UBX-only output and sets the receiver to send one
    /// `UBX-NAV-PVT` per solution, `rate_hz` solutions a second, from 1 to
    /// kMaxNavRateHz. Written to RAM only; see the file comment.
    Result<> configure(std::uint8_t rate_hz);

    /// Waits up to `timeout_ms` for the next navigation solution. Solutions
    /// arrive once per epoch whether or not there is a fix; check
    /// Solution::position() before using the position.
    Result<Solution> read(std::int32_t timeout_ms);

private:
    Config config_;
    /// ubxlib's uDeviceHandle_t, kept opaque so ubxlib's headers, which do
    /// not stand on their own, stay out of this one.
    void* device_ = nullptr;
};

}  // namespace ping::neo_m9n
