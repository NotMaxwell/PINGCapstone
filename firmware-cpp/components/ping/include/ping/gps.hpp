/// @file
/// The u-blox NEO-M9N GNSS receiver (as on the SparkFun GPS-15733 breakout),
/// over UART.
///
/// The UBX protocol itself (framing, checksums, CFG-VALSET, NAV-PVT and
/// MON-VER decoding) comes from SparkFun's u-blox GNSS v4 library, which is a
/// native ESP-IDF component. This file adds the board's configuration and a
/// plain @ref Solution that does not depend on the library's types, as the
/// Rust firmware's `neo_m9n` module does on top of the `ublox` crate.
///
/// # Configuration
///
/// @ref Gps::configure turns NMEA off on UART1, asks for one `UBX-NAV-PVT`
/// per navigation solution and sets the rate, all in the receiver's RAM
/// layer only. Nothing is left behind in its flash, so a fresh board and a
/// reused one behave the same after a power cycle; the price is that a power
/// glitch puts the receiver back to factory NMEA output, which is why the
/// application configures it again whenever it goes quiet.

#pragma once

#include <SparkFun_u-blox_GNSS_v4.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_err.h>

#include <cstdint>
#include <optional>

namespace ping::gps {

/// The fastest navigation rate the NEO-M9N supports.
inline constexpr std::uint8_t kMaxNavRateHz = 25;

/// What kind of solution the receiver has.
enum class FixType : std::uint8_t {
    kNoFix = 0,
    /// Position from dead reckoning alone. The NEO-M9N has no sensor fusion,
    /// so this should not appear.
    kDeadReckoningOnly = 1,
    kFix2D = 2,
    kFix3D = 3,
    kGnssPlusDeadReckoning = 4,
    /// Time is known but position is not trusted.
    kTimeOnly = 5,
};

const char* to_string(FixType fix);

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

/// One navigation solution, decoded from `UBX-NAV-PVT`.
struct Solution {
    /// GPS time of week of the navigation epoch, milliseconds.
    std::uint32_t itow_ms;
    FixType fix_type;
    /// The fix is within the receiver's DOP and accuracy masks. Treat the
    /// position as usable only when this is set; @ref position does.
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
    float velocity_ned_mps[3];
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

    /// Whether the receiver vouches for @ref latitude_deg and
    /// @ref longitude_deg.
    [[nodiscard]] bool has_position() const
    {
        const bool positioned = fix_type == FixType::kFix2D || fix_type == FixType::kFix3D ||
                                fix_type == FixType::kGnssPlusDeadReckoning;
        return positioned && fix_ok && !invalid_position;
    }
};

/// What the receiver says it is, from `UBX-MON-VER`.
struct Version {
    /// e.g. `NEO-M9N`.
    const char* module;
    /// e.g. `SPG`.
    const char* firmware_type;
    std::uint8_t firmware_major;
    std::uint8_t firmware_minor;
    std::uint8_t protocol_major;
    std::uint8_t protocol_minor;
};

/// A NEO-M9N on one of the ESP32's UARTs.
class Gps {
  public:
    explicit Gps(uart_port_t port) : port_(port) {}

    /// Installs the ESP-IDF UART driver on the port and routes it to the
    /// given pins, 8N1 at `baud`. Nothing is sent.
    esp_err_t begin(gpio_num_t tx, gpio_num_t rx, int baud);

    /// Waits for signs of life from the receiver and reads what it is. Also
    /// a cheap way to check it is there and the baud rate matches.
    std::optional<Version> connect();

    /// Switches UART1 to UBX-only output and sets the receiver to send one
    /// `UBX-NAV-PVT` per solution, `rate_hz` solutions a second. `rate_hz`
    /// must be between 1 and @ref kMaxNavRateHz.
    bool configure(std::uint8_t rate_hz);

    /// The next navigation solution if one has arrived, without waiting.
    ///
    /// Solutions arrive once per navigation epoch whether or not there is a
    /// fix; check @ref Solution::has_position before using the position.
    std::optional<Solution> poll();

  private:
    uart_port_t port_;
    bool started_ = false;
    SFE_UBLOX_GNSS_SERIAL gnss_;
};

} // namespace ping::gps
