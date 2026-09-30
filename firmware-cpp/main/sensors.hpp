/// @file
/// One FreeRTOS task per sensor, each keeping its latest reading where a
/// button press can pick it up.
///
/// The sensors are driven by their Arduino libraries (Adafruit's BNO08x,
/// SparkFun's u-blox GNSS v3 and Bud Ryerson's TFMini-Plus) over the Arduino
/// core's Wire and HardwareSerial. What is kept here is only what the
/// application uses, in its own units.

#pragma once

#include <cstdint>
#include <optional>

#include "latest.hpp"
#include "ping/geo.hpp"

namespace ping {

/// What kind of solution the GPS has: UBX-NAV-PVT's fixType.
enum class FixType : std::uint8_t {
    kNoFix = 0,
    kDeadReckoningOnly = 1,
    kFix2D = 2,
    kFix3D = 3,
    kGnssPlusDeadReckoning = 4,
    kTimeOnly = 5,
};

[[nodiscard]] const char* to_string(FixType fix);

struct LatLon {
    double latitude_deg;
    double longitude_deg;
};

/// One navigation solution from the NEO-M9N.
struct GpsFix {
    FixType fix_type;
    /// The fix is within the receiver's DOP and accuracy masks.
    bool fix_ok;
    /// The receiver flagged latitude, longitude and heights as invalid
    /// despite a fix.
    bool invalid_position;
    /// Satellites used in the solution.
    std::uint8_t satellites;
    double latitude_deg;
    double longitude_deg;
    /// Height above mean sea level, metres.
    float height_msl_m;
    /// 1σ horizontal position accuracy estimate, metres.
    float horizontal_accuracy_m;

    /// Latitude and longitude, when the receiver vouches for them.
    [[nodiscard]] std::optional<LatLon> position() const;
};

/// How far the BNO085 trusts a report, from the low two bits of its status.
/// For the rotation vector this is the magnetometer's calibration.
enum class Accuracy : std::uint8_t {
    kUnreliable = 0,
    kLow = 1,
    kMedium = 2,
    kHigh = 3,
};

[[nodiscard]] const char* to_string(Accuracy accuracy);

/// One rotation vector report from the BNO085.
struct Orientation {
    /// Turns a vector in the sensor's frame into the world frame: X east,
    /// Y magnetic north, Z up.
    geo::Quaternion quaternion;
    /// The hub's estimate of its heading error, radians.
    float heading_accuracy_rad;
    Accuracy calibration;
};

/// One distance reading from the TF03.
struct Range {
    /// Distance to the target, or nothing when nothing was in range or the
    /// return was too weak to trust.
    std::optional<std::uint16_t> distance_cm;
    /// The distance field as sent, which reads as the out-of-range threshold
    /// when there was no usable return.
    std::uint16_t raw_distance_cm;
    /// Strength of the return, 0 to 3500.
    std::uint16_t strength;
};

namespace readings {

extern Latest<GpsFix> gps;
extern Latest<Orientation> imu;
extern Latest<Range> lidar;

}  // namespace readings

namespace sensors {

// Each starts the FreeRTOS task that brings its sensor up and keeps its
// reading up to date from then on. Call initArduino() first.

/// Starts the task behind readings::gps.
void start_gps();

/// Starts the task behind readings::imu.
void start_imu();

/// Starts the task behind readings::lidar.
void start_lidar();

}  // namespace sensors

}  // namespace ping
