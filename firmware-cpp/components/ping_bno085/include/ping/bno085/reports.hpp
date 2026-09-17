/// @file
/// SH-2 sensor identifiers and input report decoding.
///
/// The hub reports fixed point values; each field has a documented Q point,
/// so a raw `int16_t` read as Q14 is worth `raw / 2^14`. The Q points and
/// report lengths here follow the CEVA SH-2 reference driver (`sh2.c`,
/// `sh2_SensorValue.c`).
///
/// Like @ref shtp.hpp this is pure decoding: no hardware, no ESP-IDF, so the
/// host test suite exercises it directly.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>

namespace ping::bno085 {

/// Identifiers for the sensors the hub can be asked to report.
///
/// The value doubles as the feature report ID used to enable the sensor and
/// as the report ID prefixing its input reports.
///
/// The SH-2 names are self-describing and documented upstream, so they carry
/// no per-constant comment here.
enum class SensorId : std::uint8_t {
    kAccelerometer = 0x01,
    kGyroscopeCalibrated = 0x02,
    kMagneticFieldCalibrated = 0x03,
    kLinearAcceleration = 0x04,
    kRotationVector = 0x05,
    kGravity = 0x06,
    kGyroscopeUncalibrated = 0x07,
    kGameRotationVector = 0x08,
    kGeomagneticRotationVector = 0x09,
    kPressure = 0x0A,
    kAmbientLight = 0x0B,
    kHumidity = 0x0C,
    kProximity = 0x0D,
    kTemperature = 0x0E,
    kMagneticFieldUncalibrated = 0x0F,
    kTapDetector = 0x10,
    kStepCounter = 0x11,
    kSignificantMotion = 0x12,
    kStabilityClassifier = 0x13,
    kRawAccelerometer = 0x14,
    kRawGyroscope = 0x15,
    kRawMagnetometer = 0x16,
    kStepDetector = 0x18,
    kShakeDetector = 0x19,
    kFlipDetector = 0x1A,
    kPickupDetector = 0x1B,
    kStabilityDetector = 0x1C,
    kPersonalActivityClassifier = 0x1E,
    kSleepDetector = 0x1F,
    kTiltDetector = 0x20,
    kPocketDetector = 0x21,
    kCircleDetector = 0x22,
    kHeartRateMonitor = 0x23,
    kArvrStabilizedRotationVector = 0x28,
    kArvrStabilizedGameRotationVector = 0x29,
    kGyroIntegratedRotationVector = 0x2A,
};

/// Narrows a report ID off the wire to a [`SensorId`], or nothing when the
/// hub named a sensor this driver does not know.
std::optional<SensorId> sensor_from(std::uint8_t value) noexcept;

/// Length in bytes of this sensor's input report, report ID included.
///
/// A packet can hold several reports back to back, so this is what lets the
/// reader find where the next one starts.
std::size_t report_len(SensorId sensor) noexcept;

/// How far the hub trusts its own calibration for a report.
enum class Accuracy : std::uint8_t {
    kUnreliable = 0,
    kLow = 1,
    kMedium = 2,
    kHigh = 3,
};

/// A three axis measurement in the sensor's engineering units.
struct Vec3 {
    float x;
    float y;
    float z;

    friend bool operator==(const Vec3&, const Vec3&) = default;
};

/// A unit quaternion describing orientation.
struct Quaternion {
    float i;
    float j;
    float k;
    float real;

    friend bool operator==(const Quaternion&, const Quaternion&) = default;
};

/// An unscaled measurement straight off the ADC, with the hub's own timestamp.
struct RawVec3 {
    std::int16_t x;
    std::int16_t y;
    std::int16_t z;
    /// Sample time on the hub's microsecond clock.
    std::uint32_t timestamp_us;

    friend bool operator==(const RawVec3&, const RawVec3&) = default;
};

/// @name Decoded report payloads
///
/// One struct per SH-2 report this driver understands. They are separate
/// types rather than a bare @ref Vec3 so that @ref SensorData can tell an
/// accelerometer reading from a gravity vector, which are the same shape on
/// the wire.
/// @{

/// Acceleration including gravity, m/s².
struct AccelerometerData {
    Vec3 value;
    friend bool operator==(const AccelerometerData&, const AccelerometerData&) = default;
};

/// Acceleration with gravity removed, m/s².
struct LinearAccelerationData {
    Vec3 value;
    friend bool operator==(const LinearAccelerationData&, const LinearAccelerationData&) = default;
};

/// Gravity alone, m/s².
struct GravityData {
    Vec3 value;
    friend bool operator==(const GravityData&, const GravityData&) = default;
};

/// Calibrated angular rate, rad/s.
struct GyroscopeData {
    Vec3 value;
    friend bool operator==(const GyroscopeData&, const GyroscopeData&) = default;
};

/// Angular rate before bias removal, plus the bias the hub would subtract,
/// both rad/s.
struct GyroscopeUncalibratedData {
    Vec3 rate;
    Vec3 bias;
    friend bool operator==(const GyroscopeUncalibratedData&, const GyroscopeUncalibratedData&) = default;
};

/// Calibrated magnetic field, µT.
struct MagneticFieldData {
    Vec3 value;
    friend bool operator==(const MagneticFieldData&, const MagneticFieldData&) = default;
};

/// Magnetic field before hard iron correction, plus that correction, µT.
struct MagneticFieldUncalibratedData {
    Vec3 field;
    Vec3 bias;
    friend bool operator==(const MagneticFieldUncalibratedData&, const MagneticFieldUncalibratedData&) = default;
};

/// Absolute orientation fused from all three sensors.
struct RotationVectorData {
    Quaternion quaternion;
    /// Estimated heading error, radians.
    float accuracy_rad;
    friend bool operator==(const RotationVectorData&, const RotationVectorData&) = default;
};

/// Orientation without magnetometer, so it drifts in yaw but never jumps.
struct GameRotationVectorData {
    Quaternion quaternion;
    friend bool operator==(const GameRotationVectorData&, const GameRotationVectorData&) = default;
};

/// Orientation from accelerometer and magnetometer only.
struct GeomagneticRotationVectorData {
    Quaternion quaternion;
    float accuracy_rad;
    friend bool operator==(const GeomagneticRotationVectorData&, const GeomagneticRotationVectorData&) = default;
};

/// Rotation vector tuned for AR/VR, which favours smoothness over latency.
struct ArvrStabilizedRotationVectorData {
    Quaternion quaternion;
    float accuracy_rad;
    friend bool operator==(const ArvrStabilizedRotationVectorData&, const ArvrStabilizedRotationVectorData&) = default;
};

/// Game rotation vector tuned for AR/VR.
struct ArvrStabilizedGameRotationVectorData {
    Quaternion quaternion;
    friend bool operator==(const ArvrStabilizedGameRotationVectorData&,
                           const ArvrStabilizedGameRotationVectorData&) = default;
};

/// High rate orientation with the angular velocity used to integrate it.
/// Arrives on its own channel and carries no timestamp or accuracy.
struct GyroIntegratedRotationVectorData {
    Quaternion quaternion;
    /// Angular velocity, rad/s.
    Vec3 angular_velocity;
    friend bool operator==(const GyroIntegratedRotationVectorData&,
                           const GyroIntegratedRotationVectorData&) = default;
};

struct RawAccelerometerData {
    RawVec3 value;
    friend bool operator==(const RawAccelerometerData&, const RawAccelerometerData&) = default;
};

struct RawGyroscopeData {
    RawVec3 value;
    friend bool operator==(const RawGyroscopeData&, const RawGyroscopeData&) = default;
};

struct RawMagnetometerData {
    RawVec3 value;
    friend bool operator==(const RawMagnetometerData&, const RawMagnetometerData&) = default;
};

/// Steps counted since the sensor was enabled, and how stale the count is.
struct StepCounterData {
    std::uint32_t steps;
    std::uint32_t latency_us;
    friend bool operator==(const StepCounterData&, const StepCounterData&) = default;
};

struct StepDetectorData {
    std::uint32_t latency_us;
    friend bool operator==(const StepDetectorData&, const StepDetectorData&) = default;
};

/// Tap flags: bits 0-2 are the X, Y and Z axes, bit 3 marks a double tap.
struct TapDetectorData {
    std::uint8_t flags;
    friend bool operator==(const TapDetectorData&, const TapDetectorData&) = default;
};

struct SignificantMotionData {
    std::uint16_t motion;
    friend bool operator==(const SignificantMotionData&, const SignificantMotionData&) = default;
};

struct ShakeDetectorData {
    std::uint16_t shake;
    friend bool operator==(const ShakeDetectorData&, const ShakeDetectorData&) = default;
};

/// Motion class: 0 unknown, 1 on table, 2 stationary, 3 stable, 4 motion.
struct StabilityClassifierData {
    std::uint8_t classification;
    friend bool operator==(const StabilityClassifierData&, const StabilityClassifierData&) = default;
};

/// A report this driver recognises but does not decode.
struct UndecodedData {
    SensorId sensor;
    friend bool operator==(const UndecodedData&, const UndecodedData&) = default;
};

/// @}

/// The decoded payload of a single input report.
///
/// This is the C++ spelling of the Rust original's `enum SensorData`: a
/// closed sum type, matched with `std::visit` or probed with
/// `std::holds_alternative`/`std::get_if`.
using SensorData = std::variant<AccelerometerData,
                                LinearAccelerationData,
                                GravityData,
                                GyroscopeData,
                                GyroscopeUncalibratedData,
                                MagneticFieldData,
                                MagneticFieldUncalibratedData,
                                RotationVectorData,
                                GameRotationVectorData,
                                GeomagneticRotationVectorData,
                                ArvrStabilizedRotationVectorData,
                                ArvrStabilizedGameRotationVectorData,
                                GyroIntegratedRotationVectorData,
                                RawAccelerometerData,
                                RawGyroscopeData,
                                RawMagnetometerData,
                                StepCounterData,
                                StepDetectorData,
                                TapDetectorData,
                                SignificantMotionData,
                                ShakeDetectorData,
                                StabilityClassifierData,
                                UndecodedData>;

/// One input report, with the metadata the hub attaches to it.
struct Event {
    SensorId sensor;
    SensorData data;
    /// Per-sensor sequence number; gaps mean reports were dropped.
    std::uint8_t sequence;
    Accuracy accuracy;
    /// The report's own delay field, microseconds. Useful for ordering
    /// reports that arrived batched in one packet.
    std::uint32_t delay_us;
    /// Offset from the moment this packet was read to the moment the sample
    /// was taken, microseconds. Normally negative, since the hub delivers a
    /// sample some time after measuring it. The driver has no clock of its
    /// own, so add this to a timestamp taken when the read returned.
    std::int64_t timestamp_offset_us;
};

/// Decodes one input report.
///
/// @param report must be exactly `report_len(sensor)` bytes, starting at the
///        report ID.
/// @param reference_delta_ticks is the correction the preceding base
///        timestamp report established, in the hub's 100 µs ticks; it is
///        added to the report's own delay.
Event decode(SensorId sensor, std::span<const std::uint8_t> report, std::int32_t reference_delta_ticks) noexcept;

/// Decodes the gyro-integrated rotation vector.
///
/// This one arrives alone on its own channel with no report ID, sequence
/// number or timestamp, so it does not go through @ref decode.
Event decode_gyro_integrated_rv(std::span<const std::uint8_t> payload) noexcept;

} // namespace ping::bno085
