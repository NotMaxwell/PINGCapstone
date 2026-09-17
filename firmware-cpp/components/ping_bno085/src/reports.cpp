#include "ping/bno085/reports.hpp"

namespace ping::bno085 {
namespace {

std::int16_t i16_at(std::span<const std::uint8_t> bytes, std::size_t offset) noexcept
{
    return static_cast<std::int16_t>(static_cast<std::uint16_t>(bytes[offset]) |
                                     (static_cast<std::uint16_t>(bytes[offset + 1]) << 8));
}

std::uint16_t u16_at(std::span<const std::uint8_t> bytes, std::size_t offset) noexcept
{
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[offset]) |
                                      (static_cast<std::uint16_t>(bytes[offset + 1]) << 8));
}

std::uint32_t u32_at(std::span<const std::uint8_t> bytes, std::size_t offset) noexcept
{
    return static_cast<std::uint32_t>(bytes[offset]) | (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
}

/// Scales a fixed point value with the given Q point to a float.
constexpr float q(std::int16_t raw, unsigned point) noexcept
{
    return static_cast<float>(raw) / static_cast<float>(1U << point);
}

/// Reads three consecutive Q-point values starting at `offset`.
Vec3 vec3_at(std::span<const std::uint8_t> bytes, std::size_t offset, unsigned point) noexcept
{
    return Vec3{
        .x = q(i16_at(bytes, offset), point),
        .y = q(i16_at(bytes, offset + 2), point),
        .z = q(i16_at(bytes, offset + 4), point),
    };
}

Quaternion quaternion_at(std::span<const std::uint8_t> bytes, std::size_t offset) noexcept
{
    return Quaternion{
        .i = q(i16_at(bytes, offset), 14),
        .j = q(i16_at(bytes, offset + 2), 14),
        .k = q(i16_at(bytes, offset + 4), 14),
        .real = q(i16_at(bytes, offset + 6), 14),
    };
}

RawVec3 raw_vec3_at(std::span<const std::uint8_t> bytes, std::size_t offset) noexcept
{
    return RawVec3{
        .x = i16_at(bytes, offset),
        .y = i16_at(bytes, offset + 2),
        .z = i16_at(bytes, offset + 4),
        // Two reserved bytes sit between the Z axis and the timestamp.
        .timestamp_us = u32_at(bytes, offset + 8),
    };
}

constexpr Accuracy accuracy_from_status(std::uint8_t status) noexcept
{
    switch (status & 0x03) {
    case 0:
        return Accuracy::kUnreliable;
    case 1:
        return Accuracy::kLow;
    case 2:
        return Accuracy::kMedium;
    default:
        return Accuracy::kHigh;
    }
}

} // namespace

std::optional<SensorId> sensor_from(std::uint8_t value) noexcept
{
    switch (value) {
    case 0x01:
        return SensorId::kAccelerometer;
    case 0x02:
        return SensorId::kGyroscopeCalibrated;
    case 0x03:
        return SensorId::kMagneticFieldCalibrated;
    case 0x04:
        return SensorId::kLinearAcceleration;
    case 0x05:
        return SensorId::kRotationVector;
    case 0x06:
        return SensorId::kGravity;
    case 0x07:
        return SensorId::kGyroscopeUncalibrated;
    case 0x08:
        return SensorId::kGameRotationVector;
    case 0x09:
        return SensorId::kGeomagneticRotationVector;
    case 0x0A:
        return SensorId::kPressure;
    case 0x0B:
        return SensorId::kAmbientLight;
    case 0x0C:
        return SensorId::kHumidity;
    case 0x0D:
        return SensorId::kProximity;
    case 0x0E:
        return SensorId::kTemperature;
    case 0x0F:
        return SensorId::kMagneticFieldUncalibrated;
    case 0x10:
        return SensorId::kTapDetector;
    case 0x11:
        return SensorId::kStepCounter;
    case 0x12:
        return SensorId::kSignificantMotion;
    case 0x13:
        return SensorId::kStabilityClassifier;
    case 0x14:
        return SensorId::kRawAccelerometer;
    case 0x15:
        return SensorId::kRawGyroscope;
    case 0x16:
        return SensorId::kRawMagnetometer;
    case 0x18:
        return SensorId::kStepDetector;
    case 0x19:
        return SensorId::kShakeDetector;
    case 0x1A:
        return SensorId::kFlipDetector;
    case 0x1B:
        return SensorId::kPickupDetector;
    case 0x1C:
        return SensorId::kStabilityDetector;
    case 0x1E:
        return SensorId::kPersonalActivityClassifier;
    case 0x1F:
        return SensorId::kSleepDetector;
    case 0x20:
        return SensorId::kTiltDetector;
    case 0x21:
        return SensorId::kPocketDetector;
    case 0x22:
        return SensorId::kCircleDetector;
    case 0x23:
        return SensorId::kHeartRateMonitor;
    case 0x28:
        return SensorId::kArvrStabilizedRotationVector;
    case 0x29:
        return SensorId::kArvrStabilizedGameRotationVector;
    case 0x2A:
        return SensorId::kGyroIntegratedRotationVector;
    default:
        return std::nullopt;
    }
}

std::size_t report_len(SensorId sensor) noexcept
{
    switch (sensor) {
    case SensorId::kAccelerometer:
    case SensorId::kGyroscopeCalibrated:
    case SensorId::kMagneticFieldCalibrated:
    case SensorId::kLinearAcceleration:
    case SensorId::kGravity:
        return 10;

    case SensorId::kRotationVector:
    case SensorId::kGeomagneticRotationVector:
    case SensorId::kArvrStabilizedRotationVector:
    case SensorId::kGyroIntegratedRotationVector:
        return 14;

    case SensorId::kGyroscopeUncalibrated:
    case SensorId::kMagneticFieldUncalibrated:
    case SensorId::kRawAccelerometer:
    case SensorId::kRawGyroscope:
    case SensorId::kRawMagnetometer:
    case SensorId::kPersonalActivityClassifier:
        return 16;

    case SensorId::kGameRotationVector:
    case SensorId::kArvrStabilizedGameRotationVector:
    case SensorId::kStepCounter:
        return 12;

    case SensorId::kPressure:
    case SensorId::kAmbientLight:
    case SensorId::kStepDetector:
    case SensorId::kPickupDetector:
        return 8;

    case SensorId::kTapDetector:
        return 5;

    case SensorId::kHumidity:
    case SensorId::kProximity:
    case SensorId::kTemperature:
    case SensorId::kSignificantMotion:
    case SensorId::kStabilityClassifier:
    case SensorId::kShakeDetector:
    case SensorId::kFlipDetector:
    case SensorId::kStabilityDetector:
    case SensorId::kSleepDetector:
    case SensorId::kTiltDetector:
    case SensorId::kPocketDetector:
    case SensorId::kCircleDetector:
    case SensorId::kHeartRateMonitor:
        return 6;
    }

    // Unreachable for any value `sensor_from` produced; keeps the compiler
    // from warning about a fall-through on a scoped enum.
    return 0;
}

Event decode(SensorId sensor, std::span<const std::uint8_t> report, std::int32_t reference_delta_ticks) noexcept
{
    // Every report on the input channels opens the same way: report ID,
    // sequence number, a status byte whose low bits are the accuracy and
    // whose high bits are the top of the delay, then the rest of the delay.
    // Both the delay and the timebase count 100 µs ticks.
    const std::uint8_t status = report[2];
    const auto delay_ticks =
        static_cast<std::uint16_t>((static_cast<std::uint16_t>(status & 0xFC) << 6) | report[3]);

    SensorData data = UndecodedData{.sensor = sensor};

    switch (sensor) {
    case SensorId::kAccelerometer:
        data = AccelerometerData{.value = vec3_at(report, 4, 8)};
        break;
    case SensorId::kLinearAcceleration:
        data = LinearAccelerationData{.value = vec3_at(report, 4, 8)};
        break;
    case SensorId::kGravity:
        data = GravityData{.value = vec3_at(report, 4, 8)};
        break;
    case SensorId::kGyroscopeCalibrated:
        data = GyroscopeData{.value = vec3_at(report, 4, 9)};
        break;
    case SensorId::kGyroscopeUncalibrated:
        data = GyroscopeUncalibratedData{
            .rate = vec3_at(report, 4, 9),
            .bias = vec3_at(report, 10, 9),
        };
        break;
    case SensorId::kMagneticFieldCalibrated:
        data = MagneticFieldData{.value = vec3_at(report, 4, 4)};
        break;
    case SensorId::kMagneticFieldUncalibrated:
        data = MagneticFieldUncalibratedData{
            .field = vec3_at(report, 4, 4),
            .bias = vec3_at(report, 10, 4),
        };
        break;
    case SensorId::kRotationVector:
        data = RotationVectorData{
            .quaternion = quaternion_at(report, 4),
            .accuracy_rad = q(i16_at(report, 12), 12),
        };
        break;
    case SensorId::kGameRotationVector:
        data = GameRotationVectorData{.quaternion = quaternion_at(report, 4)};
        break;
    case SensorId::kGeomagneticRotationVector:
        data = GeomagneticRotationVectorData{
            .quaternion = quaternion_at(report, 4),
            .accuracy_rad = q(i16_at(report, 12), 12),
        };
        break;
    case SensorId::kArvrStabilizedRotationVector:
        data = ArvrStabilizedRotationVectorData{
            .quaternion = quaternion_at(report, 4),
            .accuracy_rad = q(i16_at(report, 12), 12),
        };
        break;
    case SensorId::kArvrStabilizedGameRotationVector:
        data = ArvrStabilizedGameRotationVectorData{.quaternion = quaternion_at(report, 4)};
        break;
    case SensorId::kRawAccelerometer:
        data = RawAccelerometerData{.value = raw_vec3_at(report, 4)};
        break;
    case SensorId::kRawGyroscope:
        data = RawGyroscopeData{.value = raw_vec3_at(report, 4)};
        break;
    case SensorId::kRawMagnetometer:
        data = RawMagnetometerData{.value = raw_vec3_at(report, 4)};
        break;
    case SensorId::kStepCounter:
        data = StepCounterData{
            .steps = u32_at(report, 8),
            .latency_us = u32_at(report, 4),
        };
        break;
    case SensorId::kStepDetector:
        data = StepDetectorData{.latency_us = u32_at(report, 4)};
        break;
    case SensorId::kTapDetector:
        data = TapDetectorData{.flags = report[4]};
        break;
    case SensorId::kSignificantMotion:
        data = SignificantMotionData{.motion = u16_at(report, 4)};
        break;
    case SensorId::kShakeDetector:
        data = ShakeDetectorData{.shake = u16_at(report, 4)};
        break;
    case SensorId::kStabilityClassifier:
        data = StabilityClassifierData{.classification = report[4]};
        break;

    // Recognised well enough to skip over, but not decoded.
    case SensorId::kPressure:
    case SensorId::kAmbientLight:
    case SensorId::kHumidity:
    case SensorId::kProximity:
    case SensorId::kTemperature:
    case SensorId::kFlipDetector:
    case SensorId::kPickupDetector:
    case SensorId::kStabilityDetector:
    case SensorId::kPersonalActivityClassifier:
    case SensorId::kSleepDetector:
    case SensorId::kTiltDetector:
    case SensorId::kPocketDetector:
    case SensorId::kCircleDetector:
    case SensorId::kHeartRateMonitor:
    case SensorId::kGyroIntegratedRotationVector:
        break;
    }

    return Event{
        .sensor = sensor,
        .data = data,
        .sequence = report[1],
        .accuracy = accuracy_from_status(status),
        .delay_us = static_cast<std::uint32_t>(delay_ticks) * 100,
        .timestamp_offset_us =
            (static_cast<std::int64_t>(delay_ticks) + static_cast<std::int64_t>(reference_delta_ticks)) * 100,
    };
}

Event decode_gyro_integrated_rv(std::span<const std::uint8_t> payload) noexcept
{
    return Event{
        .sensor = SensorId::kGyroIntegratedRotationVector,
        .data = GyroIntegratedRotationVectorData{
            .quaternion = quaternion_at(payload, 0),
            .angular_velocity = vec3_at(payload, 8, 10),
        },
        .sequence = 0,
        .accuracy = Accuracy::kUnreliable,
        .delay_us = 0,
        .timestamp_offset_us = 0,
    };
}

} // namespace ping::bno085
