/// @file
/// SH-2 input report decoding tests.
///
/// The fixed point scaling is the part most likely to be quietly wrong — a
/// Q point off by one still produces plausible looking numbers — so each
/// report type gets a hand-built frame with values chosen to be exact in
/// binary floating point.

#include "ping/bno085/reports.hpp"

#include <gtest/gtest.h>

#include <vector>

using namespace ping::bno085;

namespace {

/// Little-endian 16-bit, the way every SH-2 field is laid out.
void push_le16(std::vector<std::uint8_t>& out, std::int16_t value)
{
    const auto raw = static_cast<std::uint16_t>(value);
    out.push_back(static_cast<std::uint8_t>(raw));
    out.push_back(static_cast<std::uint8_t>(raw >> 8));
}

void push_le32(std::vector<std::uint8_t>& out, std::uint32_t value)
{
    for (int shift = 0; shift < 32; shift += 8) {
        out.push_back(static_cast<std::uint8_t>(value >> shift));
    }
}

/// The four byte preamble every input report opens with.
std::vector<std::uint8_t> report_head(SensorId sensor, std::uint8_t sequence, std::uint8_t status, std::uint8_t delay_lsb)
{
    return {static_cast<std::uint8_t>(sensor), sequence, status, delay_lsb};
}

/// A raw value that means `1.0` at the given Q point.
constexpr std::int16_t one_at_q(unsigned point)
{
    return static_cast<std::int16_t>(1 << point);
}

} // namespace

TEST(Reports, NarrowsKnownSensorIdsOnly)
{
    EXPECT_EQ(sensor_from(0x05), SensorId::kRotationVector);
    EXPECT_EQ(sensor_from(0x2A), SensorId::kGyroIntegratedRotationVector);
    // 0x17 sits in a gap in the SH-2 table.
    EXPECT_FALSE(sensor_from(0x17).has_value());
    EXPECT_FALSE(sensor_from(0x00).has_value());
    EXPECT_FALSE(sensor_from(0xFB).has_value()); // base timestamp, not a sensor
}

TEST(Reports, ReportLengthsMatchTheSh2Tables)
{
    EXPECT_EQ(report_len(SensorId::kAccelerometer), 10u);
    EXPECT_EQ(report_len(SensorId::kRotationVector), 14u);
    EXPECT_EQ(report_len(SensorId::kGameRotationVector), 12u);
    EXPECT_EQ(report_len(SensorId::kGyroscopeUncalibrated), 16u);
    EXPECT_EQ(report_len(SensorId::kTapDetector), 5u);
    EXPECT_EQ(report_len(SensorId::kStepDetector), 8u);
    EXPECT_EQ(report_len(SensorId::kStabilityClassifier), 6u);
}

TEST(Reports, DecodesRotationVectorAtQ14AndQ12)
{
    auto frame = report_head(SensorId::kRotationVector, 0x11, 0x03 /* accuracy high */, 10);
    push_le16(frame, 0);                 // i
    push_le16(frame, one_at_q(14) / 2);  // j = 0.5
    push_le16(frame, 0);                 // k
    push_le16(frame, one_at_q(14));      // real = 1.0
    push_le16(frame, one_at_q(12) / 4);  // accuracy = 0.25 rad, Q12
    ASSERT_EQ(frame.size(), report_len(SensorId::kRotationVector));

    const Event event = decode(SensorId::kRotationVector, frame, 0);

    EXPECT_EQ(event.sensor, SensorId::kRotationVector);
    EXPECT_EQ(event.sequence, 0x11);
    EXPECT_EQ(event.accuracy, Accuracy::kHigh);

    const auto* rv = std::get_if<RotationVectorData>(&event.data);
    ASSERT_NE(rv, nullptr);
    EXPECT_FLOAT_EQ(rv->quaternion.i, 0.0F);
    EXPECT_FLOAT_EQ(rv->quaternion.j, 0.5F);
    EXPECT_FLOAT_EQ(rv->quaternion.k, 0.0F);
    EXPECT_FLOAT_EQ(rv->quaternion.real, 1.0F);
    EXPECT_FLOAT_EQ(rv->accuracy_rad, 0.25F);
}

TEST(Reports, DecodesAccelerometerAtQ8)
{
    auto frame = report_head(SensorId::kAccelerometer, 1, 0x02 /* medium */, 0);
    push_le16(frame, one_at_q(8));       //  1.0 m/s²
    push_le16(frame, -one_at_q(8) * 2);  // -2.0 m/s²
    push_le16(frame, one_at_q(8) / 4);   //  0.25 m/s²
    ASSERT_EQ(frame.size(), report_len(SensorId::kAccelerometer));

    const Event event = decode(SensorId::kAccelerometer, frame, 0);
    EXPECT_EQ(event.accuracy, Accuracy::kMedium);

    const auto* accel = std::get_if<AccelerometerData>(&event.data);
    ASSERT_NE(accel, nullptr);
    EXPECT_FLOAT_EQ(accel->value.x, 1.0F);
    EXPECT_FLOAT_EQ(accel->value.y, -2.0F);
    EXPECT_FLOAT_EQ(accel->value.z, 0.25F);
}

TEST(Reports, DecodesGyroscopeAtQ9)
{
    auto frame = report_head(SensorId::kGyroscopeCalibrated, 2, 0x01 /* low */, 0);
    push_le16(frame, one_at_q(9));
    push_le16(frame, 0);
    push_le16(frame, -one_at_q(9));
    const Event event = decode(SensorId::kGyroscopeCalibrated, frame, 0);

    const auto* gyro = std::get_if<GyroscopeData>(&event.data);
    ASSERT_NE(gyro, nullptr);
    EXPECT_FLOAT_EQ(gyro->value.x, 1.0F);
    EXPECT_FLOAT_EQ(gyro->value.z, -1.0F);
    EXPECT_EQ(event.accuracy, Accuracy::kLow);
}

TEST(Reports, DecodesMagneticFieldAtQ4)
{
    auto frame = report_head(SensorId::kMagneticFieldCalibrated, 3, 0x00 /* unreliable */, 0);
    push_le16(frame, one_at_q(4) * 25); // 25 µT
    push_le16(frame, 0);
    push_le16(frame, 0);
    const Event event = decode(SensorId::kMagneticFieldCalibrated, frame, 0);

    const auto* mag = std::get_if<MagneticFieldData>(&event.data);
    ASSERT_NE(mag, nullptr);
    EXPECT_FLOAT_EQ(mag->value.x, 25.0F);
    EXPECT_EQ(event.accuracy, Accuracy::kUnreliable);
}

TEST(Reports, DecodesUncalibratedGyroIntoRateAndBias)
{
    auto frame = report_head(SensorId::kGyroscopeUncalibrated, 4, 0x03, 0);
    push_le16(frame, one_at_q(9));     // rate.x = 1.0
    push_le16(frame, 0);
    push_le16(frame, 0);
    push_le16(frame, one_at_q(9) / 8); // bias.x = 0.125
    push_le16(frame, 0);
    push_le16(frame, 0);
    ASSERT_EQ(frame.size(), report_len(SensorId::kGyroscopeUncalibrated));

    const Event event = decode(SensorId::kGyroscopeUncalibrated, frame, 0);
    const auto* uncal = std::get_if<GyroscopeUncalibratedData>(&event.data);
    ASSERT_NE(uncal, nullptr);
    EXPECT_FLOAT_EQ(uncal->rate.x, 1.0F);
    EXPECT_FLOAT_EQ(uncal->bias.x, 0.125F);
}

TEST(Reports, DelayComesFromBothTheStatusByteAndTheDelayByte)
{
    // The delay is 14 bits: the top six live in the status byte's high bits,
    // the low eight in the byte after it. Both count 100 µs ticks.
    auto frame = report_head(SensorId::kAccelerometer, 0, /* status */ 0x04 | 0x02, /* delay lsb */ 0x00);
    push_le16(frame, 0);
    push_le16(frame, 0);
    push_le16(frame, 0);

    const Event event = decode(SensorId::kAccelerometer, frame, 0);
    // (0x04 << 6) == 256 ticks == 25.6 ms
    EXPECT_EQ(event.delay_us, 25'600u);
    // Accuracy is the low two bits, untouched by the delay.
    EXPECT_EQ(event.accuracy, Accuracy::kMedium);
}

TEST(Reports, TimestampOffsetFoldsInTheBaseTimestampCorrection)
{
    auto frame = report_head(SensorId::kAccelerometer, 0, 0x03, /* delay lsb */ 10);
    push_le16(frame, 0);
    push_le16(frame, 0);
    push_le16(frame, 0);

    // A base timestamp report earlier in the packet moved the reference back
    // by 5 ticks.
    const Event event = decode(SensorId::kAccelerometer, frame, -5);
    EXPECT_EQ(event.delay_us, 1'000u);              // the report's own delay
    EXPECT_EQ(event.timestamp_offset_us, 500);      // (10 - 5) * 100 µs
}

TEST(Reports, DecodesRawAccelerometerWithItsOwnTimestamp)
{
    auto frame = report_head(SensorId::kRawAccelerometer, 5, 0x03, 0);
    push_le16(frame, 1234);
    push_le16(frame, -5678);
    push_le16(frame, 9);
    push_le16(frame, 0);                 // two reserved bytes
    push_le32(frame, 0xDEADBEEF);        // hub microsecond clock
    ASSERT_EQ(frame.size(), report_len(SensorId::kRawAccelerometer));

    const Event event = decode(SensorId::kRawAccelerometer, frame, 0);
    const auto* raw = std::get_if<RawAccelerometerData>(&event.data);
    ASSERT_NE(raw, nullptr);
    EXPECT_EQ(raw->value.x, 1234);
    EXPECT_EQ(raw->value.y, -5678);
    EXPECT_EQ(raw->value.z, 9);
    EXPECT_EQ(raw->value.timestamp_us, 0xDEADBEEFu);
}

TEST(Reports, DecodesStepCounterFieldsInTheRightOrder)
{
    // Latency comes first on the wire, the count second — easy to transpose.
    auto frame = report_head(SensorId::kStepCounter, 6, 0x03, 0);
    push_le32(frame, 4'000); // latency µs
    push_le32(frame, 77);    // steps
    ASSERT_EQ(frame.size(), report_len(SensorId::kStepCounter));

    const Event event = decode(SensorId::kStepCounter, frame, 0);
    const auto* steps = std::get_if<StepCounterData>(&event.data);
    ASSERT_NE(steps, nullptr);
    EXPECT_EQ(steps->steps, 77u);
    EXPECT_EQ(steps->latency_us, 4'000u);
}

TEST(Reports, LeavesRecognisedButUndecodedSensorsAlone)
{
    auto frame = report_head(SensorId::kTemperature, 7, 0x03, 0);
    push_le16(frame, 0);

    const Event event = decode(SensorId::kTemperature, frame, 0);
    const auto* undecoded = std::get_if<UndecodedData>(&event.data);
    ASSERT_NE(undecoded, nullptr);
    EXPECT_EQ(undecoded->sensor, SensorId::kTemperature);
}

TEST(Reports, DecodesGyroIntegratedRotationVectorWithoutAPreamble)
{
    // This one arrives alone on channel 5: no report ID, no sequence, no
    // status, no delay. Quaternion at Q14, angular velocity at Q10.
    std::vector<std::uint8_t> payload;
    push_le16(payload, 0);
    push_le16(payload, 0);
    push_le16(payload, 0);
    push_le16(payload, one_at_q(14));     // real = 1.0
    push_le16(payload, one_at_q(10) * 2); // ω.x = 2.0 rad/s
    push_le16(payload, 0);
    push_le16(payload, -one_at_q(10));    // ω.z = -1.0 rad/s
    ASSERT_EQ(payload.size(), report_len(SensorId::kGyroIntegratedRotationVector));

    const Event event = decode_gyro_integrated_rv(payload);
    EXPECT_EQ(event.sensor, SensorId::kGyroIntegratedRotationVector);
    EXPECT_EQ(event.delay_us, 0u);

    const auto* girv = std::get_if<GyroIntegratedRotationVectorData>(&event.data);
    ASSERT_NE(girv, nullptr);
    EXPECT_FLOAT_EQ(girv->quaternion.real, 1.0F);
    EXPECT_FLOAT_EQ(girv->angular_velocity.x, 2.0F);
    EXPECT_FLOAT_EQ(girv->angular_velocity.z, -1.0F);
}
