//! SH-2 sensor identifiers and input report decoding.
//!
//! The hub reports fixed point values; each field has a documented Q point,
//! so a raw `i16` read as Q14 is worth `raw / 2^14`. The Q points and report
//! lengths here follow the CEVA SH-2 reference driver (`sh2.c`,
//! `sh2_SensorValue.c`).

/// Identifiers for the sensors the hub can be asked to report.
///
/// The value doubles as the feature report ID used to enable the sensor and
/// as the report ID prefixing its input reports.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
#[allow(
    missing_docs,
    reason = "the SH-2 names are self-describing and documented upstream"
)]
pub enum SensorId {
    Accelerometer = 0x01,
    GyroscopeCalibrated = 0x02,
    MagneticFieldCalibrated = 0x03,
    LinearAcceleration = 0x04,
    RotationVector = 0x05,
    Gravity = 0x06,
    GyroscopeUncalibrated = 0x07,
    GameRotationVector = 0x08,
    GeomagneticRotationVector = 0x09,
    Pressure = 0x0A,
    AmbientLight = 0x0B,
    Humidity = 0x0C,
    Proximity = 0x0D,
    Temperature = 0x0E,
    MagneticFieldUncalibrated = 0x0F,
    TapDetector = 0x10,
    StepCounter = 0x11,
    SignificantMotion = 0x12,
    StabilityClassifier = 0x13,
    RawAccelerometer = 0x14,
    RawGyroscope = 0x15,
    RawMagnetometer = 0x16,
    StepDetector = 0x18,
    ShakeDetector = 0x19,
    FlipDetector = 0x1A,
    PickupDetector = 0x1B,
    StabilityDetector = 0x1C,
    PersonalActivityClassifier = 0x1E,
    SleepDetector = 0x1F,
    TiltDetector = 0x20,
    PocketDetector = 0x21,
    CircleDetector = 0x22,
    HeartRateMonitor = 0x23,
    ArvrStabilizedRotationVector = 0x28,
    ArvrStabilizedGameRotationVector = 0x29,
    GyroIntegratedRotationVector = 0x2A,
}

impl SensorId {
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x01 => Some(Self::Accelerometer),
            0x02 => Some(Self::GyroscopeCalibrated),
            0x03 => Some(Self::MagneticFieldCalibrated),
            0x04 => Some(Self::LinearAcceleration),
            0x05 => Some(Self::RotationVector),
            0x06 => Some(Self::Gravity),
            0x07 => Some(Self::GyroscopeUncalibrated),
            0x08 => Some(Self::GameRotationVector),
            0x09 => Some(Self::GeomagneticRotationVector),
            0x0A => Some(Self::Pressure),
            0x0B => Some(Self::AmbientLight),
            0x0C => Some(Self::Humidity),
            0x0D => Some(Self::Proximity),
            0x0E => Some(Self::Temperature),
            0x0F => Some(Self::MagneticFieldUncalibrated),
            0x10 => Some(Self::TapDetector),
            0x11 => Some(Self::StepCounter),
            0x12 => Some(Self::SignificantMotion),
            0x13 => Some(Self::StabilityClassifier),
            0x14 => Some(Self::RawAccelerometer),
            0x15 => Some(Self::RawGyroscope),
            0x16 => Some(Self::RawMagnetometer),
            0x18 => Some(Self::StepDetector),
            0x19 => Some(Self::ShakeDetector),
            0x1A => Some(Self::FlipDetector),
            0x1B => Some(Self::PickupDetector),
            0x1C => Some(Self::StabilityDetector),
            0x1E => Some(Self::PersonalActivityClassifier),
            0x1F => Some(Self::SleepDetector),
            0x20 => Some(Self::TiltDetector),
            0x21 => Some(Self::PocketDetector),
            0x22 => Some(Self::CircleDetector),
            0x23 => Some(Self::HeartRateMonitor),
            0x28 => Some(Self::ArvrStabilizedRotationVector),
            0x29 => Some(Self::ArvrStabilizedGameRotationVector),
            0x2A => Some(Self::GyroIntegratedRotationVector),
            _ => None,
        }
    }

    /// Length in bytes of this sensor's input report, report ID included.
    ///
    /// A packet can hold several reports back to back, so this is what lets
    /// the reader find where the next one starts.
    pub const fn report_len(self) -> usize {
        match self {
            Self::Accelerometer
            | Self::GyroscopeCalibrated
            | Self::MagneticFieldCalibrated
            | Self::LinearAcceleration
            | Self::Gravity => 10,
            Self::RotationVector
            | Self::GeomagneticRotationVector
            | Self::ArvrStabilizedRotationVector
            | Self::GyroIntegratedRotationVector => 14,
            Self::GyroscopeUncalibrated
            | Self::MagneticFieldUncalibrated
            | Self::RawAccelerometer
            | Self::RawGyroscope
            | Self::RawMagnetometer
            | Self::PersonalActivityClassifier => 16,
            Self::GameRotationVector
            | Self::ArvrStabilizedGameRotationVector
            | Self::StepCounter => 12,
            Self::Pressure | Self::AmbientLight | Self::StepDetector | Self::PickupDetector => 8,
            Self::TapDetector => 5,
            Self::Humidity
            | Self::Proximity
            | Self::Temperature
            | Self::SignificantMotion
            | Self::StabilityClassifier
            | Self::ShakeDetector
            | Self::FlipDetector
            | Self::StabilityDetector
            | Self::SleepDetector
            | Self::TiltDetector
            | Self::PocketDetector
            | Self::CircleDetector
            | Self::HeartRateMonitor => 6,
        }
    }
}

/// How far the hub trusts its own calibration for a report.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Accuracy {
    Unreliable,
    Low,
    Medium,
    High,
}

impl Accuracy {
    const fn from_status(status: u8) -> Self {
        match status & 0x03 {
            0 => Self::Unreliable,
            1 => Self::Low,
            2 => Self::Medium,
            _ => Self::High,
        }
    }
}

/// A three axis measurement in the sensor's engineering units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// A unit quaternion describing orientation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub i: f32,
    pub j: f32,
    pub k: f32,
    pub real: f32,
}

/// An unscaled measurement straight off the ADC, with the hub's own timestamp.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RawVec3 {
    pub x: i16,
    pub y: i16,
    pub z: i16,
    /// Sample time on the hub's microsecond clock.
    pub timestamp_us: u32,
}

/// The decoded payload of a single input report.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SensorData {
    /// Acceleration including gravity, m/s².
    Accelerometer(Vec3),
    /// Acceleration with gravity removed, m/s².
    LinearAcceleration(Vec3),
    /// Gravity alone, m/s².
    Gravity(Vec3),
    /// Calibrated angular rate, rad/s.
    Gyroscope(Vec3),
    /// Angular rate before bias removal, plus the bias the hub would subtract,
    /// both rad/s.
    GyroscopeUncalibrated { rate: Vec3, bias: Vec3 },
    /// Calibrated magnetic field, µT.
    MagneticField(Vec3),
    /// Magnetic field before hard iron correction, plus that correction, µT.
    MagneticFieldUncalibrated { field: Vec3, bias: Vec3 },
    /// Absolute orientation fused from all three sensors.
    RotationVector {
        quaternion: Quaternion,
        /// Estimated heading error, radians.
        accuracy_rad: f32,
    },
    /// Orientation without magnetometer, so it drifts in yaw but never jumps.
    GameRotationVector(Quaternion),
    /// Orientation from accelerometer and magnetometer only.
    GeomagneticRotationVector {
        quaternion: Quaternion,
        accuracy_rad: f32,
    },
    /// Rotation vector tuned for AR/VR, which favours smoothness over latency.
    ArvrStabilizedRotationVector {
        quaternion: Quaternion,
        accuracy_rad: f32,
    },
    /// Game rotation vector tuned for AR/VR.
    ArvrStabilizedGameRotationVector(Quaternion),
    /// High rate orientation with the angular velocity used to integrate it.
    /// Arrives on its own channel and carries no timestamp or accuracy.
    GyroIntegratedRotationVector {
        quaternion: Quaternion,
        /// Angular velocity, rad/s.
        angular_velocity: Vec3,
    },
    RawAccelerometer(RawVec3),
    RawGyroscope(RawVec3),
    RawMagnetometer(RawVec3),
    /// Steps counted since the sensor was enabled, and how stale the count is.
    StepCounter { steps: u32, latency_us: u32 },
    StepDetector { latency_us: u32 },
    /// Tap flags: bit 0-2 are the X, Y and Z axes, bit 3 marks a double tap.
    TapDetector { flags: u8 },
    SignificantMotion { motion: u16 },
    ShakeDetector { shake: u16 },
    /// Motion class: 0 unknown, 1 on table, 2 stationary, 3 stable, 4 motion.
    StabilityClassifier { classification: u8 },
    /// A report this driver recognises but does not decode.
    Undecoded(SensorId),
}

/// One input report, with the metadata the hub attaches to it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Event {
    pub sensor: SensorId,
    pub data: SensorData,
    /// Per-sensor sequence number; gaps mean reports were dropped.
    pub sequence: u8,
    pub accuracy: Accuracy,
    /// The report's own delay field, microseconds. Useful for ordering
    /// reports that arrived batched in one packet.
    pub delay_us: u32,
    /// Offset from the moment this packet was read to the moment the sample
    /// was taken, microseconds. Normally negative, since the hub delivers a
    /// sample some time after measuring it. The driver has no clock of its
    /// own, so add this to a timestamp taken when the read returned.
    pub timestamp_offset_us: i64,
}

fn i16_at(bytes: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

fn u16_at(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

fn u32_at(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

/// Scales a fixed point value with the given Q point to a float.
fn q(raw: i16, point: u32) -> f32 {
    f32::from(raw) / (1u32 << point) as f32
}

/// Reads three consecutive Q-point values starting at `offset`.
fn vec3_at(bytes: &[u8], offset: usize, point: u32) -> Vec3 {
    Vec3 {
        x: q(i16_at(bytes, offset), point),
        y: q(i16_at(bytes, offset + 2), point),
        z: q(i16_at(bytes, offset + 4), point),
    }
}

fn quaternion_at(bytes: &[u8], offset: usize) -> Quaternion {
    Quaternion {
        i: q(i16_at(bytes, offset), 14),
        j: q(i16_at(bytes, offset + 2), 14),
        k: q(i16_at(bytes, offset + 4), 14),
        real: q(i16_at(bytes, offset + 6), 14),
    }
}

fn raw_vec3_at(bytes: &[u8], offset: usize) -> RawVec3 {
    RawVec3 {
        x: i16_at(bytes, offset),
        y: i16_at(bytes, offset + 2),
        z: i16_at(bytes, offset + 4),
        // Two reserved bytes sit between the Z axis and the timestamp.
        timestamp_us: u32_at(bytes, offset + 8),
    }
}

/// Decodes one input report.
///
/// `report` must be exactly `sensor.report_len()` bytes, starting at the
/// report ID. `reference_delta_ticks` is the correction the preceding base
/// timestamp report established, in the hub's 100 µs ticks; it is added to
/// the report's own delay.
pub(crate) fn decode(sensor: SensorId, report: &[u8], reference_delta_ticks: i32) -> Event {
    // Every report on the input channels opens the same way: report ID,
    // sequence number, a status byte whose low bits are the accuracy and
    // whose high bits are the top of the delay, then the rest of the delay.
    // Both the delay and the timebase count 100 µs ticks.
    let status = report[2];
    let delay_ticks = (u16::from(status & 0xFC) << 6) | u16::from(report[3]);

    let data = match sensor {
        SensorId::Accelerometer => SensorData::Accelerometer(vec3_at(report, 4, 8)),
        SensorId::LinearAcceleration => SensorData::LinearAcceleration(vec3_at(report, 4, 8)),
        SensorId::Gravity => SensorData::Gravity(vec3_at(report, 4, 8)),
        SensorId::GyroscopeCalibrated => SensorData::Gyroscope(vec3_at(report, 4, 9)),
        SensorId::GyroscopeUncalibrated => SensorData::GyroscopeUncalibrated {
            rate: vec3_at(report, 4, 9),
            bias: vec3_at(report, 10, 9),
        },
        SensorId::MagneticFieldCalibrated => SensorData::MagneticField(vec3_at(report, 4, 4)),
        SensorId::MagneticFieldUncalibrated => SensorData::MagneticFieldUncalibrated {
            field: vec3_at(report, 4, 4),
            bias: vec3_at(report, 10, 4),
        },
        SensorId::RotationVector => SensorData::RotationVector {
            quaternion: quaternion_at(report, 4),
            accuracy_rad: q(i16_at(report, 12), 12),
        },
        SensorId::GameRotationVector => SensorData::GameRotationVector(quaternion_at(report, 4)),
        SensorId::GeomagneticRotationVector => SensorData::GeomagneticRotationVector {
            quaternion: quaternion_at(report, 4),
            accuracy_rad: q(i16_at(report, 12), 12),
        },
        SensorId::ArvrStabilizedRotationVector => SensorData::ArvrStabilizedRotationVector {
            quaternion: quaternion_at(report, 4),
            accuracy_rad: q(i16_at(report, 12), 12),
        },
        SensorId::ArvrStabilizedGameRotationVector => {
            SensorData::ArvrStabilizedGameRotationVector(quaternion_at(report, 4))
        }
        SensorId::RawAccelerometer => SensorData::RawAccelerometer(raw_vec3_at(report, 4)),
        SensorId::RawGyroscope => SensorData::RawGyroscope(raw_vec3_at(report, 4)),
        SensorId::RawMagnetometer => SensorData::RawMagnetometer(raw_vec3_at(report, 4)),
        SensorId::StepCounter => SensorData::StepCounter {
            latency_us: u32_at(report, 4),
            steps: u32_at(report, 8),
        },
        SensorId::StepDetector => SensorData::StepDetector {
            latency_us: u32_at(report, 4),
        },
        SensorId::TapDetector => SensorData::TapDetector { flags: report[4] },
        SensorId::SignificantMotion => SensorData::SignificantMotion {
            motion: u16_at(report, 4),
        },
        SensorId::ShakeDetector => SensorData::ShakeDetector {
            shake: u16_at(report, 4),
        },
        SensorId::StabilityClassifier => SensorData::StabilityClassifier {
            classification: report[4],
        },
        other => SensorData::Undecoded(other),
    };

    Event {
        sensor,
        data,
        sequence: report[1],
        accuracy: Accuracy::from_status(status),
        delay_us: u32::from(delay_ticks) * 100,
        timestamp_offset_us: (i64::from(delay_ticks) + i64::from(reference_delta_ticks)) * 100,
    }
}

/// Decodes the gyro-integrated rotation vector.
///
/// This one arrives alone on its own channel with no report ID, sequence
/// number or timestamp, so it does not go through [`decode`].
pub(crate) fn decode_gyro_integrated_rv(payload: &[u8]) -> Event {
    Event {
        sensor: SensorId::GyroIntegratedRotationVector,
        data: SensorData::GyroIntegratedRotationVector {
            quaternion: quaternion_at(payload, 0),
            angular_velocity: vec3_at(payload, 8, 10),
        },
        sequence: 0,
        accuracy: Accuracy::Unreliable,
        delay_us: 0,
        timestamp_offset_us: 0,
    }
}
