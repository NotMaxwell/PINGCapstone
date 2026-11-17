/// @file
/// Fixed settings that do not depend on ESP-IDF: rates, aiming, calibration
/// and timing. Kept apart from the pin map in @ref ping/constants.hpp so the
/// host tests can use them. Mirrors `src/constants.rs`; keep the two in step.

#pragma once

#include "ping/calibration.hpp"
#include "ping/geo.hpp"

#include <cstdint>
#include <optional>

namespace ping {

// ---------------------------------------------------------------------------
// Sensor rates
// ---------------------------------------------------------------------------

/// How many positions a second the GPS computes.
inline constexpr std::uint8_t kGpsRateHz = 10;

/// How often the IMU reports its orientation: every 10 ms, i.e. 100 Hz.
inline constexpr std::uint32_t kImuReportIntervalUs = 10'000;

/// How often the IMU reports the magnetic field: every 20 ms, i.e. 50 Hz.
inline constexpr std::uint32_t kMagReportIntervalUs = 20'000;

// ---------------------------------------------------------------------------
// Aiming
// ---------------------------------------------------------------------------

/// The direction the LiDAR looks, in the IMU's frame: the axes printed on the
/// BNO085 breakout. The default is +Y because that is where the BNO085 itself
/// points: lying level with +Y towards magnetic north, it reports no rotation.
inline constexpr geo::Vec3 kLidarBoresight{.x = 0.0F, .y = 1.0F, .z = 0.0F};

/// The magnetometer's hard and soft iron correction, as `imu_cal` measures
/// and prints it. `std::nullopt` means it has not been calibrated: the device
/// still reports a heading, from the hard iron found during handling after
/// power up (see @ref ping::mag_cal::StartupCalibration), but marks it
/// unreliable until then.
inline constexpr std::optional<MagCalibration> kMagCalibration = std::nullopt;

/// Earth's field in Huntsville, Alabama (WMM2025, late 2026): total
/// strength, µT, which fitted fields are checked against.
inline constexpr float kExpectedFieldUt = 48.8F;

/// How far below the horizon that field points there, degrees.
inline constexpr float kExpectedDipDeg = 62.6F;

/// Everything added to the compass heading, degrees, positive clockwise. It
/// starts as Huntsville's magnetic declination, 4.3° west (WMM2025, late
/// 2026). `heading_cal` replaces it with the measured sum of declination,
/// mounting error and leftover hard iron; `docs/heading-calibration.md`
/// explains why it is one number.
inline constexpr float kHeadingCorrectionDeg = -4.3F;

/// Where the device is taken to be while the GPS has no fix, so it can be
/// tried without an antenna; every coordinate worked out from it says so.
/// `std::nullopt`: every coordinate comes from a live fix. For checking the
/// maths, `geo::Position{40.0, -90.0, 200.0}` is a good made-up point.
inline constexpr std::optional<geo::Position> kFallbackPosition = std::nullopt;

// ---------------------------------------------------------------------------
// Timing, in milliseconds
// ---------------------------------------------------------------------------

/// How often the IMU task logs the compass bearing.
inline constexpr std::uint32_t kCompassLogIntervalMs = 2'000;

/// How often the GPS task logs its fix.
inline constexpr std::uint32_t kGpsLogIntervalMs = 5'000;

// Readings older than these when the button is pressed are not used.
inline constexpr std::uint32_t kMaxGpsAgeMs = 2'000;
inline constexpr std::uint32_t kMaxImuAgeMs = 250;
inline constexpr std::uint32_t kMaxLidarAgeMs = 250;

/// A GPS this long without a solution has probably lost its configuration
/// with its backup supply; the GPS task then configures it again.
inline constexpr std::uint32_t kGpsSilenceMs = 3'000;

/// An IMU this long without a report has probably reset; the IMU task then
/// starts it again.
inline constexpr std::uint32_t kImuSilenceMs = 1'000;

/// A TF03 this long without a measurement is not sending any: it streams 100
/// a second.
inline constexpr std::uint32_t kLidarSilenceMs = 1'000;

/// How long the button has to stay down to count as pressed, or up to count
/// as released.
inline constexpr std::uint32_t kDebounceMs = 30;

/// How long a line may take to leave over the USB serial port. It only drains
/// while a computer reads it, so with nothing attached a write would block.
inline constexpr std::uint32_t kSerialTimeoutMs = 100;

/// How often the display is rewritten.
inline constexpr std::uint32_t kLcdRefreshMs = 200;

/// How long the display shows a press's result before going back to the live
/// readings.
inline constexpr std::uint32_t kTargetHoldMs = 10'000;

/// How long to wait before retrying a sensor that failed to come up.
inline constexpr std::uint32_t kRetryDelayMs = 1'000;

} // namespace ping
