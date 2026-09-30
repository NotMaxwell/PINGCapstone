/// @file
/// How the device is wired and tuned. Everything a builder is likely to
/// change lives here.

#pragma once

#include <chrono>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_types.h"
#include "driver/uart.h"
#include "ping/geo.hpp"

namespace ping::config {

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Wiring, as in the Rust demo binaries.
// ---------------------------------------------------------------------------

/// GPS: GPIO15 (TX) to the NEO-M9N board's RX1, GPIO16 (RX) to its TX1.
inline constexpr int kGpsUart = UART_NUM_2;
inline constexpr int kGpsTxPin = 15;
inline constexpr int kGpsRxPin = 16;

/// IMU: SDA on GPIO8, SCL on GPIO9, and the BNO085's INT line on GPIO10.
inline constexpr i2c_port_num_t kImuI2c = I2C_NUM_0;
inline constexpr gpio_num_t kImuSdaPin = GPIO_NUM_8;
inline constexpr gpio_num_t kImuSclPin = GPIO_NUM_9;
inline constexpr gpio_num_t kImuIntPin = GPIO_NUM_10;

/// LiDAR: GPIO17 (TX) to the TF03's blue RxD wire, GPIO18 (RX) to its brown
/// TxD wire.
inline constexpr uart_port_t kLidarUart = UART_NUM_1;
inline constexpr int kLidarTxPin = 17;
inline constexpr int kLidarRxPin = 18;

/// GPIO0 is the BOOT button on most ESP32-S3 boards, so this works with
/// nothing wired up. For a button of your own, wire it from a free GPIO to
/// ground and change the pin here. Holding GPIO0 down while the board resets
/// starts the ROM bootloader instead of this program.
inline constexpr gpio_num_t kButtonPin = GPIO_NUM_0;

// ---------------------------------------------------------------------------
// Sensor settings.
// ---------------------------------------------------------------------------

/// How many positions a second the GPS computes.
inline constexpr std::uint8_t kGpsRateHz = 10;

/// How often the IMU reports its orientation: every 10 ms, i.e. 100 Hz.
inline constexpr std::uint32_t kImuReportIntervalUs = 10'000;

/// The direction the LiDAR looks, in the IMU's frame: the axes printed on
/// the BNO085 breakout. Set this to match how the two are mounted. The
/// default is +Y because that is where the BNO085 itself points: lying level
/// with +Y towards magnetic north, it reports no rotation at all.
inline constexpr geo::Vec3 kLidarBoresight{.x = 0.0F, .y = 1.0F, .z = 0.0F};

/// How far magnetic north lies east of true north where the device is used,
/// in degrees, negative for west. The IMU measures bearings from magnetic
/// north, but coordinates are laid out from true north. Look the value up at
/// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
inline constexpr float kMagneticDeclinationDeg = 0.0F;

// ---------------------------------------------------------------------------
// Timing.
// ---------------------------------------------------------------------------

// Readings older than these when the button is pressed are not used. The
// GPS allowance covers a few missed solutions; the IMU and LiDAR report 100
// times a second, so a quarter of a second without a report means something
// is wrong.
inline constexpr std::chrono::milliseconds kMaxGpsAge = 2s;
inline constexpr std::chrono::milliseconds kMaxImuAge = 250ms;
inline constexpr std::chrono::milliseconds kMaxLidarAge = 250ms;

/// A GPS this long without a solution has probably lost power, and with it
/// its configuration, which lives in RAM. The GPS task then configures it
/// again.
inline constexpr std::chrono::milliseconds kGpsSilence = 3s;

/// An IMU this long without a report has probably reset, which stops every
/// report. The IMU task then starts it again.
inline constexpr std::chrono::milliseconds kImuSilence = 1s;

/// How long to wait before trying a sensor again after it failed to start.
inline constexpr std::chrono::milliseconds kRetryDelay = 1s;

/// How long the button has to stay down to count as pressed, or up to count
/// as released: longer than a switch's contact bounce, shorter than a tap.
inline constexpr std::chrono::milliseconds kDebounce = 30ms;

}  // namespace ping::config
