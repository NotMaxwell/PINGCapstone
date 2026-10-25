/// @file
/// The CEVA/Hillcrest BNO085 nine axis sensor hub, over SPI.
///
/// The SH-2 protocol is handled by CEVA's own reference library, by way of
/// Myles Parfeniuk's esp32_BNO08x component, which supplies the SPI HAL that
/// library needs and runs it in its own FreeRTOS tasks. This file only brings
/// it up on the board's pins and hands out what the application uses from
/// each rotation vector report.
///
/// # SPI mode and the WAKE pin
///
/// The hub selects SPI when PS0 and PS1 are both high as it leaves reset. PS1
/// is tied high on the board; PS0 is the breakout's P0, wired to
/// @ref ping::kImuWake. The library does not drive a wake line, so this holds
/// it high before the library's first reset and leaves it there.

#pragma once

#include <BNO08x.hpp>

#include "ping/geo.hpp"

#include <cstdint>
#include <optional>

namespace ping::imu {

/// What the application keeps from each rotation vector report.
struct Orientation {
    geo::Quaternion quaternion;
    /// The hub's estimate of its heading error, radians.
    float heading_accuracy_rad;
    /// How far the hub trusts its magnetometer calibration.
    BNO08xAccuracy calibration;
};

/// A short name for an accuracy, for logs and the display.
const char* to_string(BNO08xAccuracy accuracy);

/// A BNO085 on the board's IMU pins (see @ref ping/constants.hpp).
class Imu {
  public:
    /// Resets the hub and starts its rotation vector every `interval_us`
    /// microseconds. Safe to call again to start over: the first call brings
    /// the library up, later ones reset the hub, after which the library
    /// enables the same reports again by itself.
    bool start(std::uint32_t interval_us);

    /// Identification the hub sent while starting, valid after @ref start.
    [[nodiscard]] sh2_ProductId_t product_id();

    /// Waits for the next rotation vector report, for at most the library's
    /// data timeout (`CONFIG_ESP32_BNO08X_DATA_AVAILABLE_TIMEOUT_MS`).
    std::optional<Orientation> wait();

  private:
    std::optional<BNO08x> hub_;
    bool running_ = false;
};

} // namespace ping::imu
