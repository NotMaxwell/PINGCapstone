/// @file
/// Standalone demo for the BNO085 IMU.
///
/// Brings up the sensor hub over SPI, starts the rotation vector at 100 Hz and
/// shows the compass heading on the display and in the log, five times a
/// second. Kept separate from main.cpp so the application entry point stays
/// clear of sensor bring-up code.
///
/// The heading is the direction @ref ping::kLidarBoresight points, worked out
/// the same way the application aims, by @ref ping::geo::Aim::from_orientation.
/// It is shown from true north, using @ref ping::kMagneticDeclinationDeg, with
/// the nearest of the 16 compass points, and again from magnetic north. The
/// hub fuses the accelerometer with the magnetometer, so the heading holds
/// while the device is tilted. Pitch and roll are those of the breakout
/// itself; see @ref ping::geo::to_euler.

#include "ping/constants.hpp"
#include "ping/display.hpp"
#include "ping/geo.hpp"
#include "ping/imu.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>
#include <cstddef>
#include <numbers>

using namespace ping;

namespace {

/// The nearest of the 16 points of the compass to `bearing_deg`, degrees
/// clockwise from north, from 0 up to 360.
const char* compass_point(float bearing_deg)
{
    static constexpr std::array<const char*, 16> kPoints{
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",
    };
    // Each point covers 22.5°, centred on its own bearing. A bearing a hair
    // below zero still lands on north.
    const float sector = (bearing_deg / 22.5F) + 0.5F;
    const auto index = sector > 0.0F ? static_cast<std::size_t>(sector) : std::size_t{0};
    return kPoints[index % kPoints.size()];
}

} // namespace

extern "C" void app_main()
{
    static Display lcd;
    ESP_ERROR_CHECK(lcd.begin());

    static imu::Imu imu;

    lcd.show({"BNO085", "starting"});
    while (!imu.start(kImuReportIntervalUs)) {
        lcd.show({"BNO085 no reply", "retrying"});
        vTaskDelay(pdMS_TO_TICKS(kRetryDelayMs));
    }
    const sh2_ProductId_t id = imu.product_id();
    lcd.show({
        "BNO085 up",
        Text("fw %u.%u.%u", id.swVersionMajor, id.swVersionMinor, id.swVersionPatch),
        Text("reset cause %u", id.resetCause),
    });

    RefreshTimer refresh{kLcdRefreshMs};
    for (;;) {
        const auto orientation = imu.wait();
        if (!orientation || !refresh.due()) {
            continue;
        }

        const float heading =
            geo::Aim::from_orientation(orientation->quaternion, kLidarBoresight, kMagneticDeclinationDeg).azimuth_deg;
        const float magnetic = geo::Aim::from_orientation(orientation->quaternion, kLidarBoresight, 0.0F).azimuth_deg;
        const geo::Euler angles = geo::to_euler(orientation->quaternion);
        const float accuracy_deg = orientation->heading_accuracy_rad * (180.0F / std::numbers::pi_v<float>);
        lcd.show({
            Text("hdg %6.1f° %s", heading, compass_point(heading)),
            Text("mag %6.1f°", magnetic),
            Text("p%+6.1f r%+7.1f", angles.pitch_deg, angles.roll_deg),
            Text("acc %s ±%.1f°", imu::to_string(orientation->calibration), accuracy_deg),
        });
    }
}
