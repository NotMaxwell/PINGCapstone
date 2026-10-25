/// @file
/// Standalone demo for the BNO085 IMU.
///
/// Brings up the sensor hub over SPI, starts the rotation vector at 100 Hz and
/// shows the orientation as yaw, pitch and roll on the display and in the
/// log, five times a second. Kept separate from main.cpp so the application
/// entry point stays clear of sensor bring-up code.
///
/// The angles are those of the breakout itself, with its printed +Y axis as
/// forward; see @ref ping::geo::to_euler. Yaw is from magnetic north.

#include "ping/constants.hpp"
#include "ping/display.hpp"
#include "ping/geo.hpp"
#include "ping/imu.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <numbers>

using namespace ping;

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

        const geo::Euler angles = geo::to_euler(orientation->quaternion);
        const float accuracy_deg = orientation->heading_accuracy_rad * (180.0F / std::numbers::pi_v<float>);
        lcd.show({
            Text("yaw   %7.1f°", angles.yaw_deg),
            Text("pitch %+7.1f°", angles.pitch_deg),
            Text("roll  %+7.1f°", angles.roll_deg),
            Text("acc %s ±%.1f°", imu::to_string(orientation->calibration), accuracy_deg),
        });
    }
}
