/// @file
/// Standalone demo for the BNO085 IMU.
///
/// Brings up the sensor hub over SPI, starts the hub's own rotation vector at
/// 100 Hz and
/// shows the orientation as yaw, pitch and roll on the display and in the
/// log, five times a second. Kept separate from main.cpp so the application
/// entry point stays clear of sensor bring-up code.
///
/// The angles are those of the breakout itself, with its printed +Y axis as
/// forward; see @ref ping::geo::to_euler. Yaw is from magnetic north by the
/// hub's own magnetometer calibration, not the main program's compass. A hub
/// that goes quiet or resets is started again.

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
    for (;;) {
        const auto id = imu.start();
        if (id && imu.enable_report(imu::SensorId::kRotationVector, kImuReportIntervalUs)) {
            lcd.show({
                "BNO085 up",
                Text("fw %u.%u.%u", id->sw_major, id->sw_minor, id->sw_patch),
                Text("reset cause %u", id->reset_cause),
            });
        } else {
            lcd.show({"BNO085 failed", id ? "enable report" : imu::to_string(id.error()), "retrying"});
            vTaskDelay(pdMS_TO_TICKS(kRetryDelayMs));
            continue;
        }

        RefreshTimer refresh{kLcdRefreshMs};
        for (;;) {
            const auto event = imu.next(kImuSilenceMs);
            if (!event || !*event) {
                lcd.show({"BNO085 lost", event ? "went quiet" : imu::to_string(event.error()), "restarting"});
                break;
            }
            if ((*event)->sensor != imu::SensorId::kRotationVector || !refresh.due()) {
                continue;
            }
            const imu::Event& e = **event;
            const geo::Euler angles = geo::to_euler(e.quaternion);
            const float accuracy_deg = e.heading_accuracy_rad * (180.0F / std::numbers::pi_v<float>);
            lcd.show({
                Text("yaw   %7.1f°", static_cast<double>(angles.yaw_deg)),
                Text("pitch %+7.1f°", static_cast<double>(angles.pitch_deg)),
                Text("roll  %+7.1f°", static_cast<double>(angles.roll_deg)),
                Text("acc %s ±%.1f°", geo::to_string(e.accuracy), static_cast<double>(accuracy_deg)),
            });
        }
    }
}
