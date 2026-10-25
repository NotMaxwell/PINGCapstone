/// @file
/// Standalone demo for the TF03 LiDAR.
///
/// Asks the TF03 for its firmware version, for the log, then shows each
/// distance and signal strength on the display and in the log, five times a
/// second. When nothing arrives it says what to check. Kept separate from
/// main.cpp so the application entry point stays clear of sensor bring-up
/// code. `docs/tf03-test-plan.md` walks through using it on the bench.

#include "ping/constants.hpp"
#include "ping/display.hpp"
#include "ping/tf03.hpp"

#include <esp_log.h>

using namespace ping;

extern "C" void app_main()
{
    static Display lcd;
    ESP_ERROR_CHECK(lcd.begin());

    static tf03::Tf03 lidar{kLidarUart};
    ESP_ERROR_CHECK(lidar.begin(kLidarTx, kLidarRx, kLidarBaudRate));

    lcd.show({"TF03", "starting"});

    // The TF03 streams measurements unasked, so distances need only its TxD
    // wire. Answering a command also needs its RxD wire, so the firmware
    // version is asked for once, for the log, and its answer waited for no
    // further.
    if (const auto version = lidar.firmware_version()) {
        ESP_LOGI("lidar", "TF03 firmware %u.%u.%u", version->major, version->minor, version->patch);
    } else {
        ESP_LOGW("lidar", "TF03 did not answer the firmware version request: %s. Reading distances anyway.",
                 tf03::to_string(version.error()));
    }

    RefreshTimer refresh{kLcdRefreshMs};
    for (;;) {
        const auto reading = lidar.read(kLidarSilenceMs);
        if (reading && !refresh.due()) {
            continue;
        }

        if (!reading) {
            if (reading.error().kind == tf03::Error::Kind::kTimeout) {
                lcd.show({"TF03 no data", "check power,", "brown wire, GND"});
            } else {
                lcd.show({"TF03 read failed", tf03::to_string(reading.error())});
            }
        } else if (reading->distance_cm) {
            lcd.show({
                Text("dist %9.2f m", static_cast<float>(*reading->distance_cm) / 100.0F),
                Text("strength %7u", reading->strength),
            });
        } else {
            lcd.show({
                "out of range",
                Text("raw %u cm", reading->raw_distance_cm),
                Text("strength %7u", reading->strength),
            });
        }
    }
}
