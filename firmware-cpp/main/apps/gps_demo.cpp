/// @file
/// Standalone demo for the NEO-M9N GPS.
///
/// Configures the receiver for UBX at
/// @ref ping::kGpsRateHz, shows what it is, then shows the position, its accuracy and the
/// satellite count on the display and in the log, five times a second. Kept
/// separate from main.cpp so the application entry point stays clear of
/// sensor bring-up code.

#include "ping/constants.hpp"
#include "ping/display.hpp"
#include "ping/gps.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using namespace ping;

extern "C" void app_main()
{
    static Display lcd;
    ESP_ERROR_CHECK(lcd.begin());

    static gps::Gps gps{kGpsUart};
    ESP_ERROR_CHECK(gps.begin(kGpsTx, kGpsRx, kGpsBaudRate));

    lcd.show({"GPS", "starting"});
    for (;;) {
        const auto version = gps.configure(kGpsRateHz);
        if (version) {
            lcd.show({
                "GPS up",
                version->module,
                Text("fw %s %u.%02u", version->firmware_type, version->firmware_major, version->firmware_minor),
                Text("protocol %u.%02u", version->protocol_major, version->protocol_minor),
            });
            break;
        }
        lcd.show({"GPS not ready", gps::to_string(version.error()), "retrying"});
        vTaskDelay(pdMS_TO_TICKS(kRetryDelayMs));
    }

    RefreshTimer refresh{kLcdRefreshMs};
    for (;;) {
        const auto solution = gps.read(kGpsSilenceMs);
        if (!solution) {
            lcd.show({"GPS read failed", gps::to_string(solution.error())});
            if (solution.error() == gps::Error::kTimeout) {
                // Quiet that long: probably lost power and its configuration.
                while (!gps.configure(kGpsRateHz)) {
                    vTaskDelay(pdMS_TO_TICKS(kRetryDelayMs));
                }
            }
            continue;
        }
        if (!refresh.due()) {
            continue;
        }

        if (solution->has_position()) {
            lcd.show({
                Text("lat %+12.7f", solution->latitude_deg),
                Text("lon %+12.7f", solution->longitude_deg),
                Text("acc ±%.1f m", static_cast<double>(solution->horizontal_accuracy_m)),
                Text("%u satellites", solution->satellites),
            });
        } else {
            lcd.show({"no fix", gps::to_string(solution->fix_type), Text("%u satellites", solution->satellites)});
        }
    }
}
