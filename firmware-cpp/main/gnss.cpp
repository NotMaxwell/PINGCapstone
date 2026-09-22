/// @file
/// u-blox NEO-M9N via SparkFun's u-blox GNSS v3 library (SparkFun make the
/// breakout board). The receiver is switched to UBX-only output and pushes a
/// NAV-PVT solution at the navigation rate, so getPVT() never blocks.

#include "config.hpp"
#include "samples.hpp"
#include "sensors.hpp"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "esp_log.h"
#include "esp_timer.h"

namespace ping {
namespace {

constexpr const char* kTag = "gnss";

bool configure(SFE_UBLOX_GNSS_SERIAL& gnss)
{
    // RAM layer only: nothing is written to the module's flash, so a fresh
    // board and a reused one behave the same after a power cycle.
    return gnss.setUART1Output(COM_TYPE_UBX) &&
           gnss.setNavigationFrequency(config::kGnssNavRateHz) &&
           gnss.setAutoPVT(true);
}

} // namespace

void gnss_task(void*)
{
    HardwareSerial& port = Serial2;
    port.begin(config::kGnssBaud, SERIAL_8N1, config::kGnssRx, config::kGnssTx);

    SFE_UBLOX_GNSS_SERIAL gnss;
    while (!gnss.begin(port)) {
        ESP_LOGW(kTag, "NEO-M9N not answering at %lu baud, retrying",
                 static_cast<unsigned long>(config::kGnssBaud));
        delay(1000);
    }
    while (!configure(gnss)) {
        ESP_LOGW(kTag, "NEO-M9N rejected configuration, retrying");
        delay(1000);
    }
    ESP_LOGI(kTag, "NEO-M9N up, %u Hz NAV-PVT", config::kGnssNavRateHz);

    for (;;) {
        if (gnss.getPVT()) {
            fix_mailbox.put(FixSample{
                .stamp_us = esp_timer_get_time(),
                .latitude_deg = gnss.getLatitude() * 1e-7,
                .longitude_deg = gnss.getLongitude() * 1e-7,
                .altitude_m = gnss.getAltitude() * 1e-3,
                .horizontal_accuracy_m = gnss.getHorizontalAccEst() * 1e-3,
                .vertical_accuracy_m = gnss.getVerticalAccEst() * 1e-3,
                .fix_type = gnss.getFixType(),
                .fix_ok = gnss.getGnssFixOk(),
                .differential = gnss.getDiffSoln(),
                .satellites = gnss.getSIV(),
            });
        }
        delay(10);
    }
}

} // namespace ping
