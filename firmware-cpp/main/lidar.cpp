/// @file
/// Benewake TF03 via Bud Ryerson's TFMPlus library.
///
/// Benewake publish no driver of their own, only example sketches; TFMPlus
/// parses the same 9-byte `59 59 Dist_L Dist_H Str_L Str_H _ _ Sum` frame the
/// TF03 sends. Two TF03 differences matter and are handled here rather than
/// by patching the library:
///   - bytes 6-7 are reserved on the TF03, so TFMPlus's "temperature" output
///     is meaningless and ignored;
///   - out of range is reported as the configured over-range distance
///     (factory default 18000 cm), not 0xFFFF, and weak returns are flagged
///     only by low signal strength.

#include "config.hpp"
#include "samples.hpp"
#include "sensors.hpp"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TFMPlus.h>

#include "esp_log.h"
#include "esp_timer.h"

namespace ping {
namespace {

constexpr const char* kTag = "lidar";

/// Below this the TF03 manual says the distance is unreliable.
constexpr std::int16_t kMinStrength = 40;
/// The factory over-range sentinel. Deliberately left above the TF03-100's
/// 100 m reach so a genuine 100 m return stays distinguishable from it.
constexpr std::int16_t kOverRangeCm = 18'000;

} // namespace

void lidar_task(void*)
{
    HardwareSerial& port = Serial1;
    port.begin(config::kLidarBaud, SERIAL_8N1, config::kLidarRx, config::kLidarTx);

    TFMPlus tf03;
    tf03.begin(&port);

    // Only ask for the firmware version: the TF03 keeps its settings in
    // flash, and writing configuration on every boot risks leaving it at a
    // baud rate nobody expects.
    if (tf03.sendCommand(GET_FIRMWARE_VERSION, 0)) {
        ESP_LOGI(kTag, "TF03 firmware %u.%u.%u", tf03.version[0], tf03.version[1], tf03.version[2]);
    } else {
        ESP_LOGW(kTag, "TF03 did not answer the version query (status %u); streaming anyway", tf03.status);
    }

    std::int16_t distance = 0;
    std::int16_t strength = 0;
    std::int16_t unused_temperature = 0;

    for (;;) {
        // Blocks until a frame arrives (≤ 10 ms at the TF03's 100 Hz default).
        if (!tf03.getData(distance, strength, unused_temperature)) {
            delay(5);
            continue;
        }

        range_mailbox.put(RangeSample{
            .stamp_us = esp_timer_get_time(),
            .distance_cm = static_cast<std::uint16_t>(distance),
            .strength = static_cast<std::uint16_t>(strength),
            .valid = strength >= kMinStrength && distance > 0 && distance != kOverRangeCm,
        });
    }
}

} // namespace ping
