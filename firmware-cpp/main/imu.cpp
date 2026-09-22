/// @file
/// BNO085 via Adafruit_BNO08x, which wraps CEVA's SH-2 reference driver.
///
/// This replaces the hand-written SHTP driver in the Rust tree
/// (src/bno085/): framing, fragment reassembly, timestamps and fixed-point
/// decoding all happen inside CEVA's sh2.c now.

#include "config.hpp"
#include "samples.hpp"
#include "sensors.hpp"

#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <Wire.h>

#include "esp_log.h"

namespace ping {
namespace {

constexpr const char* kTag = "imu";

/// The reports one ImuSample is assembled from. The rotation vector closes a
/// sample; the others just refresh their part of it.
constexpr sh2_SensorId_t kReports[] = {
    SH2_ROTATION_VECTOR,
    SH2_GYROSCOPE_CALIBRATED,
    SH2_ACCELEROMETER,
};

bool enable_reports(Adafruit_BNO08x& imu)
{
    for (const auto report : kReports) {
        if (!imu.enableReport(report, config::kImuReportIntervalUs)) {
            ESP_LOGE(kTag, "could not enable report 0x%02x", report);
            return false;
        }
    }
    return true;
}

} // namespace

void imu_task(void*)
{
    Wire.begin(config::kImuSda, config::kImuScl, config::kImuI2cHz);
    // The BNO085 stretches SCL while it has nothing to send; give it room.
    Wire.setTimeOut(100);

    // Passing the reset pin lets the library pulse NRST during begin, which
    // recovers a hub left mid-transfer by an earlier crash.
    Adafruit_BNO08x imu{config::kImuReset};

    while (!imu.begin_I2C(config::kImuAddress, &Wire)) {
        ESP_LOGW(kTag, "BNO085 not answering at 0x%02x, retrying", config::kImuAddress);
        delay(1000);
    }
    ESP_LOGI(kTag, "BNO085 up: firmware %u.%u.%lu",
             imu.prodIds.entry[0].swVersionMajor,
             imu.prodIds.entry[0].swVersionMinor,
             static_cast<unsigned long>(imu.prodIds.entry[0].swVersionPatch));

    while (!enable_reports(imu)) {
        delay(1000);
    }

    ImuSample sample{};
    sh2_SensorValue_t value{};

    for (;;) {
        // A reset (brown-out, watchdog, ESD) silently drops every enabled
        // report, so turn them back on.
        if (imu.wasReset()) {
            ESP_LOGW(kTag, "BNO085 reset itself; re-enabling reports");
            enable_reports(imu);
        }

        while (imu.getSensorEvent(&value)) {
            switch (value.sensorId) {
            case SH2_GYROSCOPE_CALIBRATED:
                sample.gyro_x = value.un.gyroscope.x;
                sample.gyro_y = value.un.gyroscope.y;
                sample.gyro_z = value.un.gyroscope.z;
                break;
            case SH2_ACCELEROMETER:
                sample.accel_x = value.un.accelerometer.x;
                sample.accel_y = value.un.accelerometer.y;
                sample.accel_z = value.un.accelerometer.z;
                break;
            case SH2_ROTATION_VECTOR:
                sample.qx = value.un.rotationVector.i;
                sample.qy = value.un.rotationVector.j;
                sample.qz = value.un.rotationVector.k;
                sample.qw = value.un.rotationVector.real;
                sample.heading_accuracy_rad = value.un.rotationVector.accuracy;
                sample.calibration = value.status & 0x03;
                // sh2 timestamps events on the host clock it was given,
                // which in the Adafruit HAL is micros() == esp_timer.
                sample.stamp_us = static_cast<std::int64_t>(value.timestamp);
                imu_mailbox.put(sample);
                break;
            default:
                break;
            }
        }

        // Reports arrive every 10 ms; polling at 2 ms keeps latency low
        // without hammering the bus.
        delay(2);
    }
}

} // namespace ping
