#include "sensors.hpp"

#include <type_traits>
#include <utility>

#include "config.hpp"
#include "esp_log.h"
#include "rtos.hpp"

namespace ping {

namespace readings {

Latest<neo_m9n::Solution> gps{"GPS"};
Latest<bno085::Orientation> imu{"IMU"};
Latest<tf03::Measurement> lidar{"LiDAR"};

}  // namespace readings

namespace sensors {

namespace {

constexpr const char* kGpsTag = "gps";
constexpr const char* kImuTag = "imu";
constexpr const char* kLidarTag = "lidar";

/// Calls `attempt` until it succeeds, logging each failure and pausing
/// between tries, and returns what it succeeded with.
template <typename Attempt>
auto retry(const char* tag, const char* what, Attempt attempt)
{
    for (;;) {
        auto result = attempt();
        if (result) {
            if constexpr (std::is_void_v<typename decltype(result)::value_type>) {
                return;
            } else {
                return *std::move(result);
            }
        }
        ESP_LOGW(tag, "%s failed: %s, retrying", what, result.error().message());
        rtos::sleep(config::kRetryDelay);
    }
}

/// Resets the hub and starts its rotation vector.
bno085::Result<bno085::ProductId> start(bno085::Bno085& imu)
{
    return imu.start().and_then([&](const bno085::ProductId& id) {
        return imu.enable_rotation_vector(config::kImuReportIntervalUs).transform([&] { return id; });
    });
}

}  // namespace

void gps_task(neo_m9n::NeoM9n& gps)
{
    retry(kGpsTag, "Opening the GPS UART", [&] { return gps.open(); });

    const auto silence_ms = static_cast<std::int32_t>(config::kGpsSilence.count());
    for (;;) {
        // The configuration lives in the receiver's RAM, so this runs again
        // whenever the receiver goes quiet, in case a power glitch has put it
        // back to its factory NMEA output.
        retry(kGpsTag, "GPS configuration", [&] { return gps.configure(config::kGpsRateHz); });
        ESP_LOGI(kGpsTag, "GPS up: %u solutions a second", config::kGpsRateHz);

        for (;;) {
            const auto solution = gps.read(silence_ms);
            if (solution) {
                readings::gps.set(*solution);
            } else if (solution.error().timed_out()) {
                ESP_LOGW(kGpsTag, "GPS went quiet, configuring it again");
                break;
            } else {
                ESP_LOGW(kGpsTag, "GPS read failed: %s", solution.error().message());
            }
        }
    }
}

void imu_task(bno085::Bno085& imu)
{
    for (;;) {
        const bno085::ProductId id = retry(kImuTag, "BNO085 bring-up", [&] { return start(imu); });
        ESP_LOGI(kImuTag, "BNO085 up: firmware %u.%u.%u, reset cause %s", id.version_major,
                 id.version_minor, id.version_patch, bno085::to_string(id.reset_cause));

        // The hub pulls INT low when it has reports waiting; poll() waits for
        // that and drains the hub.
        for (;;) {
            if (!imu.poll(rtos::ticks(config::kImuSilence))) {
                ESP_LOGW(kImuTag, "BNO085 went quiet, starting it again");
                break;
            }
            if (const auto orientation = imu.take_orientation()) {
                readings::imu.set(*orientation);
            }
            // A hub that resets on its own stops every report.
            if (imu.take_reset()) {
                ESP_LOGW(kImuTag, "BNO085 reset itself, starting it again");
                break;
            }
        }
    }
}

void lidar_task(tf03::Tf03& lidar)
{
    const tf03::FirmwareVersion version =
        retry(kLidarTag, "Asking the TF03 for its version", [&] { return lidar.firmware_version(); });
    ESP_LOGI(kLidarTag, "TF03 up: firmware %u.%u.%u", version.major, version.minor, version.patch);

    for (;;) {
        const auto measurement = lidar.read();
        if (measurement) {
            readings::lidar.set(*measurement);
        } else {
            // Typically a receive overflow; the parser resynchronises on the
            // next frame, so carry on reading.
            ESP_LOGW(kLidarTag, "TF03 read failed: %s", measurement.error().message());
        }
    }
}

}  // namespace sensors

}  // namespace ping
