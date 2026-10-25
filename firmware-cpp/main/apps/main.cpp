/// @file
/// PING: aim the device at something, press the button, and it reports that
/// thing's GPS coordinate.
///
/// Each sensor runs in its own FreeRTOS task and keeps its latest reading in a
/// shared slot:
///
/// * the NEO-M9N GPS gives the device's own position,
/// * the BNO085 IMU gives its orientation, and from that the direction the
///   LiDAR points: a compass bearing and an elevation angle,
/// * the TF03 LiDAR gives the distance to whatever it points at.
///
/// When the button is pressed, the main task takes the latest reading from
/// each and walks the LiDAR's range out from the GPS position along that
/// direction (see @ref ping/geo.hpp). The result goes to the USB serial port,
/// one line per press, alongside the logs, where `idf.py monitor` or any
/// serial monitor shows it.
///
/// Before trusting the bearings, set @ref ping::kLidarBoresight to how the
/// LiDAR is mounted against the IMU and @ref ping::kMagneticDeclinationDeg to
/// where the device is used. Both live in @ref ping/constants.hpp, with the
/// pin map and every other fixed setting.

#include "ping/constants.hpp"
#include "ping/geo.hpp"
#include "ping/gps.hpp"
#include "ping/imu.hpp"
#include "ping/tf03.hpp"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <expected>
#include <mutex>
#include <numbers>
#include <optional>
#include <string>

namespace {

using namespace ping;

constexpr const char* kTag = "ping";

/// Each sensor task's stack, in bytes.
constexpr std::uint32_t kSensorStack = 6 * 1024;

/// How often the main task looks at the button.
constexpr std::uint32_t kButtonPollMs = 5;

std::int64_t now_ms() { return esp_timer_get_time() / 1000; }

std::string format(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
std::string format(const char* fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, args);
    va_end(args);
    return buf;
}

/// Why a press produced no coordinate, as a sentence.
using Problem = std::string;

/// The latest reading from one sensor, and when it arrived. The sensor's task
/// overwrites it; a button press copies it out.
template <typename T>
class Latest {
  public:
    explicit Latest(const char* sensor) : sensor_(sensor) {}

    void set(const T& value)
    {
        const std::lock_guard lock(mutex_);
        reading_ = value;
        at_ms_ = now_ms();
    }

    /// The reading, as long as it is no older than `max_age_ms`.
    std::expected<T, Problem> get(std::uint32_t max_age_ms)
    {
        const std::lock_guard lock(mutex_);
        if (!reading_) {
            return std::unexpected(format("no %s reading yet", sensor_));
        }
        const std::int64_t age = now_ms() - at_ms_;
        if (age > max_age_ms) {
            return std::unexpected(format("last %s reading is %lld ms old", sensor_, static_cast<long long>(age)));
        }
        return *reading_;
    }

  private:
    const char* sensor_;
    std::mutex mutex_;
    std::optional<T> reading_;
    std::int64_t at_ms_ = 0;
};

Latest<gps::Solution> g_gps{"GPS"};
Latest<imu::Orientation> g_imu{"IMU"};
Latest<tf03::Measurement> g_lidar{"LiDAR"};

/// A located target, and what it was located from.
struct Sighting {
    geo::Position target;
    geo::Position origin;
    geo::Aim aim;
    float range_m;
    /// The GPS's 1σ estimate of its horizontal error, metres.
    float origin_accuracy_m;
    std::uint8_t satellites;
    /// The IMU's estimate of its heading error, degrees.
    float heading_accuracy_deg;
    /// How far the IMU trusts its magnetometer calibration.
    BNO08xAccuracy compass;

    [[nodiscard]] std::string describe() const
    {
        return format("%.7f, %.7f, alt %.1f m"
                      " | range %.2f m, bearing %.1f deg, elevation %.1f deg"
                      " | from %.7f, %.7f, alt %.1f m, +/-%.1f m, %u satellites"
                      " | heading +/-%.1f deg, compass calibration %s",
                      target.latitude_deg, target.longitude_deg, target.altitude_m, range_m, aim.azimuth_deg,
                      aim.elevation_deg, origin.latitude_deg, origin.longitude_deg, origin.altitude_m,
                      origin_accuracy_m, satellites, heading_accuracy_deg, imu::to_string(compass));
    }
};

/// Locates what the LiDAR is pointing at, from the latest sensor readings.
std::expected<Sighting, Problem> take_sighting()
{
    const auto solution = g_gps.get(kMaxGpsAgeMs);
    if (!solution) {
        return std::unexpected(solution.error());
    }
    const auto orientation = g_imu.get(kMaxImuAgeMs);
    if (!orientation) {
        return std::unexpected(orientation.error());
    }
    const auto measurement = g_lidar.get(kMaxLidarAgeMs);
    if (!measurement) {
        return std::unexpected(measurement.error());
    }

    if (!solution->has_position()) {
        return std::unexpected(format("GPS has no usable fix (%s, %u satellites)", gps::to_string(solution->fix_type),
                                      solution->satellites));
    }
    if (!measurement->distance_cm) {
        return std::unexpected(format("LiDAR has no return it trusts (strength %u)", measurement->strength));
    }

    const geo::Position origin{
        .latitude_deg = solution->latitude_deg,
        .longitude_deg = solution->longitude_deg,
        .altitude_m = solution->height_msl_m,
    };
    const geo::Aim aim = geo::Aim::from_orientation(orientation->quaternion, kLidarBoresight, kMagneticDeclinationDeg);
    const float range_m = static_cast<float>(*measurement->distance_cm) / 100.0F;

    return Sighting{
        .target = origin.project(aim, range_m),
        .origin = origin,
        .aim = aim,
        .range_m = range_m,
        .origin_accuracy_m = solution->horizontal_accuracy_m,
        .satellites = solution->satellites,
        .heading_accuracy_deg = orientation->heading_accuracy_rad * (180.0F / std::numbers::pi_v<float>),
        .compass = orientation->calibration,
    };
}

/// Keeps the latest navigation solution from the NEO-M9N.
void gps_task(void* /*arg*/)
{
    static gps::Gps gps{kGpsUart};
    ESP_ERROR_CHECK(gps.begin(kGpsTx, kGpsRx, kGpsBaudRate));

    for (;;) {
        // The configuration lives in the receiver's RAM, so this runs again
        // whenever the receiver goes quiet, in case a power glitch has put it
        // back to its factory NMEA output.
        while (!gps.configure(kGpsRateHz)) {
            ESP_LOGW(kTag, "GPS configuration failed, retrying");
            vTaskDelay(pdMS_TO_TICKS(kRetryDelayMs));
        }
        ESP_LOGI(kTag, "GPS up: %u solutions a second", kGpsRateHz);

        std::int64_t last = now_ms();
        for (;;) {
            if (auto solution = gps.poll()) {
                g_gps.set(*solution);
                last = now_ms();
            } else if (now_ms() - last > kGpsSilenceMs) {
                ESP_LOGW(kTag, "GPS went quiet, configuring it again");
                break;
            } else {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}

/// Keeps the latest orientation from the BNO085.
void imu_task(void* /*arg*/)
{
    static imu::Imu imu;

    for (;;) {
        if (!imu.start(kImuReportIntervalUs)) {
            ESP_LOGW(kTag, "BNO085 bring-up failed, retrying");
            vTaskDelay(pdMS_TO_TICKS(kRetryDelayMs));
            continue;
        }
        const sh2_ProductId_t id = imu.product_id();
        ESP_LOGI(kTag, "BNO085 up: firmware %u.%u.%u, reset cause %u", id.swVersionMajor, id.swVersionMinor,
                 id.swVersionPatch, id.resetCause);

        std::int64_t last = now_ms();
        for (;;) {
            if (auto orientation = imu.wait()) {
                g_imu.set(*orientation);
                last = now_ms();
            } else if (now_ms() - last > kImuSilenceMs) {
                ESP_LOGW(kTag, "BNO085 went quiet, starting it again");
                break;
            }
        }
    }
}

/// Keeps the latest distance from the TF03.
void lidar_task(void* /*arg*/)
{
    static tf03::Tf03 lidar{kLidarUart};
    ESP_ERROR_CHECK(lidar.begin(kLidarTx, kLidarRx, kLidarBaudRate));

    // The TF03 streams measurements unasked, so distances need only its TxD
    // wire. Answering a command also needs its RxD wire, so the firmware
    // version is asked for once, for the log, and its answer waited for no
    // further.
    if (auto version = lidar.firmware_version()) {
        ESP_LOGI(kTag, "TF03 firmware %u.%u.%u", version->major, version->minor, version->patch);
    } else {
        ESP_LOGW(kTag, "TF03 did not answer the firmware version request: %s. Reading distances anyway.",
                 tf03::to_string(version.error()));
    }

    for (;;) {
        if (auto measurement = lidar.read(kLidarSilenceMs)) {
            g_lidar.set(*measurement);
        } else {
            ESP_LOGW(kTag, "TF03 read failed: %s", tf03::to_string(measurement.error()));
        }
    }
}

bool button_down() { return gpio_get_level(kButton) == 0; }

/// Waits for the button to be let go and to stay up past any contact bounce.
void wait_for_release()
{
    for (;;) {
        while (button_down()) {
            vTaskDelay(pdMS_TO_TICKS(kButtonPollMs));
        }
        vTaskDelay(pdMS_TO_TICKS(kDebounceMs));
        if (!button_down()) {
            return;
        }
    }
}

} // namespace

extern "C" void app_main()
{
    // Trigger button, wired from its GPIO to ground.
    gpio_config_t button{};
    button.pin_bit_mask = 1ULL << kButton;
    button.mode = GPIO_MODE_INPUT;
    button.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&button));

    xTaskCreate(gps_task, "gps", kSensorStack, nullptr, 5, nullptr);
    xTaskCreate(imu_task, "imu", kSensorStack, nullptr, 5, nullptr);
    xTaskCreate(lidar_task, "lidar", kSensorStack, nullptr, 5, nullptr);

    std::uint32_t presses = 0;
    for (;;) {
        while (!button_down()) {
            vTaskDelay(pdMS_TO_TICKS(kButtonPollMs));
        }
        // Take the readings from when the button went down, before pressing
        // it has had time to jog where the device points.
        const auto sighting = take_sighting();
        vTaskDelay(pdMS_TO_TICKS(kDebounceMs));
        if (!button_down()) {
            // Too short to be a press: noise, or contacts bouncing.
            continue;
        }

        ++presses;
        if (sighting) {
            std::printf("target #%lu: %s\n", static_cast<unsigned long>(presses), sighting->describe().c_str());
        } else {
            std::printf("target #%lu: none, %s\n", static_cast<unsigned long>(presses), sighting.error().c_str());
        }

        // One coordinate per press, however long the button is held.
        wait_for_release();
    }
}
