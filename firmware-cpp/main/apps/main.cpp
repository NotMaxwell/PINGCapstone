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
/// direction (see @ref ping/geo.hpp). Without a fix it walks out from
/// @ref ping::kFallbackPosition instead, if one is set, and says so. The
/// result goes to the USB serial port, one line per press, alongside the logs.
/// The display shows the live heading, compass calibration, range and GPS
/// state, and after each press the target for @ref ping::kTargetHoldMs.
///
/// The bearing comes from @ref ping::mag_cal::Compass (game rotation vector
/// plus the ESP32's own magnetometer correction), measured from true north
/// with @ref ping::kHeadingCorrectionDeg. `docs/heading-calibration.md` is the
/// procedure.
///
/// Every task recovers from its own sensor's failures: a sensor that stops
/// answering is brought up again, and a missing display is retried, so one
/// bad connection never stops the others.

#include "ping/constants.hpp"
#include "ping/display.hpp"
#include "ping/geo.hpp"
#include "ping/gps.hpp"
#include "ping/imu.hpp"
#include "ping/mag_cal.hpp"
#include "ping/rtos.hpp"
#include "ping/tf03.hpp"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>

#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <expected>
#include <numbers>
#include <optional>
#include <utility>
#include <variant>

namespace {

using namespace ping;
using rtos::delay_ms;
using rtos::now_ms;

constexpr const char* kTag = "ping";

/// Stack per task, bytes. The sensor tasks format log lines with printf,
/// which wants a couple of kilobytes on its own.
constexpr std::uint32_t kTaskStack = 6 * 1024;
constexpr UBaseType_t kSensorPriority = 5;
/// Below the sensors: a slow display never delays a reading.
constexpr UBaseType_t kDisplayPriority = 3;

/// How often the main task looks at the button.
constexpr std::uint32_t kButtonPollMs = 5;

constexpr float kDegPerRad = 180.0F / std::numbers::pi_v<float>;

/// What the IMU task keeps from each compass update.
struct Orientation {
    geo::Quaternion quaternion;
    float heading_accuracy_rad;
    geo::Accuracy calibration;

    /// Where the LiDAR points, as a true bearing and an elevation.
    [[nodiscard]] geo::Aim aim() const
    {
        return geo::Aim::from_orientation(quaternion, kLidarBoresight, kHeadingCorrectionDeg);
    }
};

// ---------------------------------------------------------------------------
// Why a press found nothing
// ---------------------------------------------------------------------------

struct Stale {
    const char* sensor;
    rtos::Age age;
};
struct NoFix {
    gps::FixType fix_type;
    std::uint8_t satellites;
};
struct NoReturn {
    std::uint16_t strength;
};
using Problem = std::variant<Stale, NoFix, NoReturn>;

/// A sentence for the serial line.
Text describe(const Problem& problem)
{
    if (const auto* p = std::get_if<Stale>(&problem)) {
        if (!p->age.ms) {
            return Text("no %s reading yet", p->sensor);
        }
        return Text("last %s reading is %" PRId64 " ms old", p->sensor, *p->age.ms);
    }
    if (const auto* p = std::get_if<NoFix>(&problem)) {
        return Text("GPS has no usable fix (%s, %u sats)", gps::to_string(p->fix_type), p->satellites);
    }
    return Text("LiDAR has no trusted return (strength %u)", std::get<NoReturn>(problem).strength);
}

/// The same in two lines short enough for the display.
std::pair<Text, Text> explain(const Problem& problem)
{
    if (const auto* p = std::get_if<Stale>(&problem)) {
        if (!p->age.ms) {
            return {Text("no %s", p->sensor), Text("reading yet")};
        }
        return {Text("%s silent", p->sensor), Text("for %.1f s", static_cast<double>(*p->age.ms) / 1000.0)};
    }
    if (const auto* p = std::get_if<NoFix>(&problem)) {
        return {Text("GPS no fix"), Text("%u satellites", p->satellites)};
    }
    return {Text("LiDAR: no return"), Text("strength %u", std::get<NoReturn>(problem).strength)};
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

rtos::Latest<gps::Solution> g_gps{"GPS"};
rtos::Latest<Orientation> g_imu{"IMU"};
rtos::Latest<tf03::Measurement> g_lidar{"LiDAR"};

/// A located target, and what it was located from.
struct Sighting {
    geo::Position target;
    geo::Position origin;
    geo::Aim aim;
    float range_m;
    /// Set when the origin is @ref ping::kFallbackPosition, saying why the GPS
    /// position was not used.
    std::optional<Problem> fallback_because;
    /// The GPS's 1σ estimate of its horizontal error, metres.
    float origin_accuracy_m;
    std::uint8_t satellites;
    float heading_accuracy_deg;
    geo::Accuracy compass;
};

struct Press {
    std::uint32_t number;
    std::optional<Sighting> sighting;
    /// Why there is no sighting, when there is none.
    std::optional<Problem> problem;
};
rtos::Latest<Press> g_target{"target"};

template <typename T>
std::optional<T> fresh(rtos::Latest<T>& slot, std::uint32_t max_age_ms, std::optional<Problem>& problem)
{
    rtos::Age age;
    auto reading = slot.get(max_age_ms, &age);
    if (!reading && !problem) {
        problem = Stale{.sensor = slot.sensor(), .age = age};
    }
    return reading;
}

/// Locates what the LiDAR is pointing at, from the latest sensor readings.
Press take_sighting(std::uint32_t number)
{
    Press press{.number = number, .sighting = std::nullopt, .problem = std::nullopt};

    const auto orientation = fresh(g_imu, kMaxImuAgeMs, press.problem);
    const auto measurement = fresh(g_lidar, kMaxLidarAgeMs, press.problem);
    if (!orientation || !measurement) {
        return press;
    }
    if (!measurement->distance_cm) {
        press.problem = NoReturn{.strength = measurement->strength};
        return press;
    }

    std::optional<Problem> gps_problem;
    std::optional<geo::Position> origin;
    float accuracy_m = 0.0F;
    std::uint8_t satellites = 0;
    if (const auto solution = fresh(g_gps, kMaxGpsAgeMs, gps_problem)) {
        if (solution->has_position()) {
            origin = geo::Position{
                .latitude_deg = solution->latitude_deg,
                .longitude_deg = solution->longitude_deg,
                .altitude_m = static_cast<double>(solution->height_msl_m),
            };
            accuracy_m = solution->horizontal_accuracy_m;
            satellites = solution->satellites;
        } else {
            gps_problem = NoFix{.fix_type = solution->fix_type, .satellites = solution->satellites};
        }
    }
    std::optional<Problem> fallback_because;
    if (!origin) {
        if (!kFallbackPosition) {
            press.problem = gps_problem;
            return press;
        }
        origin = kFallbackPosition;
        fallback_because = gps_problem;
    }

    const geo::Aim aim = orientation->aim();
    const float range_m = static_cast<float>(*measurement->distance_cm) / 100.0F;
    press.sighting = Sighting{
        .target = origin->project(aim, range_m),
        .origin = *origin,
        .aim = aim,
        .range_m = range_m,
        .fallback_because = fallback_because,
        .origin_accuracy_m = accuracy_m,
        .satellites = satellites,
        .heading_accuracy_deg = orientation->heading_accuracy_rad * kDegPerRad,
        .compass = orientation->calibration,
    };
    return press;
}

void print_press(const Press& press)
{
    if (!press.sighting) {
        std::printf("target #%" PRIu32 ": none, %s\n", press.number, describe(*press.problem).c_str());
        std::fflush(stdout);
        return;
    }
    const Sighting& s = *press.sighting;
    const Text source = s.fallback_because ? Text("FALLBACK POSITION, %s", describe(*s.fallback_because).c_str())
                                           : Text("+/-%.1f m, %u satellites", static_cast<double>(s.origin_accuracy_m),
                                                  s.satellites);
    std::printf("target #%" PRIu32 ": %.7f, %.7f, alt %.1f m"
                " | range %.2f m, bearing %.1f deg, elevation %.1f deg"
                " | from %.7f, %.7f, alt %.1f m, %s"
                " | heading +/-%.1f deg, compass calibration %s\n",
                press.number, s.target.latitude_deg, s.target.longitude_deg, s.target.altitude_m,
                static_cast<double>(s.range_m), static_cast<double>(s.aim.azimuth_deg),
                static_cast<double>(s.aim.elevation_deg), s.origin.latitude_deg, s.origin.longitude_deg,
                s.origin.altitude_m, source.c_str(), static_cast<double>(s.heading_accuracy_deg),
                geo::to_string(s.compass));
    std::fflush(stdout);
}

// ---------------------------------------------------------------------------
// Sensor tasks
// ---------------------------------------------------------------------------

void log_gps(const gps::Solution& solution)
{
    if (solution.has_position()) {
        ESP_LOGI(kTag, "GPS: %s, %u satellites, %.7f, %.7f +/-%.1f m", gps::to_string(solution.fix_type),
                 solution.satellites, solution.latitude_deg, solution.longitude_deg,
                 static_cast<double>(solution.horizontal_accuracy_m));
    } else {
        ESP_LOGI(kTag, "GPS: no fix yet (%s)", gps::to_string(solution.fix_type));
    }
}

/// Keeps the latest navigation solution from the NEO-M9N.
void gps_task(void* /*arg*/)
{
    const rtos::Watchdog watchdog;
    static gps::Gps gps{kGpsUart};
    while (esp_err_t err = gps.begin(kGpsTx, kGpsRx, kGpsBaudRate)) {
        ESP_LOGE(kTag, "GPS UART setup failed: %s, retrying", esp_err_to_name(err));
        watchdog.feed();
        delay_ms(kRetryDelayMs);
    }

    std::int64_t next_log = 0;
    for (;;) {
        // The configuration survives a restart only while the receiver's
        // backup supply holds, so this runs again whenever it goes quiet.
        for (;;) {
            watchdog.feed();
            const auto version = gps.configure(kGpsRateHz);
            if (version) {
                break;
            }
            ESP_LOGW(kTag, "GPS configuration failed: %s, retrying", gps::to_string(version.error()));
            delay_ms(kRetryDelayMs);
        }
        ESP_LOGI(kTag, "GPS up: %u solutions a second", kGpsRateHz);

        std::int64_t last = now_ms();
        for (;;) {
            watchdog.feed();
            const auto solution = gps.read(kGpsSilenceMs);
            if (solution) {
                g_gps.set(*solution);
                last = now_ms();
                if (now_ms() >= next_log) {
                    next_log = now_ms() + kGpsLogIntervalMs;
                    log_gps(*solution);
                }
                continue;
            }
            if (now_ms() - last >= kGpsSilenceMs) {
                ESP_LOGW(kTag, "GPS went quiet (%s), configuring it again", gps::to_string(solution.error()));
                break;
            }
            // Typically an RX overflow; the parser resynchronises on the next
            // frame, so carry on reading.
            ESP_LOGW(kTag, "GPS read failed: %s", gps::to_string(solution.error()));
        }
    }
}

void log_compass(const Orientation& orientation)
{
    const geo::Aim aim = orientation.aim();
    ESP_LOGI(kTag, "Compass: bearing %.1f deg, elevation %.1f deg, heading +/-%.1f deg, calibration %s%s",
             static_cast<double>(aim.azimuth_deg), static_cast<double>(aim.elevation_deg),
             static_cast<double>(orientation.heading_accuracy_rad * kDegPerRad), geo::to_string(orientation.calibration),
             orientation.calibration == geo::Accuracy::kHigh
                 ? ""
                 : " (keep turning and tilting the device to calibrate; if it persists, the field here is disturbed)");
}

/// Says what the IMU's wire did, so a dropout can be pinned on the hub (no
/// INT edges), the bus (failed transfers, bad headers) or a hub reset.
void log_imu_link(const imu::Imu& imu)
{
    const imu::LinkStats s = imu.link_stats();
    const auto now = static_cast<std::uint32_t>(esp_timer_get_time());
    ESP_LOGW(kTag,
             "BNO085 link: %" PRIu32 " INT edges (last %" PRIu32 " ms ago), %" PRIu32 " transfers ok, %" PRIu32
             " failed (last %s), %" PRIu32 " cargos, %" PRIu32 " empty / %" PRIu32 " invalid headers, %" PRIu32
             " oversized, %" PRIu32 " wake timeouts, %" PRIu32 " hub resets, %" PRIu32 " SHTP errors, %" PRIu32
             " events (last %" PRIu32 " ms ago), %" PRIu32 " dropped",
             s.int_edges, (now - s.last_int_us) / 1000, s.transfers_ok, s.transfers_failed,
             esp_err_to_name(s.last_spi_error), s.cargos_read, s.empty_headers, s.invalid_headers, s.oversized_cargos,
             s.wake_timeouts, s.hub_resets, s.shtp_errors, s.sensor_events, (now - s.last_event_us) / 1000,
             s.dropped_events);
}

/// Resets the hub and starts the reports the compass is built from.
///
/// Each step can wait seconds for the hub (CEVA's HAL allows 2 s for it to
/// boot, and each request up to 1 s), so the watchdog is fed between steps.
std::expected<imu::ProductId, imu::Error> start_imu(imu::Imu& imu, const rtos::Watchdog& watchdog)
{
    auto id = imu.start();
    watchdog.feed();
    if (!id) {
        return id;
    }
    // `imu_cal` calibrates the magnetometer; the hub keeps calibrating only
    // what the game rotation vector relies on.
    if (auto r = imu.configure_calibration(true, false, false); !r) {
        return std::unexpected(r.error());
    }
    watchdog.feed();
    if (auto r = imu.enable_report(imu::SensorId::kGameRotationVector, kImuReportIntervalUs); !r) {
        return std::unexpected(r.error());
    }
    watchdog.feed();
    if (auto r = imu.enable_report(imu::SensorId::kMagneticFieldUncalibrated, kMagReportIntervalUs); !r) {
        return std::unexpected(r.error());
    }
    return id;
}

/// Keeps the latest orientation from the BNO085.
void imu_task(void* /*arg*/)
{
    const rtos::Watchdog watchdog;
    static imu::Imu imu;

    for (;;) {
        watchdog.feed();
        const auto id = start_imu(imu, watchdog);
        if (!id) {
            ESP_LOGW(kTag, "BNO085 bring-up failed: %s, retrying", imu::to_string(id.error()));
            log_imu_link(imu);
            delay_ms(kRetryDelayMs);
            continue;
        }
        ESP_LOGI(kTag, "BNO085 up: firmware %u.%u.%u, reset cause %u", id->sw_major, id->sw_minor, id->sw_patch,
                 id->reset_cause);

        // The game rotation vector's frame starts over with the hub.
        mag_cal::Compass compass{kMagCalibration};
        mag_cal::StartupCalibration startup{kMagCalibration ? std::optional(kMagCalibration->matrix) : std::nullopt};
        std::int64_t next_log = 0;
        std::int64_t last = now_ms();

        for (;;) {
            watchdog.feed();
            const auto event = imu.next(kImuSilenceMs);
            if (!event) {
                ESP_LOGW(kTag, "BNO085 failed: %s, starting it again", imu::to_string(event.error()));
                log_imu_link(imu);
                break;
            }
            if (!*event) {
                if (now_ms() - last >= kImuSilenceMs) {
                    ESP_LOGW(kTag, "BNO085 went quiet, starting it again");
                    log_imu_link(imu);
                    break;
                }
                continue;
            }
            last = now_ms();
            const imu::Event& e = **event;
            if (e.sensor == imu::SensorId::kMagneticFieldUncalibrated) {
                compass.update_field(e.vector);
                if (const auto found = startup.add(e.vector)) {
                    ESP_LOGI(kTag,
                             "Compass calibrated from handling: hard iron %.1f %.1f %.1f uT, field %.1f uT, "
                             "residual %.4f",
                             static_cast<double>(found->offset[0]), static_cast<double>(found->offset[1]),
                             static_cast<double>(found->offset[2]), static_cast<double>(found->field_ut),
                             static_cast<double>(found->residual));
                    compass.set_calibration(*found);
                }
            } else if (e.sensor == imu::SensorId::kGameRotationVector) {
                compass.update_game(e.quaternion);
                const auto heading = compass.heading();
                if (!heading) {
                    continue;
                }
                const Orientation orientation{
                    .quaternion = heading->quaternion,
                    .heading_accuracy_rad = heading->accuracy_rad,
                    .calibration = heading->status,
                };
                g_imu.set(orientation);
                if (now_ms() >= next_log) {
                    next_log = now_ms() + kCompassLogIntervalMs;
                    log_compass(orientation);
                }
            }
        }
    }
}

/// Keeps the latest distance from the TF03.
void lidar_task(void* /*arg*/)
{
    const rtos::Watchdog watchdog;
    static tf03::Tf03 lidar{kLidarUart};
    while (esp_err_t err = lidar.begin(kLidarTx, kLidarRx, kLidarBaudRate)) {
        ESP_LOGE(kTag, "LiDAR UART setup failed: %s, retrying", esp_err_to_name(err));
        watchdog.feed();
        delay_ms(kRetryDelayMs);
    }

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

    std::uint32_t silent_periods = 0;
    for (;;) {
        watchdog.feed();
        auto measurement = lidar.read(kLidarSilenceMs);
        if (measurement) {
            if (silent_periods > 0) {
                ESP_LOGI(kTag, "TF03 back after %" PRIu32 " silent periods", silent_periods);
                silent_periods = 0;
            }
            g_lidar.set(*measurement);
            continue;
        }
        ESP_LOGW(kTag, "TF03 read failed: %s", tf03::to_string(measurement.error()));
        if (measurement.error().kind != tf03::Error::Kind::kTimeout) {
            continue;
        }

        // Say exactly what the wire did, so a dropout can be pinned on the
        // sensor (no bytes), the line (frame errors) or the parser (bytes
        // but no packets).
        ++silent_periods;
        const tf03::LinkStats& s = lidar.stats();
        const std::int64_t now = esp_timer_get_time();
        ESP_LOGW(kTag,
                 "TF03 link: %" PRIu32 " bytes, %" PRIu32 " packets, %" PRIu32 " bad checksums, %" PRIu32
                 " skipped, %" PRIu32 " frame errs, %" PRIu32 " FIFO ovf, %" PRIu32 " buf full, last byte %" PRId64
                 " ms ago, last frame %" PRId64 " ms ago",
                 s.parser.bytes, s.parser.packets, s.parser.checksum_errors, s.parser.skipped, s.frame_errors,
                 s.fifo_overflows, s.buffer_full, s.last_byte_us ? (now - s.last_byte_us) / 1000 : -1,
                 s.last_frame_us ? (now - s.last_frame_us) / 1000 : -1);
        // Rule out the ESP32's own UART: after a few silent periods, start
        // its driver from scratch.
        if (silent_periods % 3 == 0) {
            if (esp_err_t err = lidar.restart_uart(); err != ESP_OK) {
                ESP_LOGE(kTag, "TF03 UART restart failed: %s", esp_err_to_name(err));
            } else {
                ESP_LOGW(kTag, "TF03 UART restarted");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

/// The nearest of the sixteen compass points to `bearing_deg`.
const char* compass_point(float bearing_deg)
{
    static constexpr const char* kPoints[16] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                                "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    // Each point covers 22.5°, centred on its own bearing.
    return kPoints[static_cast<unsigned>(bearing_deg / 22.5F + 0.5F) % 16];
}

/// Shows the latest press for @ref ping::kTargetHoldMs after it, otherwise
/// the live readings.
void refresh(Display& lcd)
{
    if (const auto press = g_target.get(kTargetHoldMs)) {
        if (press->sighting) {
            const Sighting& s = *press->sighting;
            lcd.show({s.fallback_because ? Text("#%" PRIu32 " test origin", press->number)
                                         : Text("#%" PRIu32 " target", press->number),
                      Text("%c %.6f", s.target.latitude_deg < 0 ? 'S' : 'N', std::abs(s.target.latitude_deg)),
                      Text("%c %.6f", s.target.longitude_deg < 0 ? 'W' : 'E', std::abs(s.target.longitude_deg)),
                      Text("%.1fm @ %.0f°", static_cast<double>(s.range_m), static_cast<double>(s.aim.azimuth_deg))},
                     false);
        } else {
            const auto [why, detail] = explain(*press->problem);
            lcd.show({Text("#%" PRIu32 " no target", press->number), why, detail}, false);
        }
        return;
    }

    std::optional<Problem> imu_problem;
    std::pair<Text, Text> heading{Text(""), Text("")};
    if (const auto orientation = fresh(g_imu, kMaxImuAgeMs, imu_problem)) {
        const geo::Aim aim = orientation->aim();
        heading.first = Text("hdg %.1f° %s", static_cast<double>(aim.azimuth_deg), compass_point(aim.azimuth_deg));
        switch (orientation->calibration) {
        case geo::Accuracy::kHigh:
            heading.second =
                Text("cal high ±%.0f°", static_cast<double>(orientation->heading_accuracy_rad * kDegPerRad));
            break;
        case geo::Accuracy::kMedium:
            heading.second = Text("cal med");
            break;
        case geo::Accuracy::kLow:
            heading.second = Text("cal low");
            break;
        case geo::Accuracy::kUnreliable:
            heading.second = Text("cal bad - move");
            break;
        }
    } else {
        heading = explain(*imu_problem);
    }

    std::optional<Problem> ignored;
    Text range("LiDAR silent");
    if (const auto m = fresh(g_lidar, kMaxLidarAgeMs, ignored)) {
        range = m->distance_cm ? Text("rng %.2f m", static_cast<double>(*m->distance_cm) / 100.0)
                               : Text("rng: no return");
    }
    Text gps("GPS silent");
    if (const auto s = fresh(g_gps, kMaxGpsAgeMs, ignored)) {
        gps = Text("GPS %s %usat", gps::to_string(s->fix_type), s->satellites);
    }
    lcd.show({heading.first, heading.second, range, gps}, false);
}

/// Keeps the display up to date. @ref Display::show brings a missing or
/// unplugged display back by itself, so this only paces it.
void display_task(void* /*arg*/)
{
    static Display lcd;
    while (esp_err_t err = lcd.begin()) {
        ESP_LOGE(kTag, "display I2C setup failed: %s, retrying", esp_err_to_name(err));
        delay_ms(kRetryDelayMs);
    }
    for (;;) {
        refresh(lcd);
        // A missing display is retried less often.
        delay_ms(lcd.ready() ? kLcdRefreshMs : kRetryDelayMs);
    }
}

// ---------------------------------------------------------------------------
// Button
// ---------------------------------------------------------------------------

bool button_down() { return gpio_get_level(kButton) == 0; }

/// Waits for the button to be let go and to stay up past any contact bounce.
void wait_for_release()
{
    for (;;) {
        while (button_down()) {
            delay_ms(kButtonPollMs);
        }
        delay_ms(kDebounceMs);
        if (!button_down()) {
            return;
        }
    }
}

} // namespace

extern "C" void app_main()
{
    // Polled rather than edge-triggered: no ISR to share with the IMU's INT
    // handler, and 5 ms is far inside a press.
    gpio_config_t button{};
    button.pin_bit_mask = 1ULL << kButton;
    button.mode = GPIO_MODE_INPUT;
    button.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&button));

    // Without its tasks the device cannot work at all, so a failure here
    // (out of memory at boot) restarts the chip rather than limping on.
    if (!rtos::spawn(gps_task, "gps", kTaskStack, kSensorPriority) ||
        !rtos::spawn(imu_task, "imu", kTaskStack, kSensorPriority) ||
        !rtos::spawn(lidar_task, "lidar", kTaskStack, kSensorPriority) ||
        !rtos::spawn(display_task, "display", kTaskStack, kDisplayPriority)) {
        esp_restart();
    }

    std::uint32_t presses = 0;
    for (;;) {
        while (!button_down()) {
            delay_ms(kButtonPollMs);
        }
        // Take the readings from when the button went down, before pressing
        // it has had time to jog where the device points.
        const Press press = take_sighting(presses + 1);
        delay_ms(kDebounceMs);
        if (!button_down()) {
            // Too short to be a press: noise, or contacts bouncing.
            continue;
        }

        ++presses;
        g_target.set(press);
        print_press(press);

        // One coordinate per press, however long the button is held.
        wait_for_release();
    }
}
