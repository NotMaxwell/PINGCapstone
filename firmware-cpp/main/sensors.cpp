#include "sensors.hpp"

#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <TFMPlus.h>
#include <Wire.h>

#include "config.hpp"
#include "esp_log.h"
#include "rtos.hpp"

namespace ping {

namespace readings {

Latest<GpsFix> gps{"GPS"};
Latest<Orientation> imu{"IMU"};
Latest<Range> lidar{"LiDAR"};

}  // namespace readings

const char* to_string(FixType fix)
{
    switch (fix) {
    case FixType::kNoFix: return "no fix";
    case FixType::kDeadReckoningOnly: return "dead reckoning only";
    case FixType::kFix2D: return "2D";
    case FixType::kFix3D: return "3D";
    case FixType::kGnssPlusDeadReckoning: return "GNSS + dead reckoning";
    case FixType::kTimeOnly: return "time only";
    }
    return "?";
}

std::optional<LatLon> GpsFix::position() const
{
    const bool has_position = fix_type == FixType::kFix2D || fix_type == FixType::kFix3D ||
                              fix_type == FixType::kGnssPlusDeadReckoning;
    if (has_position && fix_ok && !invalid_position) {
        return LatLon{.latitude_deg = latitude_deg, .longitude_deg = longitude_deg};
    }
    return std::nullopt;
}

const char* to_string(Accuracy accuracy)
{
    switch (accuracy) {
    case Accuracy::kUnreliable: return "unreliable";
    case Accuracy::kLow: return "low";
    case Accuracy::kMedium: return "medium";
    case Accuracy::kHigh: return "high";
    }
    return "?";
}

namespace sensors {

namespace {

constexpr const char* kGpsTag = "gps";
constexpr const char* kImuTag = "imu";
constexpr const char* kLidarTag = "lidar";

/// Most reports the IMU task reads before giving other tasks a turn. At
/// 100 Hz the hub has one or two waiting; a hub stuck holding INT low would
/// otherwise keep the task spinning.
constexpr int kMaxImuReadsPerWake = 32;

// Stack sizes, in bytes. Every task formats log lines, and SparkFun's
// library parses UBX on the GPS task's stack.
constexpr std::uint32_t kGpsStack = 8192;
constexpr std::uint32_t kImuStack = 6144;
constexpr std::uint32_t kLidarStack = 6144;

/// The sensor tasks run above app_main's priority of 1, so nothing the main
/// task does holds up a reading.
constexpr UBaseType_t kPriority = 5;

/// How often the GPS and IMU tasks look for new data. The GPS delivers ten
/// solutions a second and the IMU a hundred reports.
constexpr std::chrono::milliseconds kPollInterval{1};

/// Calls `attempt` until it returns true, logging each failure and pausing
/// between tries.
template <typename Attempt>
void retry(const char* tag, const char* what, Attempt attempt)
{
    while (!attempt()) {
        ESP_LOGW(tag, "%s failed, retrying", what);
        rtos::sleep(config::kRetryDelay);
    }
}

// ---------------------------------------------------------------------------
// GPS
// ---------------------------------------------------------------------------

/// When the GPS task last heard a solution, in Arduino millis().
std::uint32_t g_last_solution_ms = 0;

/// SparkFun's library calls this from checkCallbacks(), in the GPS task,
/// with each UBX-NAV-PVT the receiver sends.
void on_nav_pvt(UBX_NAV_PVT_data_t* pvt)
{
    // NAV-PVT scales position to degrees × 1e7 and lengths to millimetres.
    readings::gps.set(GpsFix{
        .fix_type = pvt->fixType <= static_cast<std::uint8_t>(FixType::kTimeOnly)
                        ? static_cast<FixType>(pvt->fixType)
                        // Values 6-255 are reserved; no solution worth using
                        // carries one.
                        : FixType::kNoFix,
        .fix_ok = pvt->flags.bits.gnssFixOK != 0,
        .invalid_position = pvt->flags3.bits.invalidLlh != 0,
        .satellites = pvt->numSV,
        .latitude_deg = pvt->lat * 1e-7,
        .longitude_deg = pvt->lon * 1e-7,
        .height_msl_m = static_cast<float>(pvt->hMSL) * 1e-3F,
        .horizontal_accuracy_m = static_cast<float>(pvt->hAcc) * 1e-3F,
    });
    g_last_solution_ms = millis();
}

/// UBX only on UART1, one NAV-PVT per solution, kGpsRateHz solutions a
/// second. Written to the receiver's RAM only, so nothing is left behind in
/// its flash and a fresh board and a reused one behave the same after a
/// power cycle.
bool configure(SFE_UBLOX_GNSS_SERIAL& gnss)
{
    return gnss.setUART1Output(COM_TYPE_UBX, VAL_LAYER_RAM) &&
           gnss.setNavigationFrequency(config::kGpsRateHz, VAL_LAYER_RAM) &&
           gnss.setAutoPVTcallbackPtr(&on_nav_pvt, VAL_LAYER_RAM);
}

// ---------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------

/// Resets the hub and starts its rotation vector.
bool start(Adafruit_BNO08x& imu)
{
    return imu.begin_I2C(config::kImuAddress, &Wire) &&
           imu.enableReport(SH2_ROTATION_VECTOR, config::kImuReportIntervalUs);
}

/// Waits up to `timeout` for the hub to pull INT low, which it does while it
/// has reports waiting.
bool wait_for_interrupt(std::chrono::milliseconds timeout)
{
    const std::uint32_t start = millis();
    while (digitalRead(config::kImuIntPin) != LOW) {
        if (millis() - start >= static_cast<std::uint32_t>(timeout.count())) {
            return false;
        }
        rtos::sleep(kPollInterval);
    }
    return true;
}

void gps_task()
{
    static SFE_UBLOX_GNSS_SERIAL gnss;

    Serial2.setRxBufferSize(1024);
    Serial2.begin(config::kGpsBaud, SERIAL_8N1, config::kGpsRxPin, config::kGpsTxPin);
    retry(kGpsTag, "Looking for the GPS", [] { return gnss.begin(Serial2); });
    if (gnss.getModuleInfo()) {
        ESP_LOGI(kGpsTag, "GPS found: %s, %s firmware %u.%02u, protocol %u.%02u", gnss.getModuleName(),
                 gnss.getFirmwareType(), gnss.getFirmwareVersionHigh(), gnss.getFirmwareVersionLow(),
                 gnss.getProtocolVersionHigh(), gnss.getProtocolVersionLow());
    }

    for (;;) {
        // The configuration lives in the receiver's RAM, so this runs again
        // whenever the receiver goes quiet, in case a power glitch has put it
        // back to its factory NMEA output.
        retry(kGpsTag, "GPS configuration", [] { return configure(gnss); });
        ESP_LOGI(kGpsTag, "GPS up: %u solutions a second", config::kGpsRateHz);
        g_last_solution_ms = millis();

        const auto silence_ms = static_cast<std::uint32_t>(config::kGpsSilence.count());
        while (millis() - g_last_solution_ms < silence_ms) {
            // Parse whatever has arrived, then hand complete NAV-PVTs to
            // on_nav_pvt().
            gnss.checkUblox();
            gnss.checkCallbacks();
            rtos::sleep(kPollInterval);
        }
        ESP_LOGW(kGpsTag, "GPS went quiet, configuring it again");
    }
}

void imu_task()
{
    // No RST line is wired, so the library resets the hub over I2C.
    static Adafruit_BNO08x imu;

    pinMode(config::kImuIntPin, INPUT_PULLUP);
    Wire.begin(config::kImuSdaPin, config::kImuSclPin, config::kImuI2cHz);

    for (;;) {
        retry(kImuTag, "BNO085 bring-up", [] { return start(imu); });
        const sh2_ProductId_t& id = imu.prodIds.entry[0];
        ESP_LOGI(kImuTag, "BNO085 up: firmware %u.%u.%u, reset cause %u", id.swVersionMajor,
                 id.swVersionMinor, id.swVersionPatch, id.resetCause);
        // The reset start() itself caused; only a later one means trouble.
        imu.wasReset();

        for (;;) {
            if (!wait_for_interrupt(config::kImuSilence)) {
                ESP_LOGW(kImuTag, "BNO085 went quiet, starting it again");
                break;
            }

            // Each getSensorEvent() reads one packet. The hub holds INT low
            // until everything waiting has been read.
            for (int reads = 0; reads < kMaxImuReadsPerWake && digitalRead(config::kImuIntPin) == LOW;
                 ++reads) {
                sh2_SensorValue_t value{};
                if (imu.getSensorEvent(&value) && value.sensorId == SH2_ROTATION_VECTOR) {
                    const sh2_RotationVectorWAcc_t& rv = value.un.rotationVector;
                    readings::imu.set(Orientation{
                        .quaternion = {.i = rv.i, .j = rv.j, .k = rv.k, .real = rv.real},
                        .heading_accuracy_rad = rv.accuracy,
                        .calibration = static_cast<Accuracy>(value.status & 0x03U),
                    });
                }
            }

            // A hub that resets on its own stops every report.
            if (imu.wasReset()) {
                ESP_LOGW(kImuTag, "BNO085 reset itself, starting it again");
                break;
            }
        }
    }
}

void lidar_task()
{
    static TFMPlus lidar;

    // About a second of frames at the TF03's 100 Hz.
    Serial1.setRxBufferSize(1024);
    Serial1.begin(config::kLidarBaud, SERIAL_8N1, config::kLidarRxPin, config::kLidarTxPin);
    lidar.begin(&Serial1);

    // The TF03 shares the TFMini-Plus's command set and data frame.
    retry(kLidarTag, "Asking the TF03 for its version",
          [] { return lidar.sendCommand(GET_FIRMWARE_VERSION, 0); });
    ESP_LOGI(kLidarTag, "TF03 up: firmware %u.%u.%u", lidar.version[0], lidar.version[1],
             lidar.version[2]);

    for (;;) {
        // getData() busy-waits for a frame, so only call it once a whole one
        // has arrived, and let other tasks run in the meantime.
        if (Serial1.available() < TFMP_FRAME_SIZE) {
            rtos::sleep(kPollInterval);
            continue;
        }

        std::int16_t distance_cm = 0;
        std::int16_t strength = 0;
        std::int16_t temperature = 0;
        if (!lidar.getData(distance_cm, strength, temperature)) {
            // A checksum mismatch or a lost byte; the next frame resyncs.
            ESP_LOGW(kLidarTag, "TF03 read failed: status %u", lidar.status);
            continue;
        }


        // The TF03 sends kLidarOutOfRangeCm when nothing is in range, and its
        // returns below kLidarMinStrength are not to be trusted.
        const auto distance = static_cast<std::uint16_t>(distance_cm);
        const auto flux = static_cast<std::uint16_t>(strength);
        const bool usable = distance < config::kLidarOutOfRangeCm && flux >= config::kLidarMinStrength;
        readings::lidar.set(Range{
            .distance_cm = usable ? std::optional{distance} : std::nullopt,
            .raw_distance_cm = distance,
            .strength = flux,
        });
    }
}

}  // namespace

void start_gps()
{
    rtos::spawn<gps_task>("gps", kGpsStack, kPriority);
}

void start_imu()
{
    rtos::spawn<imu_task>("imu", kImuStack, kPriority);
}

void start_lidar()
{
    rtos::spawn<lidar_task>("lidar", kLidarStack, kPriority);
}

}  // namespace sensors

}  // namespace ping
