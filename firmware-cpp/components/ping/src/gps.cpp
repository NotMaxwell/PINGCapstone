#include "ping/gps.hpp"

namespace ping::gps {
namespace {

/// Room the ESP-IDF driver keeps for received bytes between reads; SparkFun
/// asks for at least 1 KiB.
constexpr int kRxRingBuffer = 2048;

/// How long the receiver may take to answer a poll or acknowledge a
/// configuration. u-blox answers within a navigation epoch; one second covers
/// the slowest rate this configures.
constexpr std::uint16_t kResponseTimeoutMs = 1'000;

} // namespace

const char* to_string(FixType fix)
{
    switch (fix) {
    case FixType::kNoFix:
        return "no fix";
    case FixType::kDeadReckoningOnly:
        return "dead reckoning";
    case FixType::kFix2D:
        return "2D";
    case FixType::kFix3D:
        return "3D";
    case FixType::kGnssPlusDeadReckoning:
        return "GNSS+DR";
    case FixType::kTimeOnly:
        return "time only";
    }
    return "?";
}

esp_err_t Gps::begin(gpio_num_t tx, gpio_num_t rx, int baud)
{
    uart_config_t config{};
    config.baud_rate = baud;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;
    if (esp_err_t err = uart_driver_install(port_, kRxRingBuffer, 0, 0, nullptr, 0); err != ESP_OK) {
        return err;
    }
    if (esp_err_t err = uart_param_config(port_, &config); err != ESP_OK) {
        return err;
    }
    return uart_set_pin(port_, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

std::optional<Version> Gps::connect()
{
    if (!started_) {
        started_ = gnss_.begin(port_, kResponseTimeoutMs);
        if (!started_) {
            return std::nullopt;
        }
    }
    if (!gnss_.getModuleInfo(kResponseTimeoutMs)) {
        return std::nullopt;
    }
    return Version{
        .module = gnss_.getModuleName(),
        .firmware_type = gnss_.getFirmwareType(),
        .firmware_major = gnss_.getFirmwareVersionHigh(),
        .firmware_minor = gnss_.getFirmwareVersionLow(),
        .protocol_major = gnss_.getProtocolVersionHigh(),
        .protocol_minor = gnss_.getProtocolVersionLow(),
    };
}

bool Gps::configure(std::uint8_t rate_hz)
{
    if (rate_hz < 1 || rate_hz > kMaxNavRateHz) {
        return false;
    }
    if (!started_ && !connect()) {
        return false;
    }

    // One CFG-VALSET to the RAM layer: UBX in and out on UART1, NMEA off, and
    // a measurement every 1000 / rate_hz ms with a solution from each.
    gnss_.newCfgValset(VAL_LAYER_RAM);
    gnss_.addCfgValset(UBLOX_CFG_UART1INPROT_UBX, std::uint8_t{1});
    gnss_.addCfgValset(UBLOX_CFG_UART1OUTPROT_UBX, std::uint8_t{1});
    gnss_.addCfgValset(UBLOX_CFG_UART1OUTPROT_NMEA, std::uint8_t{0});
    gnss_.addCfgValset(UBLOX_CFG_RATE_MEAS, static_cast<std::uint16_t>(1000 / rate_hz));
    gnss_.addCfgValset(UBLOX_CFG_RATE_NAV, std::uint16_t{1});
    if (!gnss_.sendCfgValset(kResponseTimeoutMs)) {
        return false;
    }

    // One NAV-PVT per solution, pushed rather than polled, and not consumed
    // implicitly by the getters: poll() reports each one once.
    return gnss_.setAutoUBX("NAV", "PVT", true, false, VAL_LAYER_RAM, kResponseTimeoutMs);
}

std::optional<Solution> Gps::poll()
{
    // With NAV-PVT pushed, this only reads what has already arrived on the
    // UART and reports whether it completed a fresh solution.
    if (!gnss_.getNAVPVT(0)) {
        return std::nullopt;
    }

    std::optional<UtcTime> utc;
    if (gnss_.getDateValid() && gnss_.getTimeValid() && gnss_.getTimeFullyResolved()) {
        utc = UtcTime{
            .year = gnss_.getYear(),
            .month = gnss_.getMonth(),
            .day = gnss_.getDay(),
            .hour = gnss_.getHour(),
            .minute = gnss_.getMinute(),
            .second = gnss_.getSecond(),
            .nanosecond = gnss_.getNanosecond(),
        };
    }

    const std::uint8_t fix = gnss_.getFixType();
    return Solution{
        .itow_ms = gnss_.getTimeOfWeek(),
        // Values 6-255 are reserved; no solution worth using carries one.
        .fix_type = fix <= 5 ? static_cast<FixType>(fix) : FixType::kNoFix,
        .fix_ok = gnss_.getGnssFixOk(),
        .differential = gnss_.getDiffSoln(),
        .satellites = gnss_.getSIV(),
        .latitude_deg = gnss_.getLatitude() * 1e-7,
        .longitude_deg = gnss_.getLongitude() * 1e-7,
        .height_ellipsoid_m = static_cast<float>(gnss_.getAltitude()) * 1e-3F,
        .height_msl_m = static_cast<float>(gnss_.getAltitudeMSL()) * 1e-3F,
        .horizontal_accuracy_m = static_cast<float>(gnss_.getHorizontalAccEst()) * 1e-3F,
        .vertical_accuracy_m = static_cast<float>(gnss_.getVerticalAccEst()) * 1e-3F,
        .velocity_ned_mps =
            {
                static_cast<float>(gnss_.getNedNorthVel()) * 1e-3F,
                static_cast<float>(gnss_.getNedEastVel()) * 1e-3F,
                static_cast<float>(gnss_.getNedDownVel()) * 1e-3F,
            },
        .ground_speed_mps = static_cast<float>(gnss_.getGroundSpeed()) * 1e-3F,
        .speed_accuracy_mps = static_cast<float>(gnss_.getSpeedAccEst()) * 1e-3F,
        .heading_of_motion_deg = static_cast<float>(gnss_.getHeading()) * 1e-5F,
        .heading_accuracy_deg = static_cast<float>(gnss_.getHeadingAccEst()) * 1e-5F,
        .pdop = static_cast<float>(gnss_.getPDOP()) * 1e-2F,
        .utc = utc,
        .invalid_position = gnss_.getInvalidLlh(),
    };
}

} // namespace ping::gps
