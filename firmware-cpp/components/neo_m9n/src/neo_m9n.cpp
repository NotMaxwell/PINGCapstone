#include "ping/neo_m9n.hpp"

#include <array>
#include <cstdlib>

#include "esp_log.h"

// ubxlib's headers expect to be included in a particular order, which its
// umbrella header takes care of. It is C, and says so.
#include "ubxlib.h"

namespace ping::neo_m9n {

namespace {

constexpr const char* kTag = "neo_m9n";

/// UBX-NAV-PVT is 6 bytes of header, 92 of body and 2 of checksum.
constexpr std::size_t kNavPvtFrameLen = 100;

/// UBX message class and ID, as ubxlib's message filter wants them.
constexpr std::uint16_t kNavPvt =
    (U_GNSS_DEC_UBX_NAV_PVT_MESSAGE_CLASS << 8) | U_GNSS_DEC_UBX_NAV_PVT_MESSAGE_ID;

bool g_ubxlib_started = false;

uDeviceHandle_t handle(void* device)
{
    return static_cast<uDeviceHandle_t>(device);
}

constexpr bool bit(std::uint32_t field, unsigned position)
{
    return ((field >> position) & 1U) != 0;
}

Solution from_nav_pvt(const uGnssDecUbxNavPvt_t& pvt)
{
    constexpr std::uint32_t kUtcResolved = (1U << U_GNSS_DEC_UBX_NAV_PVT_VALID_DATE) |
                                           (1U << U_GNSS_DEC_UBX_NAV_PVT_VALID_TIME) |
                                           (1U << U_GNSS_DEC_UBX_NAV_PVT_VALID_FULLY_RESOLVED);
    std::optional<UtcTime> utc;
    if ((pvt.valid & kUtcResolved) == kUtcResolved) {
        utc = UtcTime{
            .year = pvt.year,
            .month = pvt.month,
            .day = pvt.day,
            .hour = pvt.hour,
            .minute = pvt.min,
            .second = pvt.sec,
            .nanosecond = pvt.nano,
        };
    }

    const auto fix = pvt.fixType <= U_GNSS_DEC_UBX_NAV_PVT_FIX_TYPE_TIME_ONLY
                         ? static_cast<FixType>(pvt.fixType)
                         // Values 6-255 are reserved; no solution worth
                         // using carries one.
                         : FixType::kNoFix;

    // NAV-PVT scales everything to integers: degrees by 1e7 for position,
    // 1e5 for headings, millimetres for lengths and speeds, DOP by 100.
    return Solution{
        .itow_ms = pvt.iTOW,
        .fix_type = fix,
        .fix_ok = bit(pvt.flags, U_GNSS_DEC_UBX_NAV_PVT_FLAGS_GNSS_FIX_OK),
        .differential = bit(pvt.flags, U_GNSS_DEC_UBX_NAV_PVT_FLAGS_DIFF_SOLN),
        .satellites = pvt.numSV,
        .latitude_deg = pvt.lat * 1e-7,
        .longitude_deg = pvt.lon * 1e-7,
        .height_ellipsoid_m = static_cast<float>(pvt.height) * 1e-3F,
        .height_msl_m = static_cast<float>(pvt.hMSL) * 1e-3F,
        .horizontal_accuracy_m = static_cast<float>(pvt.hAcc) * 1e-3F,
        .vertical_accuracy_m = static_cast<float>(pvt.vAcc) * 1e-3F,
        .velocity_ned_mps = {static_cast<float>(pvt.velN) * 1e-3F,
                             static_cast<float>(pvt.velE) * 1e-3F,
                             static_cast<float>(pvt.velD) * 1e-3F},
        .ground_speed_mps = static_cast<float>(pvt.gSpeed) * 1e-3F,
        .speed_accuracy_mps = static_cast<float>(pvt.sAcc) * 1e-3F,
        .heading_of_motion_deg = static_cast<float>(pvt.headMot) * 1e-5F,
        .heading_accuracy_deg = static_cast<float>(pvt.headAcc) * 1e-5F,
        .pdop = static_cast<float>(pvt.pDOP) * 1e-2F,
        .utc = utc,
        .invalid_position = bit(pvt.flags3, U_GNSS_DEC_UBX_NAV_PVT_FLAGS3_INVALID_LLH),
    };
}

}  // namespace

const char* Error::message() const
{
    switch (code) {
    case U_ERROR_COMMON_SUCCESS: return "ok";
    case U_ERROR_COMMON_NOT_INITIALISED: return "not initialised";
    case U_ERROR_COMMON_NOT_SUPPORTED: return "not supported";
    case U_ERROR_COMMON_INVALID_PARAMETER: return "invalid parameter";
    case U_ERROR_COMMON_NO_MEMORY: return "out of memory";
    case U_ERROR_COMMON_NOT_RESPONDING: return "receiver not responding";
    case U_ERROR_COMMON_PLATFORM: return "UART error";
    case U_ERROR_COMMON_TIMEOUT: return "timed out";
    case U_ERROR_COMMON_TRUNCATED: return "message truncated";
    case U_ERROR_COMMON_BAD_DATA: return "bad data";
    case U_ERROR_COMMON_UNKNOWN_MODULE_TYPE: return "unknown module type";
    case U_GNSS_ERROR_TRANSPORT: return "transport error";
    case U_GNSS_ERROR_NACK: return "receiver refused the message (UBX-ACK-NAK)";
    case U_GNSS_ERROR_CRC: return "checksum error";
    default: return "error";
    }
}

bool Error::timed_out() const
{
    return code == U_ERROR_COMMON_TIMEOUT;
}

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

std::optional<LatLon> Solution::position() const
{
    const bool has_position = fix_type == FixType::kFix2D || fix_type == FixType::kFix3D ||
                              fix_type == FixType::kGnssPlusDeadReckoning;
    if (has_position && fix_ok && !invalid_position) {
        return LatLon{.latitude_deg = latitude_deg, .longitude_deg = longitude_deg};
    }
    return std::nullopt;
}

NeoM9n::NeoM9n(const Config& config) : config_{config}
{
    if (!g_ubxlib_started) {
        if (uPortInit() != 0 || uDeviceInit() != 0) {
            ESP_LOGE(kTag, "ubxlib failed to start");
            std::abort();
        }
        // ubxlib narrates everything it does on the console, which this
        // firmware keeps for its own diagnostics and results. uPortLogOn()
        // brings it back when debugging.
        uPortLogOff();
        g_ubxlib_started = true;
    }
}

NeoM9n::~NeoM9n()
{
    close();
}

Result<> NeoM9n::open()
{
    close();

    uDeviceCfg_t device_config{};
    device_config.deviceType = U_DEVICE_TYPE_GNSS;
    device_config.deviceCfg.cfgGnss.moduleType = U_GNSS_MODULE_TYPE_M9;
    device_config.deviceCfg.cfgGnss.pinEnablePower = -1;
    device_config.deviceCfg.cfgGnss.pinDataReady = -1;
    device_config.transportType = U_DEVICE_TRANSPORT_TYPE_UART;
    device_config.transportCfg.cfgUart.uart = config_.uart;
    device_config.transportCfg.cfgUart.baudRate = config_.baud_rate;
    device_config.transportCfg.cfgUart.pinTxd = config_.tx_pin;
    device_config.transportCfg.cfgUart.pinRxd = config_.rx_pin;
    device_config.transportCfg.cfgUart.pinCts = -1;
    device_config.transportCfg.cfgUart.pinRts = -1;

    uDeviceHandle_t device = nullptr;
    if (const std::int32_t status = uDeviceOpen(&device_config, &device); status != 0) {
        return std::unexpected(Error{status});
    }
    device_ = device;
    return {};
}

void NeoM9n::close()
{
    if (device_ != nullptr) {
        uDeviceClose(handle(device_), false);
        device_ = nullptr;
    }
}

Result<> NeoM9n::configure(std::uint8_t rate_hz)
{
    if (device_ == nullptr) {
        return std::unexpected(Error{U_ERROR_COMMON_NOT_INITIALISED});
    }
    if (rate_hz < 1 || rate_hz > kMaxNavRateHz) {
        return std::unexpected(Error{U_ERROR_COMMON_INVALID_PARAMETER});
    }

    const std::array<uGnssCfgVal_t, 6> settings{{
        {U_GNSS_CFG_VAL_KEY_ID_UART1INPROT_UBX_L, 1},
        {U_GNSS_CFG_VAL_KEY_ID_UART1OUTPROT_UBX_L, 1},
        {U_GNSS_CFG_VAL_KEY_ID_UART1OUTPROT_NMEA_L, 0},
        // One NAV-PVT per navigation solution.
        {U_GNSS_CFG_VAL_KEY_ID_MSGOUT_UBX_NAV_PVT_UART1_U1, 1},
        // Measure every 1000 / rate_hz ms, and compute a solution from every
        // measurement.
        {U_GNSS_CFG_VAL_KEY_ID_RATE_MEAS_U2, 1000U / rate_hz},
        {U_GNSS_CFG_VAL_KEY_ID_RATE_NAV_U2, 1},
    }};
    const std::int32_t status =
        uGnssCfgValSetList(handle(device_), settings.data(), settings.size(),
                           U_GNSS_CFG_VAL_TRANSACTION_NONE, U_GNSS_CFG_VAL_LAYER_RAM);
    if (status != 0) {
        return std::unexpected(Error{status});
    }
    return {};
}

Result<Solution> NeoM9n::read(std::int32_t timeout_ms)
{
    if (device_ == nullptr) {
        return std::unexpected(Error{U_ERROR_COMMON_NOT_INITIALISED});
    }

    uGnssMessageId_t wanted{};
    wanted.type = U_GNSS_PROTOCOL_UBX;
    wanted.id.ubx = kNavPvt;

    std::array<char, kNavPvtFrameLen> frame{};
    char* buffer = frame.data();
    const std::int32_t received =
        uGnssMsgReceive(handle(device_), &wanted, &buffer, frame.size(), timeout_ms, nullptr);
    if (received < 0) {
        return std::unexpected(Error{received});
    }

    uGnssDec_t* decoded = pUGnssDecAlloc(buffer, static_cast<std::size_t>(received));
    if (decoded == nullptr) {
        return std::unexpected(Error{U_ERROR_COMMON_NO_MEMORY});
    }
    Result<Solution> result = std::unexpected(Error{decoded->errorCode});
    if (decoded->errorCode == 0 && decoded->pBody != nullptr) {
        result = from_nav_pvt(decoded->pBody->ubxNavPvt);
    }
    uGnssDecFree(decoded);
    return result;
}

}  // namespace ping::neo_m9n
