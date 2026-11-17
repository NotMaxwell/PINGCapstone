#include "ping/ubx.hpp"

#include <algorithm>
#include <cstring>

namespace ping::ubx {
namespace {

std::uint16_t read_u16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::uint32_t read_u32(const std::uint8_t* p)
{
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

std::int32_t read_i32(const std::uint8_t* p)
{
    return static_cast<std::int32_t>(read_u32(p));
}

/// Metres, m/s or degrees from a field in milli- or micro-units. Computed in
/// double and narrowed once, as the Rust firmware does, so results match it.
float scaled_f32(double raw, double scale)
{
    return static_cast<float>(raw * scale);
}

/// Copies the NUL terminated string at the start of `src` (at most `n` bytes
/// of it) into `dst` of `cap` bytes, always terminating it.
void copy_cstr(char* dst, std::size_t cap, const std::uint8_t* src, std::size_t n)
{
    std::size_t i = 0;
    while (i < n && i + 1 < cap && src[i] != 0) {
        dst[i] = static_cast<char>(src[i]);
        ++i;
    }
    dst[i] = '\0';
}

/// The text after `prefix` when `s` starts with it.
const char* after_prefix(const char* s, const char* prefix)
{
    const std::size_t n = std::strlen(prefix);
    return std::strncmp(s, prefix, n) == 0 ? s + n : nullptr;
}

std::uint8_t parse_number(const char*& s)
{
    unsigned value = 0;
    while (*s >= '0' && *s <= '9') {
        value = std::min(value * 10 + static_cast<unsigned>(*s - '0'), 255U);
        ++s;
    }
    return static_cast<std::uint8_t>(value);
}

/// Reads `major.minor` from the start of `s`; leaves both zero if it does not
/// look like that.
void parse_major_minor(const char* s, std::uint8_t& major, std::uint8_t& minor)
{
    const char* p = s;
    const std::uint8_t first = parse_number(p);
    if (p == s || *p != '.') {
        return;
    }
    ++p;
    major = first;
    minor = parse_number(p);
}

} // namespace

Checksum checksum(std::span<const std::uint8_t> bytes)
{
    std::uint8_t a = 0;
    std::uint8_t b = 0;
    for (const std::uint8_t byte : bytes) {
        a = static_cast<std::uint8_t>(a + byte);
        b = static_cast<std::uint8_t>(b + a);
    }
    return {a, b};
}

std::optional<std::size_t> encode(std::uint8_t cls, std::uint8_t id, std::span<const std::uint8_t> payload,
                                  std::span<std::uint8_t> out)
{
    if (payload.size() > 0xFFFF || out.size() < payload.size() + kFrameOverhead) {
        return std::nullopt;
    }
    const std::size_t len = payload.size();
    out[0] = kSync1;
    out[1] = kSync2;
    out[2] = cls;
    out[3] = id;
    out[4] = static_cast<std::uint8_t>(len & 0xFF);
    out[5] = static_cast<std::uint8_t>(len >> 8);
    if (len != 0) {
        std::memcpy(out.data() + kHeaderLen, payload.data(), len);
    }
    const Checksum ck = checksum(out.subspan(2, len + 4));
    out[kHeaderLen + len] = ck.a;
    out[kHeaderLen + len + 1] = ck.b;
    return len + kFrameOverhead;
}

std::optional<Frame> Parser::push(std::uint8_t byte)
{
    if (consumed_ != 0) {
        std::memmove(buf_.data(), buf_.data() + consumed_, len_ - consumed_);
        len_ -= consumed_;
        consumed_ = 0;
    }
    if (len_ == buf_.size()) {
        // Only reachable with a backlog of complete frames the caller never
        // drained with poll(); losing the oldest byte resynchronises.
        std::memmove(buf_.data(), buf_.data() + 1, len_ - 1);
        --len_;
    }
    buf_[len_++] = byte;
    return poll();
}

std::optional<Frame> Parser::poll()
{
    if (consumed_ != 0) {
        std::memmove(buf_.data(), buf_.data() + consumed_, len_ - consumed_);
        len_ -= consumed_;
        consumed_ = 0;
    }
    const auto drop = [this](std::size_t n) {
        std::memmove(buf_.data(), buf_.data() + n, len_ - n);
        len_ -= n;
    };

    for (;;) {
        std::size_t skip = 0;
        while (skip < len_ && buf_[skip] != kSync1) {
            ++skip;
        }
        drop(skip);
        if (len_ < 2) {
            return std::nullopt;
        }
        if (buf_[1] != kSync2) {
            drop(1);
            continue;
        }
        if (len_ < kHeaderLen) {
            return std::nullopt;
        }
        const std::size_t payload_len = read_u16(&buf_[4]);
        if (payload_len > kMaxPayload) {
            drop(1);
            continue;
        }
        const std::size_t total = payload_len + kFrameOverhead;
        if (len_ < total) {
            return std::nullopt;
        }
        const Checksum ck = checksum({buf_.data() + 2, payload_len + 4});
        if (ck.a != buf_[total - 2] || ck.b != buf_[total - 1]) {
            // The sync characters were false or the frame is corrupt; what
            // follows the first byte may still hold a real frame.
            drop(1);
            continue;
        }
        consumed_ = total;
        return Frame{buf_[2], buf_[3], {buf_.data() + kHeaderLen, payload_len}};
    }
}

void Parser::reset()
{
    len_ = 0;
    consumed_ = 0;
}

ValsetBuilder::ValsetBuilder(std::uint8_t layers)
{
    // Version 0, the layers, two reserved bytes.
    payload_[0] = 0;
    payload_[1] = layers;
}

void ValsetBuilder::add_key(std::uint32_t key)
{
    for (std::size_t i = 0; i < 4; ++i) {
        payload_[len_ + i] = static_cast<std::uint8_t>(key >> (8 * i));
    }
    len_ += 4;
}

ValsetBuilder& ValsetBuilder::add_bool(std::uint32_t key, bool value)
{
    return add_u8(key, value ? 1 : 0);
}

ValsetBuilder& ValsetBuilder::add_u8(std::uint32_t key, std::uint8_t value)
{
    if (len_ + 5 > kCapacity) {
        overflow_ = true;
        return *this;
    }
    add_key(key);
    payload_[len_++] = value;
    return *this;
}

ValsetBuilder& ValsetBuilder::add_u16(std::uint32_t key, std::uint16_t value)
{
    if (len_ + 6 > kCapacity) {
        overflow_ = true;
        return *this;
    }
    add_key(key);
    payload_[len_++] = static_cast<std::uint8_t>(value & 0xFF);
    payload_[len_++] = static_cast<std::uint8_t>(value >> 8);
    return *this;
}

std::span<const std::uint8_t> ValsetBuilder::finish()
{
    if (overflow_) {
        return {};
    }
    const auto n = encode(kClassCfg, kIdCfgValset, {payload_.data(), len_}, frame_);
    if (!n) {
        return {};
    }
    frame_len_ = *n;
    return {frame_.data(), frame_len_};
}

std::optional<ConfigureFrame> build_configure(std::uint8_t rate_hz)
{
    if (rate_hz < 1 || rate_hz > kMaxNavRateHz) {
        return std::nullopt;
    }
    ValsetBuilder builder{static_cast<std::uint8_t>(kLayerRam | kLayerBbr)};
    builder.add_bool(key::kUart1InProtUbx, true)
        .add_bool(key::kUart1OutProtUbx, true)
        .add_bool(key::kUart1OutProtNmea, false)
        .add_u8(key::kMsgOutNavPvtUart1, 1)
        .add_u16(key::kRateMeas, static_cast<std::uint16_t>(1000 / rate_hz))
        .add_u16(key::kRateNav, 1);
    const auto frame = builder.finish();
    if (frame.empty()) {
        return std::nullopt;
    }
    ConfigureFrame out;
    std::memcpy(out.bytes.data(), frame.data(), frame.size());
    out.len = frame.size();
    return out;
}

ConfigureFrame build_poll_mon_ver()
{
    ConfigureFrame out;
    // An empty payload always fits.
    out.len = encode(kClassMon, kIdMonVer, {}, out.bytes).value_or(0);
    return out;
}

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

std::optional<Solution> decode_nav_pvt(const Frame& frame)
{
    if (frame.cls != kClassNav || frame.id != kIdNavPvt || frame.payload.size() != kNavPvtLen) {
        return std::nullopt;
    }
    const std::uint8_t* p = frame.payload.data();

    constexpr std::uint8_t kValidDate = 1U << 0;
    constexpr std::uint8_t kValidTime = 1U << 1;
    constexpr std::uint8_t kFullyResolved = 1U << 2;
    constexpr std::uint8_t kUtcResolved = kValidDate | kValidTime | kFullyResolved;
    constexpr std::uint8_t kGnssFixOk = 1U << 0;
    constexpr std::uint8_t kDiffSoln = 1U << 1;
    constexpr std::uint16_t kInvalidLlh = 1U << 0;

    Solution s{};
    s.itow_ms = read_u32(p + 0);
    // Values 6-255 are reserved; no solution worth using carries one.
    s.fix_type = p[20] <= 5 ? static_cast<FixType>(p[20]) : FixType::kNoFix;
    s.fix_ok = (p[21] & kGnssFixOk) != 0;
    s.differential = (p[21] & kDiffSoln) != 0;
    s.satellites = p[23];
    s.longitude_deg = static_cast<double>(read_i32(p + 24)) * 1e-7;
    s.latitude_deg = static_cast<double>(read_i32(p + 28)) * 1e-7;
    s.height_ellipsoid_m = scaled_f32(read_i32(p + 32), 1e-3);
    s.height_msl_m = scaled_f32(read_i32(p + 36), 1e-3);
    s.horizontal_accuracy_m = scaled_f32(read_u32(p + 40), 1e-3);
    s.vertical_accuracy_m = scaled_f32(read_u32(p + 44), 1e-3);
    s.velocity_ned_mps[0] = scaled_f32(read_i32(p + 48), 1e-3);
    s.velocity_ned_mps[1] = scaled_f32(read_i32(p + 52), 1e-3);
    s.velocity_ned_mps[2] = scaled_f32(read_i32(p + 56), 1e-3);
    s.ground_speed_mps = scaled_f32(read_i32(p + 60), 1e-3);
    s.heading_of_motion_deg = scaled_f32(read_i32(p + 64), 1e-5);
    s.speed_accuracy_mps = scaled_f32(read_u32(p + 68), 1e-3);
    s.heading_accuracy_deg = scaled_f32(read_u32(p + 72), 1e-5);
    s.pdop = scaled_f32(read_u16(p + 76), 1e-2);
    s.invalid_position = (read_u16(p + 78) & kInvalidLlh) != 0;
    if ((p[11] & kUtcResolved) == kUtcResolved) {
        s.utc = UtcTime{
            .year = read_u16(p + 4),
            .month = p[6],
            .day = p[7],
            .hour = p[8],
            .minute = p[9],
            .second = p[10],
            .nanosecond = read_i32(p + 16),
        };
    }
    return s;
}

std::optional<Version> decode_mon_ver(const Frame& frame)
{
    constexpr std::size_t kFixedLen = 40;
    constexpr std::size_t kExtensionLen = 30;
    const std::size_t len = frame.payload.size();
    if (frame.cls != kClassMon || frame.id != kIdMonVer || len < kFixedLen || (len - kFixedLen) % kExtensionLen != 0) {
        return std::nullopt;
    }
    const std::uint8_t* p = frame.payload.data();

    Version v{};
    copy_cstr(v.software, sizeof v.software, p, 30);
    copy_cstr(v.hardware, sizeof v.hardware, p + 30, 10);
    for (std::size_t off = kFixedLen; off < len; off += kExtensionLen) {
        char text[kExtensionLen + 1];
        copy_cstr(text, sizeof text, p + off, kExtensionLen);
        if (const char* module = after_prefix(text, "MOD=")) {
            std::strncpy(v.module, module, sizeof v.module - 1);
        } else if (const char* protocol = after_prefix(text, "PROTVER=")) {
            std::strncpy(v.protocol, protocol, sizeof v.protocol - 1);
            parse_major_minor(protocol, v.protocol_major, v.protocol_minor);
        } else if (const char* firmware = after_prefix(text, "FWVER=")) {
            const char* space = std::strchr(firmware, ' ');
            const std::size_t type_len = space != nullptr ? static_cast<std::size_t>(space - firmware) : std::strlen(firmware);
            std::memcpy(v.firmware_type, firmware, std::min(type_len, sizeof v.firmware_type - 1));
            if (space != nullptr) {
                parse_major_minor(space + 1, v.firmware_major, v.firmware_minor);
            }
        }
    }
    return v;
}

std::optional<Ack> decode_ack(const Frame& frame)
{
    if (frame.cls != kClassAck || (frame.id != kIdAckAck && frame.id != kIdAckNak) || frame.payload.size() != 2) {
        return std::nullopt;
    }
    return Ack{frame.payload[0], frame.payload[1], frame.id == kIdAckAck};
}

} // namespace ping::ubx
