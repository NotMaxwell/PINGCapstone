/// @file
/// The part of u-blox's UBX protocol the NEO-M9N driver uses: framing, the
/// Fletcher checksum, a resynchronising parser, the `UBX-CFG-VALSET` message
/// that configures the receiver, and decoders for `UBX-NAV-PVT`, `UBX-MON-VER`
/// and `UBX-ACK-ACK`/`ACK-NAK`.
///
/// Ported from the Rust firmware's `neo_m9n.rs`, which leans on the `ublox`
/// crate for the same jobs. Field layouts follow the u-blox M9 interface
/// description (protocol version 32, as on the NEO-M9N). Nothing here touches
/// hardware, so the host tests exercise it directly.
///
/// A UBX frame is `B5 62 CLASS ID LEN_L LEN_H payload… CK_A CK_B`, all
/// multi-byte fields little endian. The checksum runs over everything between
/// the sync characters and itself.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace ping::ubx {

inline constexpr std::uint8_t kSync1 = 0xB5;
inline constexpr std::uint8_t kSync2 = 0x62;
/// Sync characters, class, ID and length.
inline constexpr std::size_t kHeaderLen = 6;
/// Bytes a frame adds around its payload.
inline constexpr std::size_t kFrameOverhead = kHeaderLen + 2;

/// Largest payload the parser accepts. `UBX-MON-VER` is the biggest message
/// the driver reads: 40 bytes plus 30 per extension string. A frame announcing
/// more than this is treated as line noise.
inline constexpr std::size_t kMaxPayload = 512;
inline constexpr std::size_t kMaxFrame = kMaxPayload + kFrameOverhead;

inline constexpr std::uint8_t kClassNav = 0x01;
inline constexpr std::uint8_t kClassAck = 0x05;
inline constexpr std::uint8_t kClassCfg = 0x06;
inline constexpr std::uint8_t kClassMon = 0x0A;

inline constexpr std::uint8_t kIdNavPvt = 0x07;
inline constexpr std::uint8_t kIdAckNak = 0x00;
inline constexpr std::uint8_t kIdAckAck = 0x01;
inline constexpr std::uint8_t kIdCfgValset = 0x8A;
inline constexpr std::uint8_t kIdMonVer = 0x04;

/// Payload length of `UBX-NAV-PVT` on protocol version 32.
inline constexpr std::size_t kNavPvtLen = 92;

/// Configuration keys, from the M9 interface description. Bits 28..30 of a key
/// give the size of its value: 1 is one bit (sent as a byte), 2 one byte,
/// 3 two bytes.
namespace key {
inline constexpr std::uint32_t kUart1InProtUbx = 0x10730001;
inline constexpr std::uint32_t kUart1OutProtUbx = 0x10740001;
inline constexpr std::uint32_t kUart1OutProtNmea = 0x10740002;
inline constexpr std::uint32_t kMsgOutNavPvtUart1 = 0x20910007;
inline constexpr std::uint32_t kRateMeas = 0x30210001;
inline constexpr std::uint32_t kRateNav = 0x30210002;
} // namespace key

/// Layers a `UBX-CFG-VALSET` writes to. The driver uses RAM and BBR and never
/// flash.
inline constexpr std::uint8_t kLayerRam = 0x01;
inline constexpr std::uint8_t kLayerBbr = 0x02;

/// The fastest navigation rate the NEO-M9N supports.
inline constexpr std::uint8_t kMaxNavRateHz = 25;

/// Fletcher checksum as UBX defines it.
struct Checksum {
    std::uint8_t a;
    std::uint8_t b;
    friend bool operator==(const Checksum&, const Checksum&) = default;
};

/// Checksum over `bytes`, which for a frame is class, ID, length and payload.
Checksum checksum(std::span<const std::uint8_t> bytes);

/// Writes a complete frame into `out`. Returns its length, or nothing if the
/// payload exceeds 65535 bytes or `out` is too small.
std::optional<std::size_t> encode(std::uint8_t cls, std::uint8_t id, std::span<const std::uint8_t> payload,
                                  std::span<std::uint8_t> out);

/// A frame the parser has verified. The payload points into the parser and is
/// valid until the next call on it.
struct Frame {
    std::uint8_t cls;
    std::uint8_t id;
    std::span<const std::uint8_t> payload;
};

/// Splits a byte stream into verified frames.
///
/// NMEA text, garbage, frames with a bad checksum and frames announcing more
/// than @ref kMaxPayload are skipped. When a candidate frame fails, the bytes
/// it swallowed are searched again for the sync characters, so a real frame
/// that started inside a corrupted one is not lost.
class Parser {
  public:
    /// Feeds one byte. Returns the frame it completed, if any.
    ///
    /// Bytes are buffered, so a frame can be complete before the byte that
    /// reveals it: after a corrupted frame is abandoned, a good one may already
    /// be sitting in the buffer. Call @ref poll after a frame to drain those.
    [[nodiscard]] std::optional<Frame> push(std::uint8_t byte);

    /// Looks for a complete frame among the bytes already buffered.
    [[nodiscard]] std::optional<Frame> poll();

    /// Drops everything buffered. Call after bytes were lost, so a partial
    /// frame is not spliced onto an unrelated later one.
    void reset();

  private:
    std::array<std::uint8_t, kMaxFrame> buf_{};
    std::size_t len_ = 0;
    /// Length of the frame handed out last, removed on the next call.
    std::size_t consumed_ = 0;
};

/// A fixed-capacity `UBX-CFG-VALSET` under construction.
class ValsetBuilder {
  public:
    /// Largest message this builder holds: enough for the driver's six keys
    /// with room to spare.
    static constexpr std::size_t kCapacity = 64;

    /// Starts a message writing to `layers`, a mix of @ref kLayerRam and
    /// @ref kLayerBbr.
    explicit ValsetBuilder(std::uint8_t layers);

    /// A one-bit (L) value, sent as a byte.
    ValsetBuilder& add_bool(std::uint32_t key, bool value);
    ValsetBuilder& add_u8(std::uint32_t key, std::uint8_t value);
    ValsetBuilder& add_u16(std::uint32_t key, std::uint16_t value);

    /// The framed message, or an empty span if more was added than fits.
    [[nodiscard]] std::span<const std::uint8_t> finish();

  private:
    void add_key(std::uint32_t key);

    std::array<std::uint8_t, kCapacity> payload_{};
    std::size_t len_ = 4;
    bool overflow_ = false;
    std::array<std::uint8_t, kCapacity + kFrameOverhead> frame_{};
    std::size_t frame_len_ = 0;
};

/// The configuration `neo_m9n.rs` sends, as one `UBX-CFG-VALSET` to RAM and
/// BBR: UBX in and out on UART1, NMEA out off, one `UBX-NAV-PVT` per
/// solution, and a measurement every `1000 / rate_hz` ms with a solution from
/// each.
struct ConfigureFrame {
    std::array<std::uint8_t, ValsetBuilder::kCapacity + kFrameOverhead> bytes{};
    std::size_t len = 0;

    [[nodiscard]] std::span<const std::uint8_t> view() const { return {bytes.data(), len}; }
};

/// Builds the configuration frame. Nothing if `rate_hz` is outside
/// 1..@ref kMaxNavRateHz.
std::optional<ConfigureFrame> build_configure(std::uint8_t rate_hz);

/// The `UBX-MON-VER` poll request (empty payload).
ConfigureFrame build_poll_mon_ver();

/// What kind of solution the receiver has.
enum class FixType : std::uint8_t {
    kNoFix = 0,
    /// Position from dead reckoning alone. The NEO-M9N has no sensor fusion,
    /// so this should not appear.
    kDeadReckoningOnly = 1,
    kFix2D = 2,
    kFix3D = 3,
    kGnssPlusDeadReckoning = 4,
    /// Time is known but position is not trusted.
    kTimeOnly = 5,
};

const char* to_string(FixType fix);

/// UTC date and time of a solution.
struct UtcTime {
    std::uint16_t year;
    std::uint8_t month;
    std::uint8_t day;
    std::uint8_t hour;
    std::uint8_t minute;
    std::uint8_t second;
    /// Fraction of the second, which can be negative: the receiver rounds
    /// the other fields to the nearest second and puts the remainder here.
    std::int32_t nanosecond;
};

/// One navigation solution, decoded from `UBX-NAV-PVT`.
struct Solution {
    /// GPS time of week of the navigation epoch, milliseconds.
    std::uint32_t itow_ms;
    FixType fix_type;
    /// The fix is within the receiver's DOP and accuracy masks. Treat the
    /// position as usable only when this is set; @ref has_position does.
    bool fix_ok;
    /// Differential corrections (e.g. SBAS) were applied.
    bool differential;
    /// Satellites used in the solution.
    std::uint8_t satellites;
    /// Degrees, positive north.
    double latitude_deg;
    /// Degrees, positive east.
    double longitude_deg;
    /// Height above the WGS-84 ellipsoid, metres.
    float height_ellipsoid_m;
    /// Height above mean sea level, metres.
    float height_msl_m;
    /// 1σ horizontal position accuracy estimate, metres.
    float horizontal_accuracy_m;
    /// 1σ vertical position accuracy estimate, metres.
    float vertical_accuracy_m;
    /// Velocity north, east and down, m/s.
    float velocity_ned_mps[3];
    /// Horizontal speed, m/s.
    float ground_speed_mps;
    /// 1σ speed accuracy estimate, m/s.
    float speed_accuracy_mps;
    /// Direction of travel, degrees clockwise from north. Meaningless when
    /// standing still.
    float heading_of_motion_deg;
    /// 1σ heading accuracy estimate, degrees.
    float heading_accuracy_deg;
    /// Position dilution of precision.
    float pdop;
    /// UTC time of the solution, once the receiver has resolved it fully.
    std::optional<UtcTime> utc;
    /// The receiver flagged latitude, longitude and heights as invalid
    /// despite a fix.
    bool invalid_position;

    /// Whether the receiver vouches for @ref latitude_deg and
    /// @ref longitude_deg (`Solution::position()` in the Rust firmware).
    [[nodiscard]] bool has_position() const
    {
        const bool positioned = fix_type == FixType::kFix2D || fix_type == FixType::kFix3D ||
                                fix_type == FixType::kGnssPlusDeadReckoning;
        return positioned && fix_ok && !invalid_position;
    }
};

/// What the receiver says it is, from `UBX-MON-VER`. Strings are NUL
/// terminated and empty when the receiver did not send them.
struct Version {
    /// Receiver software version, e.g. `ROM SPG 4.04 (7f89f7)`.
    char software[31];
    /// Hardware version, e.g. `000A0000`.
    char hardware[11];
    /// Module name from the `MOD=` extension, e.g. `NEO-M9N`.
    char module[31];
    /// UBX protocol version from the `PROTVER=` extension, e.g. `32.01`.
    char protocol[31];
    /// Firmware type from the `FWVER=` extension, e.g. `SPG`.
    char firmware_type[8];
    /// Numbers of the `FWVER=` extension (`SPG 4.04`) and of `PROTVER=`; zero
    /// when absent.
    std::uint8_t firmware_major;
    std::uint8_t firmware_minor;
    std::uint8_t protocol_major;
    std::uint8_t protocol_minor;
};

/// An acknowledgement of a configuration message.
struct Ack {
    /// Class and ID of the message being answered.
    std::uint8_t cls;
    std::uint8_t id;
    /// `UBX-ACK-ACK` when true, `UBX-ACK-NAK` when false.
    bool accepted;
};

/// Each decoder returns nothing if the frame is another message or has an
/// impossible length.
std::optional<Solution> decode_nav_pvt(const Frame& frame);
std::optional<Version> decode_mon_ver(const Frame& frame);
std::optional<Ack> decode_ack(const Frame& frame);

} // namespace ping::ubx
