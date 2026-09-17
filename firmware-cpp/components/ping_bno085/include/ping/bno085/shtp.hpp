/// @file
/// SHTP — the Sensor Hub Transport Protocol that frames every BNO085 transfer.
///
/// All traffic, in both directions, is a *cargo*: a four byte header followed
/// by a payload. The header carries the total cargo length (header included),
/// the channel the payload belongs to, and a per-channel sequence number.
///
/// Reference: CEVA SH-2 / SHTP Reference Manual, and the reference driver at
/// <https://github.com/ceva-dsp/sh2>.
///
/// Nothing here touches hardware, so it compiles and is unit tested on the
/// host as well as on the ESP32.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace ping::bno085::shtp {

/// Length of the SHTP header prefixing every cargo.
inline constexpr std::size_t kHeaderLen = 4;

/// Set in the length field when a transfer continues the preceding cargo.
inline constexpr std::uint16_t kContinuationBit = 0x8000;

/// The SHTP channels the sensor hub advertises.
enum class Channel : std::uint8_t {
    /// SHTP-level commands and the advertisement sent after reset.
    kCommand = 0,
    /// Reset requests, and the reset-complete notification.
    kExecutable = 1,
    /// Sensor hub control: feature commands, product ID, FRS records.
    kControl = 2,
    /// Sensor reports from non-wake sensors.
    kInputNormal = 3,
    /// Sensor reports from sensors configured as wake sources.
    kInputWake = 4,
    /// The gyro-integrated rotation vector, which has its own channel.
    kInputGyroRv = 5,
};

/// How many channels the hub defines, i.e. how many sequence counters a
/// driver has to keep.
inline constexpr std::size_t kChannelCount = 6;

/// Narrows a wire byte to a [`Channel`], or nothing if the hub named a
/// channel this driver does not know.
constexpr std::optional<Channel> channel_from(std::uint8_t value) noexcept
{
    if (value < kChannelCount) {
        return static_cast<Channel>(value);
    }
    return std::nullopt;
}

/// A decoded SHTP header.
struct Header {
    /// Total cargo length in bytes, *including* these four header bytes.
    std::uint16_t cargo_len;
    /// Whether this transfer continues the previous one.
    bool continuation;
    std::uint8_t channel;
    std::uint8_t sequence;

    /// Payload length, i.e. the cargo without its header.
    [[nodiscard]] constexpr std::size_t payload_len() const noexcept
    {
        return static_cast<std::size_t>(cargo_len) - kHeaderLen;
    }

    /// True when this header came from the given channel.
    [[nodiscard]] constexpr bool is_from(Channel wanted) const noexcept
    {
        return channel == static_cast<std::uint8_t>(wanted);
    }
};

/// Decodes a header, returning nothing when the bytes do not describe a
/// cargo with a payload.
///
/// A hub with nothing to say clocks out zeros, and a hub that is asleep or
/// absent leaves the bus pulled high; a cargo of exactly `kHeaderLen` has no
/// payload to deliver. None of the three is worth reading further.
constexpr std::optional<Header> parse_header(std::span<const std::uint8_t, kHeaderLen> bytes) noexcept
{
    const auto raw = static_cast<std::uint16_t>(bytes[0] | (static_cast<std::uint16_t>(bytes[1]) << 8));
    if (raw == 0x0000 || raw == 0xFFFF) {
        return std::nullopt;
    }

    const auto cargo_len = static_cast<std::uint16_t>(raw & ~kContinuationBit);
    if (cargo_len <= kHeaderLen) {
        return std::nullopt;
    }

    return Header{
        .cargo_len = cargo_len,
        .continuation = (raw & kContinuationBit) != 0,
        .channel = bytes[2],
        .sequence = bytes[3],
    };
}

/// Builds the header for an outgoing payload.
constexpr std::array<std::uint8_t, kHeaderLen>
encode_header(std::size_t payload_len, Channel channel, std::uint8_t sequence) noexcept
{
    const auto cargo_len = static_cast<std::uint16_t>(payload_len + kHeaderLen);
    return {
        static_cast<std::uint8_t>(cargo_len),
        static_cast<std::uint8_t>(cargo_len >> 8),
        static_cast<std::uint8_t>(channel),
        sequence,
    };
}

} // namespace ping::bno085::shtp
