/// @file
/// Wire format of the Benewake TF03's UART protocol.
///
/// Two kinds of packet share the line, told apart by their first byte:
///
/// * **Data frames**, nine bytes, sent continuously at the configured frame
///   rate (or once per trigger command):
///   `59 59 DIST_L DIST_H STRENGTH_L STRENGTH_H reserved reserved SUM`
/// * **Command frames**, `5A LEN ID payload… SUM`, where `LEN` counts every
///   byte including the header and the checksum. The host sends these, and
///   the TF03 answers in the same shape.
///
/// Every packet ends in the low eight bits of the sum of the bytes before
/// it, and multi-byte fields are little endian.
///
/// Benewake publishes no host driver for the TF03, only this protocol, so
/// this is written from its *TF03 UART/CAN User Manual* (the 2024 edition,
/// and V1.2.2), cross-checked against ArduPilot's
/// `libraries/AP_RangeFinder/AP_RangeFinder_Benewake.cpp`.
///
/// Everything here is constexpr and free of ESP-IDF, so the parser is
/// checked at compile time; see src/protocol_checks.cpp.

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>

namespace ping::tf03::protocol {

/// Both of the first two bytes of every data frame.
inline constexpr std::uint8_t kDataHeader = 0x59;

/// Length of a data frame.
inline constexpr std::size_t kDataFrameLen = 9;

/// First byte of every command and every response.
inline constexpr std::uint8_t kCommandHeader = 0x5A;

/// Shortest command frame: header, length, ID and checksum, no payload.
inline constexpr std::size_t kMinCommandLen = 4;

/// Longest command frame in the protocol, which belongs to the commands
/// taking a four byte parameter (UART baud rate, CAN IDs).
inline constexpr std::size_t kMaxCommandLen = 8;

/// Longest command payload.
inline constexpr std::size_t kMaxPayload = kMaxCommandLen - kMinCommandLen;

/// Longest packet of either kind, and so how much the parser has to hold.
inline constexpr std::size_t kMaxPacketLen = std::max(kDataFrameLen, kMaxCommandLen);

/// The low eight bits of the sum of `bytes`.
constexpr std::uint8_t checksum(std::span<const std::uint8_t> bytes)
{
    std::uint8_t sum = 0;
    for (const std::uint8_t byte : bytes) {
        sum = static_cast<std::uint8_t>(sum + byte);
    }
    return sum;
}

/// A short run of bytes held by value: a command's or response's payload.
struct Payload {
    std::array<std::uint8_t, kMaxPayload> bytes{};
    std::size_t len = 0;

    [[nodiscard]] constexpr std::span<const std::uint8_t> view() const
    {
        return std::span{bytes}.first(len);
    }

    friend constexpr bool operator==(const Payload& a, const Payload& b)
    {
        return std::ranges::equal(a.view(), b.view());
    }
};

/// An outgoing command frame.
class Command {
public:
    /// Frames `payload` as the command `id`. No command in the protocol has
    /// a payload longer than kMaxPayload; anything past that is dropped.
    constexpr Command(std::uint8_t id, std::span<const std::uint8_t> payload)
    {
        const std::size_t payload_len = std::min(payload.size(), kMaxPayload);
        len_ = kMinCommandLen + payload_len;
        bytes_[0] = kCommandHeader;
        bytes_[1] = static_cast<std::uint8_t>(len_);
        bytes_[2] = id;
        std::ranges::copy(payload.first(payload_len), bytes_.begin() + 3);
        bytes_[len_ - 1] = checksum(std::span{bytes_}.first(len_ - 1));
    }

    [[nodiscard]] constexpr std::uint8_t id() const { return bytes_[2]; }

    [[nodiscard]] constexpr std::span<const std::uint8_t> payload() const
    {
        return std::span{bytes_}.subspan(3, len_ - kMinCommandLen);
    }

    /// The complete frame, ready to write to the UART.
    [[nodiscard]] constexpr std::span<const std::uint8_t> bytes() const
    {
        return std::span{bytes_}.first(len_);
    }

private:
    std::array<std::uint8_t, kMaxCommandLen> bytes_{};
    std::size_t len_ = 0;
};

/// The fields of a data frame, exactly as sent.
struct DataFrame {
    std::uint16_t distance_cm;
    std::uint16_t strength;

    friend constexpr bool operator==(const DataFrame&, const DataFrame&) = default;
};

/// A command frame sent back by the TF03 in answer to a command.
struct Response {
    /// The ID of the command being answered.
    std::uint8_t id;
    Payload payload;

    friend constexpr bool operator==(const Response&, const Response&) = default;
};

/// A packet received from the TF03.
using Packet = std::variant<DataFrame, Response>;

/// Pulls packets out of a byte stream.
///
/// Bytes are fed in one at a time as they arrive. A header is not trusted
/// until the checksum at the end of its packet agrees, and on a mismatch the
/// parser steps forward a single byte rather than throwing the whole
/// candidate away. Discarding whole candidates can lock a parser out of
/// step: a checksum byte of 0x59 directly before a real frame reads as a
/// header, and for as long as the measurement does not change, every retry
/// after a nine byte discard lands on the same false header.
class Parser {
public:
    /// Forgets any partly received packet, e.g. after bytes were lost.
    constexpr void reset() { len_ = 0; }

    /// Adds one received byte, returning a packet if it completed one.
    constexpr std::optional<Packet> push(std::uint8_t byte)
    {
        // There is always room: settle() only returns once the buffer is
        // shorter than the packet it starts with, and no packet is longer
        // than the buffer.
        buf_[len_++] = byte;
        return settle();
    }

private:
    /// Decodes the packet at the front of the buffer if it is complete and
    /// sound, stepping past bytes that cannot begin one.
    constexpr std::optional<Packet> settle()
    {
        for (;;) {
            while (len_ > 0 && !plausible_start()) {
                consume(1);
            }

            const std::optional<std::size_t> needed = expected_len();
            if (!needed || len_ < *needed) {
                return std::nullopt;
            }

            const auto candidate = std::span{buf_}.first(*needed);
            if (checksum(candidate.first(*needed - 1)) == candidate.back()) {
                const Packet packet = decode(candidate);
                consume(*needed);
                return packet;
            }

            // A false header, or a corrupted packet. A real one may begin
            // anywhere after its first byte.
            consume(1);
        }
    }

    /// Whether the buffered bytes could be the beginning of a packet.
    [[nodiscard]] constexpr bool plausible_start() const
    {
        switch (buf_[0]) {
        case kDataHeader:
            return len_ == 1 || buf_[1] == kDataHeader;
        case kCommandHeader:
            return len_ == 1 || (buf_[1] >= kMinCommandLen && buf_[1] <= kMaxCommandLen);
        default:
            return false;
        }
    }

    /// Length of the packet the buffer begins with, once enough of it has
    /// arrived to tell.
    [[nodiscard]] constexpr std::optional<std::size_t> expected_len() const
    {
        if (len_ < 2) {
            return std::nullopt;
        }
        return buf_[0] == kDataHeader ? kDataFrameLen : std::size_t{buf_[1]};
    }

    constexpr void consume(std::size_t count)
    {
        std::copy(buf_.begin() + static_cast<std::ptrdiff_t>(count),
                  buf_.begin() + static_cast<std::ptrdiff_t>(len_), buf_.begin());
        len_ -= count;
    }

    /// Decodes a packet whose header, length and checksum have been checked.
    static constexpr Packet decode(std::span<const std::uint8_t> packet)
    {
        const auto le16 = [&](std::size_t at) {
            return static_cast<std::uint16_t>(packet[at] | (packet[at + 1] << 8));
        };
        if (packet[0] == kDataHeader) {
            return DataFrame{.distance_cm = le16(2), .strength = le16(4)};
        }
        const auto received = packet.subspan(3, packet.size() - kMinCommandLen);
        Response response{.id = packet[2], .payload = {}};
        std::ranges::copy(received, response.payload.bytes.begin());
        response.payload.len = received.size();
        return response;
    }

    std::array<std::uint8_t, kMaxPacketLen> buf_{};
    std::size_t len_ = 0;
};

}  // namespace ping::tf03::protocol
