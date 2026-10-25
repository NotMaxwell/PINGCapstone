/// @file
/// Wire format of the TF03's UART protocol.
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
/// Every packet ends in the low eight bits of the sum of the bytes before it,
/// and multi-byte fields are little endian.
///
/// No library covers the TF03, so this is written here, following Benewake's
/// *TF03 UART/CAN User Manual* (the 2024 edition, and V1.2.2 as hosted by
/// Seeed) cross-checked against ArduPilot's
/// `libraries/AP_RangeFinder/AP_RangeFinder_Benewake.cpp`.
///
/// Nothing here touches hardware, so the host tests exercise it directly.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>

namespace ping::tf03 {

/// Both of the first two bytes of every data frame.
inline constexpr std::uint8_t kDataHeader = 0x59;
/// Length of a data frame.
inline constexpr std::size_t kDataFrameLen = 9;
/// First byte of every command and every response.
inline constexpr std::uint8_t kCommandHeader = 0x5A;
/// Shortest command frame: header, length, ID and checksum, with no payload.
inline constexpr std::size_t kMinCommandLen = 4;
/// Longest command frame in the protocol, which belongs to the commands
/// taking a four byte parameter (UART baud rate, CAN IDs).
inline constexpr std::size_t kMaxCommandLen = 8;
/// Longest command payload.
inline constexpr std::size_t kMaxPayload = kMaxCommandLen - kMinCommandLen;

/// The low eight bits of the sum of `bytes`.
std::uint8_t checksum(std::span<const std::uint8_t> bytes);

/// A run of at most @ref kMaxPayload bytes, held by value.
struct Payload {
    std::array<std::uint8_t, kMaxPayload> bytes{};
    std::size_t len = 0;

    [[nodiscard]] std::span<const std::uint8_t> view() const { return {bytes.data(), len}; }
    friend bool operator==(const Payload& a, const Payload& b);
};

/// An outgoing command frame.
class Command {
  public:
    /// Frames `payload` as the command `id`. A payload longer than
    /// @ref kMaxPayload is cut short; no command in the protocol has one.
    Command(std::uint8_t id, std::span<const std::uint8_t> payload = {});

    [[nodiscard]] std::uint8_t id() const { return bytes_[2]; }
    [[nodiscard]] Payload payload() const;
    /// The complete frame, ready to write to the UART.
    [[nodiscard]] std::span<const std::uint8_t> bytes() const { return {bytes_.data(), len_}; }

  private:
    std::array<std::uint8_t, kMaxCommandLen> bytes_{};
    std::size_t len_;
};

/// The fields of a data frame, exactly as sent.
struct DataFrame {
    std::uint16_t distance_cm;
    std::uint16_t strength;

    friend bool operator==(const DataFrame&, const DataFrame&) = default;
};

/// A command frame sent back by the TF03 in answer to a command.
struct Response {
    /// The ID of the command being answered.
    std::uint8_t id;
    Payload payload;

    friend bool operator==(const Response&, const Response&) = default;
};

/// A packet received from the TF03.
using Packet = std::variant<DataFrame, Response>;

/// Pulls packets out of a byte stream.
///
/// Bytes are fed in one at a time as they arrive. A header is not trusted
/// until the checksum at the end of its packet agrees, and on a mismatch the
/// parser steps forward a single byte rather than throwing the whole
/// candidate away. Discarding whole candidates can lock a parser out of step:
/// a checksum byte of `0x59` directly before a real frame reads as a header,
/// and for as long as the measurement does not change, every retry after a
/// nine byte discard lands on the same false header.
class Parser {
  public:
    /// Forgets any partly received packet, e.g. after bytes were lost.
    void reset() { len_ = 0; }

    /// Adds one received byte, returning a packet if it completed one.
    std::optional<Packet> push(std::uint8_t byte);

  private:
    [[nodiscard]] bool plausible_start() const;
    [[nodiscard]] std::optional<std::size_t> expected_len() const;
    void consume(std::size_t count);

    static constexpr std::size_t kMaxPacketLen = kDataFrameLen > kMaxCommandLen ? kDataFrameLen : kMaxCommandLen;
    std::array<std::uint8_t, kMaxPacketLen> buf_{};
    std::size_t len_ = 0;
};

} // namespace ping::tf03
