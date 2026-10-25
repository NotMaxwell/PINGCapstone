#include "ping/tf03_protocol.hpp"

#include <algorithm>

namespace ping::tf03 {
namespace {

std::uint16_t le16(const std::uint8_t* p) { return static_cast<std::uint16_t>(p[0] | (p[1] << 8)); }

/// Decodes a packet whose header, length and checksum have been checked.
Packet decode(std::span<const std::uint8_t> packet)
{
    if (packet[0] == kDataHeader) {
        return DataFrame{.distance_cm = le16(&packet[2]), .strength = le16(&packet[4])};
    }

    const auto received = packet.subspan(3, packet.size() - kMinCommandLen);
    Response response{.id = packet[2], .payload = {}};
    std::ranges::copy(received, response.payload.bytes.begin());
    response.payload.len = received.size();
    return response;
}

} // namespace

std::uint8_t checksum(std::span<const std::uint8_t> bytes)
{
    std::uint8_t sum = 0;
    for (const std::uint8_t byte : bytes) {
        sum = static_cast<std::uint8_t>(sum + byte);
    }
    return sum;
}

bool operator==(const Payload& a, const Payload& b) { return std::ranges::equal(a.view(), b.view()); }

Command::Command(std::uint8_t id, std::span<const std::uint8_t> payload)
{
    const std::size_t payload_len = std::min(payload.size(), kMaxPayload);
    len_ = kMinCommandLen + payload_len;
    bytes_[0] = kCommandHeader;
    bytes_[1] = static_cast<std::uint8_t>(len_);
    bytes_[2] = id;
    std::copy_n(payload.begin(), payload_len, bytes_.begin() + 3);
    bytes_[len_ - 1] = checksum({bytes_.data(), len_ - 1});
}

Payload Command::payload() const
{
    Payload payload;
    payload.len = len_ - kMinCommandLen;
    std::copy_n(bytes_.begin() + 3, payload.len, payload.bytes.begin());
    return payload;
}

std::optional<Packet> Parser::push(std::uint8_t byte)
{
    // There is always room: this only returns once the buffer is shorter
    // than the packet it starts with, and no packet is longer than the
    // buffer.
    buf_[len_++] = byte;

    for (;;) {
        while (len_ > 0 && !plausible_start()) {
            consume(1);
        }

        const auto needed = expected_len();
        if (!needed || len_ < *needed) {
            return std::nullopt;
        }

        const std::span<const std::uint8_t> candidate{buf_.data(), *needed};
        if (checksum(candidate.first(*needed - 1)) == candidate.back()) {
            Packet packet = decode(candidate);
            consume(*needed);
            return packet;
        }

        // A false header, or a corrupted packet. A real one may begin
        // anywhere after its first byte.
        consume(1);
    }
}

bool Parser::plausible_start() const
{
    if (buf_[0] != kDataHeader && buf_[0] != kCommandHeader) {
        return false;
    }
    if (len_ == 1) {
        return true;
    }
    if (buf_[0] == kDataHeader) {
        return buf_[1] == kDataHeader;
    }
    return buf_[1] >= kMinCommandLen && buf_[1] <= kMaxCommandLen;
}

std::optional<std::size_t> Parser::expected_len() const
{
    if (len_ < 2) {
        return std::nullopt;
    }
    return buf_[0] == kDataHeader ? kDataFrameLen : static_cast<std::size_t>(buf_[1]);
}

void Parser::consume(std::size_t count)
{
    std::copy(buf_.begin() + static_cast<std::ptrdiff_t>(count), buf_.begin() + static_cast<std::ptrdiff_t>(len_),
              buf_.begin());
    len_ -= count;
}

} // namespace ping::tf03
