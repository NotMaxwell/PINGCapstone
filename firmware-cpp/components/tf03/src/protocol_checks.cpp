// Compile-time checks of the TF03 protocol code. Nothing here runs: if a
// static_assert fails, the firmware does not build.

#include "ping/tf03/protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ping::tf03::protocol {
namespace {

/// What a byte stream parses into: how many packets, and the last one.
struct Parsed {
    std::size_t count = 0;
    std::optional<Packet> last;
};

template <std::size_t N>
constexpr Parsed parse(const std::array<std::uint8_t, N>& stream)
{
    Parser parser;
    Parsed parsed;
    for (const std::uint8_t byte : stream) {
        if (auto packet = parser.push(byte)) {
            ++parsed.count;
            parsed.last = packet;
        }
    }
    return parsed;
}

template <std::size_t N>
constexpr bool frame_is(const Command& command, const std::array<std::uint8_t, N>& expected)
{
    return std::ranges::equal(command.bytes(), expected);
}

// Commands, against the frames printed in the manual.
static_assert(frame_is(Command{0x01, {}}, std::array<std::uint8_t, 4>{0x5A, 0x04, 0x01, 0x5F}),
              "firmware version request");
static_assert(frame_is(Command{0x03, std::array<std::uint8_t, 2>{0x64, 0x00}},
                       std::array<std::uint8_t, 6>{0x5A, 0x06, 0x03, 0x64, 0x00, 0xC7}),
              "set frame rate to 100 Hz");
static_assert(Command{0x07, std::array<std::uint8_t, 1>{0x01}}.payload().size() == 1);

// 1000 cm at strength 300.
constexpr std::array<std::uint8_t, 9> kFrame{0x59, 0x59, 0xE8, 0x03, 0x2C, 0x01, 0x00, 0x00, 0xCA};
constexpr DataFrame kFrameFields{.distance_cm = 1000, .strength = 300};

static_assert(parse(kFrame).count == 1);
static_assert(parse(kFrame).last == Packet{kFrameFields});

// Noise before a frame is skipped.
static_assert(parse(std::array<std::uint8_t, 12>{0x00, 0x13, 0x5A, 0x59, 0x59, 0xE8, 0x03, 0x2C,
                                                 0x01, 0x00, 0x00, 0xCA})
                  .last == Packet{kFrameFields});

// A stray 0x59 directly before a frame reads as a false header. Stepping on
// one byte at a time still finds the real frame behind it, and the one after.
static_assert(parse(std::array<std::uint8_t, 19>{0x59, 0x59, 0x59, 0xE8, 0x03, 0x2C, 0x01, 0x00,
                                                 0x00, 0xCA, 0x59, 0x59, 0xE8, 0x03, 0x2C, 0x01,
                                                 0x00, 0x00, 0xCA})
                  .count == 2);

// A corrupted checksum is not a frame.
static_assert(parse(std::array<std::uint8_t, 9>{0x59, 0x59, 0xE8, 0x03, 0x2C, 0x01, 0x00, 0x00,
                                                0xCB})
                  .count == 0);

// A response: firmware version 2.1.3, sent as patch, minor, major.
constexpr std::array<std::uint8_t, 7> kVersion{0x5A, 0x07, 0x01, 0x03, 0x01, 0x02, 0x68};
static_assert(parse(kVersion).count == 1);
static_assert(std::get<Response>(*parse(kVersion).last).id == 0x01);
static_assert(std::get<Response>(*parse(kVersion).last).payload ==
              Payload{.bytes = {0x03, 0x01, 0x02, 0x00}, .len = 3});

// Responses and data frames interleave on the line.
static_assert(parse(std::array<std::uint8_t, 16>{0x59, 0x59, 0xE8, 0x03, 0x2C, 0x01, 0x00, 0x00,
                                                 0xCA, 0x5A, 0x07, 0x01, 0x03, 0x01, 0x02, 0x68})
                  .count == 2);

// A command header with an impossible length is skipped.
static_assert(parse(std::array<std::uint8_t, 11>{0x5A, 0x20, 0x59, 0x59, 0xE8, 0x03, 0x2C, 0x01,
                                                 0x00, 0x00, 0xCA})
                  .last == Packet{kFrameFields});

}  // namespace
}  // namespace ping::tf03::protocol
