/// @file
/// TF03 wire protocol tests. Every case here is something the parser has to
/// survive on a real, noisy, already-streaming line.

#include "ping/tf03_protocol.hpp"

#include <gtest/gtest.h>

#include <vector>

using namespace ping::tf03;

namespace {

/// A data frame as the TF03 sends it, checksum included.
std::vector<std::uint8_t> data_frame(std::uint16_t distance_cm, std::uint16_t strength)
{
    std::vector<std::uint8_t> frame{
        kDataHeader,
        kDataHeader,
        static_cast<std::uint8_t>(distance_cm),
        static_cast<std::uint8_t>(distance_cm >> 8),
        static_cast<std::uint8_t>(strength),
        static_cast<std::uint8_t>(strength >> 8),
        0,
        0,
    };
    frame.push_back(checksum(frame));
    return frame;
}

/// Feeds `bytes` to `parser` and collects every packet that comes out.
std::vector<Packet> feed(Parser& parser, const std::vector<std::uint8_t>& bytes)
{
    std::vector<Packet> packets;
    for (const std::uint8_t byte : bytes) {
        if (auto packet = parser.push(byte)) {
            packets.push_back(*packet);
        }
    }
    return packets;
}

} // namespace

TEST(Tf03Protocol, ChecksumIsTheLowByteOfTheSum)
{
    const std::vector<std::uint8_t> bytes{0xFF, 0x02, 0x10};
    EXPECT_EQ(checksum(bytes), 0x11);
}

TEST(Tf03Protocol, FramesACommand)
{
    // The firmware version request from the manual: 5A 04 01 5F.
    const Command command{0x01};
    const std::vector<std::uint8_t> expected{0x5A, 0x04, 0x01, 0x5F};
    EXPECT_EQ(std::vector<std::uint8_t>(command.bytes().begin(), command.bytes().end()), expected);
    EXPECT_EQ(command.id(), 0x01);
    EXPECT_EQ(command.payload().len, 0U);
}

TEST(Tf03Protocol, FramesACommandWithAPayload)
{
    // Frame rate 100 Hz: 5A 06 03 64 00 C7.
    const std::array<std::uint8_t, 2> rate{0x64, 0x00};
    const Command command{0x03, rate};
    const std::vector<std::uint8_t> expected{0x5A, 0x06, 0x03, 0x64, 0x00, 0xC7};
    EXPECT_EQ(std::vector<std::uint8_t>(command.bytes().begin(), command.bytes().end()), expected);
}

TEST(Tf03Protocol, DecodesADataFrame)
{
    Parser parser;
    const auto packets = feed(parser, data_frame(1234, 567));
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(std::get<DataFrame>(packets[0]), (DataFrame{.distance_cm = 1234, .strength = 567}));
}

TEST(Tf03Protocol, DecodesAResponse)
{
    // Firmware version 1.2.3, sent patch first.
    std::vector<std::uint8_t> frame{0x5A, 0x07, 0x01, 3, 2, 1};
    frame.push_back(checksum(frame));

    Parser parser;
    const auto packets = feed(parser, frame);
    ASSERT_EQ(packets.size(), 1U);
    const auto& response = std::get<Response>(packets[0]);
    EXPECT_EQ(response.id, 0x01);
    ASSERT_EQ(response.payload.len, 3U);
    EXPECT_EQ(response.payload.bytes[0], 3);
    EXPECT_EQ(response.payload.bytes[2], 1);
}

TEST(Tf03Protocol, SkipsGarbageBeforeAFrame)
{
    std::vector<std::uint8_t> bytes{0x00, 0x13, 0x59, 0x42};
    const auto frame = data_frame(100, 200);
    bytes.insert(bytes.end(), frame.begin(), frame.end());

    Parser parser;
    const auto packets = feed(parser, bytes);
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(std::get<DataFrame>(packets[0]).distance_cm, 100);
}

TEST(Tf03Protocol, RejectsACorruptedFrameAndRecoversOnTheNext)
{
    auto corrupted = data_frame(100, 200);
    corrupted[3] ^= 0x01;
    auto bytes = corrupted;
    const auto good = data_frame(300, 400);
    bytes.insert(bytes.end(), good.begin(), good.end());

    Parser parser;
    const auto packets = feed(parser, bytes);
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(std::get<DataFrame>(packets[0]).distance_cm, 300);
}

TEST(Tf03Protocol, DoesNotLockOntoAFalseHeader)
{
    // A frame whose checksum is 0x59, followed by more of the same: a parser
    // that throws away nine bytes after a bad candidate keeps landing on the
    // false header formed by that checksum and the next frame's first byte.
    // Find a distance that makes the checksum 0x59.
    std::vector<std::uint8_t> frame;
    for (std::uint16_t distance = 0; distance < 0x100; ++distance) {
        frame = data_frame(distance, 0);
        if (frame.back() == kDataHeader) {
            break;
        }
    }
    ASSERT_EQ(frame.back(), kDataHeader);

    // Start mid-frame, so the parser has to find its footing.
    std::vector<std::uint8_t> bytes(frame.begin() + 4, frame.end());
    for (int i = 0; i < 5; ++i) {
        bytes.insert(bytes.end(), frame.begin(), frame.end());
    }

    Parser parser;
    EXPECT_EQ(feed(parser, bytes).size(), 5U);
}

TEST(Tf03Protocol, ResetForgetsAPartialPacket)
{
    const auto frame = data_frame(100, 200);
    Parser parser;
    feed(parser, std::vector<std::uint8_t>(frame.begin(), frame.begin() + 5));
    parser.reset();
    // The tail of the old frame no longer completes anything.
    EXPECT_TRUE(feed(parser, std::vector<std::uint8_t>(frame.begin() + 5, frame.end())).empty());
    EXPECT_EQ(feed(parser, frame).size(), 1U);
}
