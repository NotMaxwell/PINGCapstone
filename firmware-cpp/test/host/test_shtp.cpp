/// @file
/// SHTP framing tests. Every case here is a bus condition the driver has to
/// survive on real hardware.

#include "ping/bno085/shtp.hpp"

#include <gtest/gtest.h>

using namespace ping::bno085::shtp;

namespace {

std::optional<Header> parse(std::array<std::uint8_t, kHeaderLen> bytes)
{
    return parse_header(std::span<const std::uint8_t, kHeaderLen>{bytes});
}

} // namespace

TEST(Shtp, RejectsAllZeroHeader)
{
    // A hub with nothing to say clocks out zeros.
    EXPECT_FALSE(parse({0x00, 0x00, 0x00, 0x00}).has_value());
}

TEST(Shtp, RejectsAllOnesHeader)
{
    // A hub that is asleep or absent leaves the bus pulled high.
    EXPECT_FALSE(parse({0xFF, 0xFF, 0x00, 0x00}).has_value());
}

TEST(Shtp, RejectsHeaderOnlyCargo)
{
    // A cargo of exactly the header length has no payload to deliver.
    EXPECT_FALSE(parse({0x04, 0x00, 0x02, 0x00}).has_value());
    EXPECT_FALSE(parse({0x03, 0x00, 0x02, 0x00}).has_value());
}

TEST(Shtp, ParsesOrdinaryHeader)
{
    const auto header = parse({0x13, 0x00, 0x03, 0x2A});
    ASSERT_TRUE(header.has_value());
    EXPECT_EQ(header->cargo_len, 19);
    EXPECT_EQ(header->payload_len(), 15u);
    EXPECT_FALSE(header->continuation);
    EXPECT_EQ(header->channel, 3);
    EXPECT_EQ(header->sequence, 0x2A);
    EXPECT_TRUE(header->is_from(Channel::kInputNormal));
    EXPECT_FALSE(header->is_from(Channel::kControl));
}

TEST(Shtp, StripsContinuationBitFromLength)
{
    // The top bit of the length field flags a continuation, and is not part
    // of the length itself.
    const auto header = parse({0x13, 0x80, 0x02, 0x01});
    ASSERT_TRUE(header.has_value());
    EXPECT_EQ(header->cargo_len, 19);
    EXPECT_TRUE(header->continuation);
}

TEST(Shtp, EncodesHeaderForOutgoingPayload)
{
    // 13 bytes of payload on the control channel, sequence 7.
    const auto header = encode_header(13, Channel::kControl, 7);
    EXPECT_EQ(header[0], 17); // 13 + 4
    EXPECT_EQ(header[1], 0);
    EXPECT_EQ(header[2], static_cast<std::uint8_t>(Channel::kControl));
    EXPECT_EQ(header[3], 7);
}

TEST(Shtp, EncodeAndParseRoundTrip)
{
    const auto encoded = encode_header(300, Channel::kInputWake, 0xFE);
    const auto header = parse_header(std::span<const std::uint8_t, kHeaderLen>{encoded});
    ASSERT_TRUE(header.has_value());
    EXPECT_EQ(header->payload_len(), 300u);
    EXPECT_EQ(header->channel, static_cast<std::uint8_t>(Channel::kInputWake));
    EXPECT_EQ(header->sequence, 0xFE);
    EXPECT_FALSE(header->continuation);
}

TEST(Shtp, NarrowsKnownChannelsOnly)
{
    EXPECT_EQ(channel_from(0), Channel::kCommand);
    EXPECT_EQ(channel_from(5), Channel::kInputGyroRv);
    EXPECT_FALSE(channel_from(6).has_value());
    EXPECT_FALSE(channel_from(0xFF).has_value());
}
