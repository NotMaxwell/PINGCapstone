#include "ping/shtp_spi.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace {

using ping::shtp::Error;
using ping::shtp::SpiBus;
using ping::shtp::SpiTransport;

/// A BNO085 as seen from the bus: a stream of bytes it clocks out, an INT
/// line, and a log of every pin change and transfer.
class FakeBus final : public SpiBus {
  public:
    std::vector<std::string> log;
    std::deque<std::uint8_t> miso;
    std::vector<std::uint8_t> mosi;
    bool int_low = false;
    bool answers_wake = true;
    bool boots = true;
    bool fail_transfers = false;
    bool cs_high = true;
    std::uint32_t time_us = 0;

    void set_cs(bool high) override
    {
        cs_high = high;
        log.emplace_back(high ? "cs+" : "cs-");
    }
    void set_wake(bool high) override
    {
        log.emplace_back(high ? "wake+" : "wake-");
        if (!high && answers_wake) {
            int_low = true;
        }
    }
    void set_reset(bool high) override
    {
        log.emplace_back(high ? "rst+" : "rst-");
        if (high && boots) {
            int_low = true;
        }
    }
    bool int_high() override { return !int_low; }
    bool wait_int_low(std::uint32_t timeout_ms) override
    {
        log.emplace_back("wait" + std::to_string(timeout_ms));
        if (!int_low) {
            time_us += timeout_ms * 1000;
        }
        return int_low;
    }
    bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t len) override
    {
        log.emplace_back("xfer" + std::to_string(len));
        EXPECT_FALSE(cs_high) << "transfer with CS high";
        if (fail_transfers) {
            return false;
        }
        for (std::size_t n = 0; n < len; ++n) {
            mosi.push_back(tx != nullptr ? tx[n] : 0);
            std::uint8_t byte = 0;
            if (!miso.empty()) {
                byte = miso.front();
                miso.pop_front();
            }
            if (rx != nullptr) {
                rx[n] = byte;
            }
        }
        return true;
    }
    void delay_ms(std::uint32_t ms) override
    {
        log.emplace_back("delay" + std::to_string(ms));
        time_us += ms * 1000;
    }
    std::uint32_t now_us() override { return time_us; }

    /// Queues a cargo of `len` bytes on `channel`, payload bytes counting up.
    void queue_cargo(std::uint16_t len, std::uint8_t channel)
    {
        miso.push_back(static_cast<std::uint8_t>(len & 0xFF));
        miso.push_back(static_cast<std::uint8_t>(len >> 8));
        miso.push_back(channel);
        miso.push_back(7);
        for (std::size_t n = 4; n < len; ++n) {
            miso.push_back(static_cast<std::uint8_t>(n));
        }
    }
};

const std::array<std::uint8_t, 6> kFrame{6, 0, 2, 0, 0xF9, 0};

TEST(ShtpSpi, ParseHeaderRejectsEmptyAndIdleBus)
{
    ping::shtp::Header header{};
    const std::array<std::uint8_t, 4> zeros{0, 0, 0, 0};
    const std::array<std::uint8_t, 4> ones{0xFF, 0xFF, 0xFF, 0xFF};
    const std::array<std::uint8_t, 4> bare{4, 0, 1, 0};
    const std::array<std::uint8_t, 4> cont{0x10, 0x81, 3, 9};
    EXPECT_FALSE(ping::shtp::parse_header(zeros, header));
    EXPECT_FALSE(ping::shtp::parse_header(ones, header));
    EXPECT_FALSE(ping::shtp::parse_header(bare, header));
    ASSERT_TRUE(ping::shtp::parse_header(cont, header));
    EXPECT_EQ(header.cargo_len, 0x110);
    EXPECT_TRUE(header.continuation);
    EXPECT_EQ(header.channel, 3);
    EXPECT_EQ(header.sequence, 9);
}

TEST(ShtpSpi, WriteWakesHubFirst)
{
    FakeBus bus;
    SpiTransport transport(bus);
    ASSERT_TRUE(transport.write(kFrame));
    const std::vector<std::string> expected{"wake-", "wait50", "wake+", "cs-", "xfer6", "cs+"};
    EXPECT_EQ(bus.log, expected);
    EXPECT_EQ(bus.mosi, std::vector<std::uint8_t>(kFrame.begin(), kFrame.end()));
    EXPECT_FALSE(transport.has_stash());
}

TEST(ShtpSpi, WriteTimesOutCleanly)
{
    FakeBus bus;
    bus.answers_wake = false;
    SpiTransport transport(bus);
    const auto result = transport.write(kFrame);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().kind, Error::Kind::kTimeout);
    const std::vector<std::string> expected{"wake-", "wait50", "wake+"};
    EXPECT_EQ(bus.log, expected);
    EXPECT_TRUE(bus.mosi.empty());
    EXPECT_TRUE(bus.cs_high);
}

TEST(ShtpSpi, WriteRejectsBadLengths)
{
    FakeBus bus;
    SpiTransport transport(bus);
    const std::array<std::uint8_t, 3> short_frame{3, 0, 0};
    std::array<std::uint8_t, ping::shtp::kMaxWrite + 1> long_frame{};
    EXPECT_EQ(transport.write(short_frame).error().kind, Error::Kind::kBadLength);
    EXPECT_EQ(transport.write(long_frame).error().kind, Error::Kind::kBadLength);
    EXPECT_TRUE(bus.log.empty());
}

TEST(ShtpSpi, ReadWithIntHighReturnsNothing)
{
    FakeBus bus;
    bus.queue_cargo(10, 3);
    SpiTransport transport(bus);
    std::array<std::uint8_t, 64> buf{};
    const auto result = transport.read(buf);
    ASSERT_TRUE(result);
    EXPECT_EQ(*result, 0U);
    EXPECT_TRUE(bus.log.empty());

    const auto waited = transport.read(buf, 5);
    ASSERT_TRUE(waited);
    EXPECT_EQ(*waited, 0U);
    const std::vector<std::string> expected{"wait5"};
    EXPECT_EQ(bus.log, expected);
}

TEST(ShtpSpi, ReadsHeaderThenBody)
{
    FakeBus bus;
    bus.int_low = true;
    bus.queue_cargo(10, 3);
    SpiTransport transport(bus);
    std::array<std::uint8_t, 64> buf{};
    const auto result = transport.read(buf);
    ASSERT_TRUE(result);
    ASSERT_EQ(*result, 10U);
    const std::vector<std::string> expected{"cs-", "xfer4", "xfer6", "cs+"};
    EXPECT_EQ(bus.log, expected);
    EXPECT_EQ(buf[0], 10);
    EXPECT_EQ(buf[2], 3);
    EXPECT_EQ(buf[4], 4);
    EXPECT_EQ(buf[9], 9);
    EXPECT_TRUE(bus.miso.empty());
    // Reading sends zeros, which the hub takes as an empty cargo.
    EXPECT_EQ(bus.mosi, std::vector<std::uint8_t>(10, 0));
}

TEST(ShtpSpi, ReadOfEmptyHeaderStops)
{
    FakeBus bus;
    bus.int_low = true;
    SpiTransport transport(bus);
    std::array<std::uint8_t, 64> buf{};
    const auto result = transport.read(buf);
    ASSERT_TRUE(result);
    EXPECT_EQ(*result, 0U);
    const std::vector<std::string> expected{"cs-", "xfer4", "cs+"};
    EXPECT_EQ(bus.log, expected);
}

TEST(ShtpSpi, CargoClockedInDuringWriteIsReturnedByNextRead)
{
    FakeBus bus;
    bus.queue_cargo(20, 3);
    SpiTransport transport(bus);
    ASSERT_TRUE(transport.write(kFrame));
    const std::vector<std::string> expected{"wake-", "wait50", "wake+", "cs-", "xfer6", "xfer14", "cs+"};
    EXPECT_EQ(bus.log, expected);
    EXPECT_TRUE(transport.has_stash());
    EXPECT_TRUE(bus.miso.empty());

    bus.log.clear();
    bus.int_low = false;
    std::array<std::uint8_t, 64> buf{};
    const auto result = transport.read(buf);
    ASSERT_TRUE(result);
    ASSERT_EQ(*result, 20U);
    EXPECT_TRUE(bus.log.empty()) << "the stash is served without touching the bus";
    EXPECT_EQ(buf[0], 20);
    EXPECT_EQ(buf[5], 5);
    EXPECT_EQ(buf[19], 19);
    EXPECT_FALSE(transport.has_stash());
}

TEST(ShtpSpi, ShortCargoDuringWriteIsStashedWhole)
{
    FakeBus bus;
    bus.queue_cargo(5, 1);
    SpiTransport transport(bus);
    ASSERT_TRUE(transport.write(kFrame));
    std::array<std::uint8_t, 64> buf{};
    const auto result = transport.read(buf);
    ASSERT_TRUE(result);
    EXPECT_EQ(*result, 5U);
    EXPECT_EQ(buf[2], 1);
    EXPECT_EQ(buf[4], 4);
}

TEST(ShtpSpi, OversizedCargoIsDiscardedAndReported)
{
    FakeBus bus;
    bus.int_low = true;
    bus.queue_cargo(100, 3);
    bus.queue_cargo(8, 3);
    SpiTransport transport(bus);
    std::array<std::uint8_t, 32> buf{};
    const auto result = transport.read(buf);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().kind, Error::Kind::kCargoTooLarge);
    EXPECT_EQ(result.error().cargo_len, 100);
    const std::vector<std::string> expected{"cs-", "xfer4", "xfer96", "cs+"};
    EXPECT_EQ(bus.log, expected);

    // The bus is still in step: the next cargo reads normally.
    const auto next = transport.read(buf);
    ASSERT_TRUE(next);
    EXPECT_EQ(*next, 8U);
}

TEST(ShtpSpi, OversizedCargoDuringWriteIsDiscarded)
{
    FakeBus bus;
    bus.queue_cargo(static_cast<std::uint16_t>(ping::shtp::kMaxCargo + 10), 0);
    SpiTransport transport(bus);
    ASSERT_TRUE(transport.write(kFrame));
    EXPECT_FALSE(transport.has_stash());
    EXPECT_TRUE(bus.miso.empty());
}

TEST(ShtpSpi, StashTooLargeForBufferIsReported)
{
    FakeBus bus;
    bus.queue_cargo(40, 3);
    SpiTransport transport(bus);
    ASSERT_TRUE(transport.write(kFrame));
    std::array<std::uint8_t, 16> buf{};
    const auto result = transport.read(buf);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().kind, Error::Kind::kCargoTooLarge);
    EXPECT_FALSE(transport.has_stash());
}

TEST(ShtpSpi, BusFailureReleasesChipSelect)
{
    FakeBus bus;
    bus.int_low = true;
    bus.fail_transfers = true;
    SpiTransport transport(bus);
    std::array<std::uint8_t, 32> buf{};
    const auto read = transport.read(buf);
    ASSERT_FALSE(read);
    EXPECT_EQ(read.error().kind, Error::Kind::kBus);
    EXPECT_TRUE(bus.cs_high);

    const auto write = transport.write(kFrame);
    ASSERT_FALSE(write);
    EXPECT_EQ(write.error().kind, Error::Kind::kBus);
    EXPECT_TRUE(bus.cs_high);
}

TEST(ShtpSpi, ResetSequence)
{
    FakeBus bus;
    SpiTransport transport(bus);
    ASSERT_TRUE(transport.reset());
    const std::vector<std::string> expected{"wake+", "cs+", "rst-", "delay10", "rst+", "wait500"};
    EXPECT_EQ(bus.log, expected);
}

TEST(ShtpSpi, ResetTimesOutWhenHubDoesNotBoot)
{
    FakeBus bus;
    bus.boots = false;
    SpiTransport transport(bus);
    const auto result = transport.reset();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().kind, Error::Kind::kTimeout);
}

TEST(ShtpSpi, ResetDropsStash)
{
    FakeBus bus;
    bus.queue_cargo(20, 3);
    SpiTransport transport(bus);
    ASSERT_TRUE(transport.write(kFrame));
    ASSERT_TRUE(transport.has_stash());
    ASSERT_TRUE(transport.reset());
    EXPECT_FALSE(transport.has_stash());
}

} // namespace
