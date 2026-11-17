#include "ping/shtp_spi.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace {

using ping::shtp::SpiBus;
using ping::shtp::SpiHal;

/// A BNO08x as the HAL sees it: cargos it wants to send, an INTN line it
/// pulls with each one, and a log of every pin change and transfer.
class FakeHub final : public SpiBus {
  public:
    SpiHal* hal = nullptr;
    std::vector<std::string> log;
    std::deque<std::vector<std::uint8_t>> outgoing;
    std::vector<std::vector<std::uint8_t>> received;
    bool boots = true;
    bool answers_wake = true;
    bool init_ok = true;
    bool fail_transfers = false;
    std::uint32_t now = 0;

    bool init_hw() override
    {
        log.emplace_back("init");
        return init_ok;
    }
    void deinit_hw() override { log.emplace_back("deinit"); }
    void rstn(bool high) override
    {
        log.emplace_back(high ? "rst+" : "rst-");
        if (high && boots) {
            queue({0x10, 0x00, 0x00, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12});
        }
    }
    void ps0_waken(bool high) override
    {
        log.emplace_back(high ? "wake+" : "wake-");
        if (!high && answers_wake) {
            edge();
        }
    }
    void csn(bool high) override
    {
        log.emplace_back(high ? "cs+" : "cs-");
        cs_low_ = !high;
        if (high && clocked_ > 0) {
            if (!mosi_.empty() && (mosi_[0] | mosi_[1]) != 0) {
                received.push_back(mosi_);
            }
            if (!outgoing.empty() && clocked_ >= kHeader) {
                outgoing.pop_front();
            }
            clocked_ = 0;
            mosi_.clear();
            if (!outgoing.empty()) {
                edge();
            }
        }
    }
    bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t len) override
    {
        log.emplace_back("xfer" + std::to_string(len));
        if (fail_transfers) {
            return false;
        }
        if (!cs_low_) {
            return true; // the dummy clock-settling transfer
        }
        for (std::size_t n = 0; n < len; ++n) {
            mosi_.push_back(tx[n]);
            const auto& cargo = outgoing.empty() ? kNothing : outgoing.front();
            rx[n] = clocked_ < cargo.size() ? cargo[clocked_] : 0;
            ++clocked_;
        }
        return true;
    }
    void enable_ints() override
    {
        log.emplace_back("ints+");
        ints_ = true;
        if (pending_) {
            pending_ = false;
            edge();
        }
    }
    void disable_ints() override
    {
        log.emplace_back("ints-");
        ints_ = false;
    }
    void wait_int(std::uint32_t timeout_us) override
    {
        if (fired_) {
            fired_ = false;
            return;
        }
        now += timeout_us;
    }
    void delay_us(std::uint32_t us) override
    {
        log.emplace_back("delay" + std::to_string(us));
        now += us;
    }
    std::uint32_t time_now_us() override { return now; }

    /// Queues a cargo and, if the bus is free, pulls INTN.
    void queue(std::vector<std::uint8_t> cargo)
    {
        outgoing.push_back(std::move(cargo));
        if (outgoing.size() == 1 && !cs_low_) {
            edge();
        }
    }

  private:
    static constexpr std::size_t kHeader = 4;
    inline static const std::vector<std::uint8_t> kNothing{};

    void edge()
    {
        if (!ints_) {
            pending_ = true;
            return;
        }
        hal->on_intn(now);
        fired_ = true;
    }

    bool cs_low_ = false;
    bool ints_ = false;
    bool pending_ = false;
    bool fired_ = false;
    std::size_t clocked_ = 0;
    std::vector<std::uint8_t> mosi_;
};

struct Rig {
    FakeHub hub;
    SpiHal hal{hub};
    Rig() { hub.hal = &hal; }
};

std::vector<std::uint8_t> cargo(std::uint16_t len, std::uint8_t channel)
{
    std::vector<std::uint8_t> out{static_cast<std::uint8_t>(len & 0xFF), static_cast<std::uint8_t>(len >> 8),
                                  channel, 0};
    for (std::size_t n = 4; n < len; ++n) {
        out.push_back(static_cast<std::uint8_t>(n));
    }
    return out;
}

const std::vector<std::uint8_t> kCommand{6, 0, 2, 0, 0xF9, 0};

TEST(SpiHal, OpenFollowsCevaResetSequence)
{
    Rig rig;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    const std::vector<std::string> expected{"init", "rst-",  "cs+",  "xfer1", "delay10000",
                                            "wake+", "rst+", "ints+"};
    EXPECT_EQ(rig.hub.log, expected);
    EXPECT_FALSE(rig.hal.in_reset());
    EXPECT_EQ(rig.hub.now, 10'000U);
}

TEST(SpiHal, OpenWaitsStartDelayForAHubThatDoesNotBoot)
{
    Rig rig;
    rig.hub.boots = false;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    EXPECT_TRUE(rig.hal.in_reset());
    EXPECT_EQ(rig.hub.now, 10'000U + ping::shtp::kStartDelayUs);
}

TEST(SpiHal, OpenFailsWhenThePlatformRefuses)
{
    Rig rig;
    rig.hub.init_ok = false;
    EXPECT_EQ(rig.hal.open(), ping::shtp::kSh2ErrIo);
    EXPECT_FALSE(rig.hal.is_open());
}

TEST(SpiHal, SecondOpenIsRefused)
{
    Rig rig;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    EXPECT_EQ(rig.hal.open(), ping::shtp::kSh2Err);
}

TEST(SpiHal, ReadsHeaderThenBodyOnIntn)
{
    Rig rig;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    rig.hub.log.clear();
    rig.hub.now = 12'345;

    std::array<std::uint8_t, 1024> buf{};
    std::uint32_t t = 0;
    const int len = rig.hal.read(buf.data(), buf.size(), &t);
    ASSERT_EQ(len, 16);
    EXPECT_EQ(buf[4], 1);
    EXPECT_EQ(buf[15], 12);
    // Stamped when INTN was seen (as interrupts came on), not when read.
    EXPECT_EQ(t, 10'000U);
    const std::vector<std::string> expected{"cs-", "xfer4", "xfer12", "cs+"};
    EXPECT_EQ(rig.hub.log, expected);
}

TEST(SpiHal, ReadWithoutIntnReturnsNothing)
{
    Rig rig;
    rig.hub.boots = false;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    rig.hub.log.clear();
    std::array<std::uint8_t, 64> buf{};
    std::uint32_t t = 0;
    EXPECT_EQ(rig.hal.read(buf.data(), buf.size(), &t), 0);
    EXPECT_TRUE(rig.hub.log.empty());
}

TEST(SpiHal, ReadWaitsForIntnWhenAsked)
{
    Rig rig;
    rig.hub.boots = false;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    rig.hal.set_read_wait_us(5'000);
    const std::uint32_t before = rig.hub.now;
    std::array<std::uint8_t, 64> buf{};
    std::uint32_t t = 0;
    EXPECT_EQ(rig.hal.read(buf.data(), buf.size(), &t), 0);
    EXPECT_EQ(rig.hub.now - before, 5'000U);
}

TEST(SpiHal, EmptyHeaderEndsTheTransfer)
{
    Rig rig;
    rig.hub.boots = false;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    rig.hal.on_intn(0); // spurious INTN, the hub sends zeros
    rig.hub.log.clear();
    std::array<std::uint8_t, 64> buf{};
    std::uint32_t t = 0;
    EXPECT_EQ(rig.hal.read(buf.data(), buf.size(), &t), 0);
    const std::vector<std::string> expected{"cs-", "xfer4", "cs+"};
    EXPECT_EQ(rig.hub.log, expected);
}

TEST(SpiHal, WriteAssertsWakeAndSendsOnIntn)
{
    Rig rig;
    rig.hub.boots = false;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    rig.hub.log.clear();

    ASSERT_EQ(rig.hal.write(kCommand.data(), static_cast<unsigned>(kCommand.size())), 6);
    const std::vector<std::string> after_write{"wake-"};
    EXPECT_EQ(rig.hub.log, after_write) << "nothing is clocked until the hub answers";

    std::array<std::uint8_t, 64> buf{};
    std::uint32_t t = 0;
    EXPECT_EQ(rig.hal.read(buf.data(), buf.size(), &t), 0);
    const std::vector<std::string> expected{"wake-", "cs-", "wake+", "xfer6", "cs+"};
    EXPECT_EQ(rig.hub.log, expected);
    ASSERT_EQ(rig.hub.received.size(), 1U);
    EXPECT_EQ(rig.hub.received[0], kCommand);
}

TEST(SpiHal, CargoClockedInDuringWriteIsDeliveredTruncated)
{
    Rig rig;
    rig.hub.boots = false;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    rig.hub.queue(cargo(20, 3));
    std::array<std::uint8_t, 64> buf{};
    std::uint32_t t = 0;
    // CEVA's read is the full cargo here, so drain it, then queue one that
    // will meet the write.
    ASSERT_EQ(rig.hal.read(buf.data(), buf.size(), &t), 20);

    ASSERT_EQ(rig.hal.write(kCommand.data(), static_cast<unsigned>(kCommand.size())), 6);
    rig.hub.queue(cargo(20, 3));
    // The write met the queued cargo: only as many bytes as were written
    // come back, as in CEVA's SPI_WRITE completion.
    const int len = rig.hal.read(buf.data(), buf.size(), &t);
    EXPECT_EQ(len, 6);
    EXPECT_EQ(buf[0], 20);
    EXPECT_EQ(buf[2], 3);
}

TEST(SpiHal, SecondWriteWaitsForTheFirst)
{
    Rig rig;
    rig.hub.boots = false;
    rig.hub.answers_wake = false;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    ASSERT_EQ(rig.hal.write(kCommand.data(), static_cast<unsigned>(kCommand.size())), 6);
    EXPECT_EQ(rig.hal.write(kCommand.data(), static_cast<unsigned>(kCommand.size())), 0);
}

TEST(SpiHal, WriteGivesUpWhenHubNeverAnswersWake)
{
    Rig rig;
    rig.hub.boots = false;
    rig.hub.answers_wake = false;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    ASSERT_EQ(rig.hal.write(kCommand.data(), static_cast<unsigned>(kCommand.size())), 6);
    rig.hub.log.clear();
    rig.hub.now += ping::shtp::kWakeTimeoutUs;
    EXPECT_EQ(rig.hal.write(kCommand.data(), static_cast<unsigned>(kCommand.size())), ping::shtp::kSh2ErrTimeout);
    const std::vector<std::string> expected{"wake+"};
    EXPECT_EQ(rig.hub.log, expected);
    EXPECT_EQ(rig.hal.counters().wake_timeouts, 1U);
    // The next write starts afresh.
    EXPECT_EQ(rig.hal.write(kCommand.data(), static_cast<unsigned>(kCommand.size())), 6);
}

TEST(SpiHal, WriteRejectsOversizedFrames)
{
    Rig rig;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    std::array<std::uint8_t, ping::shtp::kMaxTransferOut + 1> big{};
    EXPECT_EQ(rig.hal.write(big.data(), big.size()), ping::shtp::kSh2ErrBadParam);
}

TEST(SpiHal, TooSmallBufferDiscardsTheCargo)
{
    Rig rig;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    std::array<std::uint8_t, 8> buf{};
    std::uint32_t t = 0;
    EXPECT_EQ(rig.hal.read(buf.data(), buf.size(), &t), ping::shtp::kSh2ErrBadParam);
    std::array<std::uint8_t, 64> big{};
    EXPECT_EQ(rig.hal.read(big.data(), big.size(), &t), 0);
}

TEST(SpiHal, OversizedLengthIsTruncatedToTheBuffer)
{
    Rig rig;
    rig.hub.boots = false;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    rig.hub.queue(cargo(2000, 3));
    std::array<std::uint8_t, 1024> buf{};
    std::uint32_t t = 0;
    EXPECT_EQ(rig.hal.read(buf.data(), buf.size(), &t), 1024);
}

TEST(SpiHal, BusFailureIsCountedAndRecovered)
{
    Rig rig;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    rig.hub.fail_transfers = true;
    std::array<std::uint8_t, 64> buf{};
    std::uint32_t t = 0;
    EXPECT_EQ(rig.hal.read(buf.data(), buf.size(), &t), 0);
    EXPECT_EQ(rig.hal.counters().transfers_failed, 1U);
    EXPECT_EQ(rig.hub.log.back(), "cs+");

    rig.hub.fail_transfers = false;
    rig.hal.on_intn(0);
    EXPECT_EQ(rig.hal.read(buf.data(), buf.size(), &t), 16);
}

TEST(SpiHal, DeadlineAbortsWithAResetCompleteCargo)
{
    Rig rig;
    rig.hub.boots = false;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    rig.hal.arm_deadline(1'000);
    std::array<std::uint8_t, 64> buf{};
    std::uint32_t t = 0;
    EXPECT_EQ(rig.hal.read(buf.data(), buf.size(), &t), 0);
    EXPECT_FALSE(rig.hal.aborted());
    rig.hub.now += 1'000;
    ASSERT_EQ(rig.hal.read(buf.data(), buf.size(), &t), 5);
    EXPECT_TRUE(rig.hal.aborted());
    const std::array<std::uint8_t, 5> expected{5, 0, 1, 0, 1};
    EXPECT_TRUE(std::equal(expected.begin(), expected.end(), buf.begin()));
    rig.hal.disarm_deadline();
    EXPECT_FALSE(rig.hal.aborted());
}

TEST(SpiHal, CloseHoldsHubInReset)
{
    Rig rig;
    ASSERT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
    rig.hub.log.clear();
    rig.hal.close();
    const std::vector<std::string> expected{"ints-", "rst-", "cs+", "deinit"};
    EXPECT_EQ(rig.hub.log, expected);
    EXPECT_FALSE(rig.hal.is_open());
    EXPECT_EQ(rig.hal.open(), ping::shtp::kSh2Ok);
}

} // namespace
