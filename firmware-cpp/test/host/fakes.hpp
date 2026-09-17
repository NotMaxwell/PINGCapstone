/// @file
/// Test doubles satisfying the @ref ping::hal concepts.
///
/// These are the reason the drivers take their bus as a template parameter:
/// the exact same driver code under test here is what runs on the chip, with
/// no `#ifdef` separating the two.

#pragma once

#include "ping/hal/bus.hpp"

#include <algorithm>
#include <cstdint>
#include <deque>
#include <span>
#include <vector>

namespace ping::test {

/// An I²C bus that replays a script of canned reads and records every write.
///
/// Each queued response answers exactly one `read` call, truncated or
/// zero-padded to whatever length the driver asked for — which is how the
/// real hub behaves when you read fewer bytes than it has waiting.
class ScriptedI2c {
  public:
    /// Queues the bytes the next `read` should return.
    void queue_read(std::vector<std::uint8_t> bytes) { reads_.push_back(std::move(bytes)); }

    /// Makes the next `read` or `write` fail with `code` instead.
    void fail_next(std::int32_t code) { fail_ = hal::BusError{.code = code}; }

    hal::BusResult read(std::uint8_t address, std::span<std::uint8_t> into)
    {
        last_address = address;
        if (fail_) {
            const auto error = *fail_;
            fail_.reset();
            return std::unexpected(error);
        }

        std::fill(into.begin(), into.end(), std::uint8_t{0});
        if (reads_.empty()) {
            // A hub with nothing to say clocks out zeros.
            ++starved_reads;
            return {};
        }

        const auto& source = reads_.front();
        const std::size_t n = std::min(source.size(), into.size());
        std::copy_n(source.begin(), n, into.begin());
        read_lengths.push_back(into.size());
        reads_.pop_front();
        return {};
    }

    hal::BusResult write(std::uint8_t address, std::span<const std::uint8_t> from)
    {
        last_address = address;
        if (fail_) {
            const auto error = *fail_;
            fail_.reset();
            return std::unexpected(error);
        }
        writes.emplace_back(from.begin(), from.end());
        return {};
    }

    hal::BusResult write_read(std::uint8_t address, std::span<const std::uint8_t> from, std::span<std::uint8_t> into)
    {
        if (auto sent = write(address, from); !sent) {
            return sent;
        }
        return read(address, into);
    }

    std::vector<std::vector<std::uint8_t>> writes;
    std::vector<std::size_t> read_lengths;
    std::uint8_t last_address = 0;
    int starved_reads = 0;

  private:
    std::deque<std::vector<std::uint8_t>> reads_;
    std::optional<hal::BusError> fail_;
};

/// A clock that never actually sleeps, so a driver's timeout loop runs at
/// full speed in the test suite.
class FakeClock {
  public:
    void delay_ms(std::uint32_t ms)
    {
        slept_ms += ms;
        now_us_ += static_cast<std::int64_t>(ms) * 1000;
    }

    std::int64_t now_us() { return now_us_; }

    std::uint32_t slept_ms = 0;

  private:
    std::int64_t now_us_ = 0;
};

/// A UART that hands back a fixed buffer, a slice at a time.
class ScriptedUart {
  public:
    void feed(std::span<const std::uint8_t> bytes) { rx_.insert(rx_.end(), bytes.begin(), bytes.end()); }

    std::expected<std::size_t, hal::BusError> read(std::span<std::uint8_t> into, std::uint32_t /*timeout_ms*/)
    {
        const std::size_t n = std::min(into.size(), rx_.size() - cursor_);
        std::copy_n(rx_.begin() + static_cast<std::ptrdiff_t>(cursor_), n, into.begin());
        cursor_ += n;
        return n;
    }

    hal::BusResult write(std::span<const std::uint8_t> from)
    {
        writes.emplace_back(from.begin(), from.end());
        return {};
    }

    void flush_input()
    {
        rx_.clear();
        cursor_ = 0;
    }

    std::vector<std::vector<std::uint8_t>> writes;

  private:
    std::vector<std::uint8_t> rx_;
    std::size_t cursor_ = 0;
};

static_assert(hal::I2cBus<ScriptedI2c>);
static_assert(hal::Clock<FakeClock>);
static_assert(hal::ByteStream<ScriptedUart>);

} // namespace ping::test
