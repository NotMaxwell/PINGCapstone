#include "ping/shtp_spi.hpp"

#include <algorithm>
#include <cstring>

namespace ping::shtp {

namespace {

constexpr std::uint16_t kContinuationBit = 0x8000;

std::unexpected<Error> fail(Error::Kind kind, std::uint16_t cargo_len = 0)
{
    return std::unexpected(Error{.kind = kind, .cargo_len = cargo_len});
}

} // namespace

bool parse_header(std::span<const std::uint8_t, kHeaderLen> bytes, Header& out)
{
    const auto raw = static_cast<std::uint16_t>(bytes[0] | (bytes[1] << 8));
    if (raw == 0x0000 || raw == 0xFFFF) {
        return false;
    }
    const auto cargo_len = static_cast<std::uint16_t>(raw & ~kContinuationBit);
    if (cargo_len <= kHeaderLen) {
        return false;
    }
    out = Header{
        .cargo_len = cargo_len,
        .continuation = (raw & kContinuationBit) != 0,
        .channel = bytes[2],
        .sequence = bytes[3],
    };
    return true;
}

const char* to_string(Error::Kind kind)
{
    switch (kind) {
    case Error::Kind::kBus:
        return "SPI transfer failed";
    case Error::Kind::kTimeout:
        return "no INT from hub";
    case Error::Kind::kCargoTooLarge:
        return "cargo too large, discarded";
    case Error::Kind::kBadLength:
        return "bad frame length";
    }
    return "?";
}

std::expected<void, Error> SpiTransport::reset()
{
    // PS0 (WAKE) must be high as RST rises for the hub to come up in SPI mode.
    bus_.set_wake(true);
    bus_.set_cs(true);
    stash_len_ = 0;

    bus_.set_reset(false);
    bus_.delay_ms(kResetPulseMs);
    bus_.set_reset(true);

    // The hub pulls INT low once it has booted, to send its advertisement.
    if (!bus_.wait_int_low(kBootTimeoutMs)) {
        return fail(Error::Kind::kTimeout);
    }
    return {};
}

std::expected<void, Error> SpiTransport::wake()
{
    bus_.set_wake(false);
    const bool ready = bus_.wait_int_low(kWakeTimeoutMs);
    bus_.set_wake(true);
    if (!ready) {
        return fail(Error::Kind::kTimeout);
    }
    return {};
}

bool SpiTransport::discard(std::size_t count)
{
    return bus_.transfer(nullptr, nullptr, count);
}

std::expected<std::size_t, Error> SpiTransport::read(std::span<std::uint8_t> buf, std::uint32_t wait_ms)
{
    const std::size_t limit = std::min(buf.size(), kMaxCargo);

    if (stash_len_ > 0) {
        const std::size_t len = stash_len_;
        stash_len_ = 0;
        if (len > limit) {
            return fail(Error::Kind::kCargoTooLarge, static_cast<std::uint16_t>(len));
        }
        std::memcpy(buf.data(), stash_.data(), len);
        return len;
    }

    // On SPI the hub only has something to send while INT is low.
    if (bus_.int_high() && (wait_ms == 0 || !bus_.wait_int_low(wait_ms))) {
        return 0;
    }

    bus_.set_cs(false);

    std::array<std::uint8_t, kHeaderLen> head{};
    if (!bus_.transfer(nullptr, head.data(), head.size())) {
        deselect();
        return fail(Error::Kind::kBus);
    }

    Header header{};
    if (!parse_header(head, header)) {
        deselect();
        return 0;
    }

    const std::size_t total = header.cargo_len;
    if (total > limit) {
        const bool ok = discard(total - kHeaderLen);
        deselect();
        if (!ok) {
            return fail(Error::Kind::kBus);
        }
        return fail(Error::Kind::kCargoTooLarge, header.cargo_len);
    }

    std::memcpy(buf.data(), head.data(), kHeaderLen);
    const bool ok = bus_.transfer(nullptr, buf.data() + kHeaderLen, total - kHeaderLen);
    deselect();
    if (!ok) {
        return fail(Error::Kind::kBus);
    }
    return total;
}

std::expected<void, Error> SpiTransport::write(std::span<const std::uint8_t> frame)
{
    if (frame.size() < kHeaderLen || frame.size() > kMaxWrite) {
        return fail(Error::Kind::kBadLength);
    }

    if (auto woke = wake(); !woke) {
        return woke;
    }
    bus_.set_cs(false);

    const std::size_t sent = frame.size();
    std::array<std::uint8_t, kMaxWrite> incoming{};
    if (!bus_.transfer(frame.data(), incoming.data(), sent)) {
        deselect();
        return fail(Error::Kind::kBus);
    }

    // Whatever the hub clocked out while we wrote is the start of a cargo it
    // had waiting. Read the rest of it now, while CS is still low, and keep
    // it for the next read.
    Header header{};
    if (parse_header(std::span<const std::uint8_t, kHeaderLen>(incoming.data(), kHeaderLen), header)) {
        const std::size_t total = header.cargo_len;
        bool ok = true;
        if (total <= kMaxCargo) {
            const std::size_t have = std::min(sent, total);
            std::memcpy(stash_.data(), incoming.data(), have);
            if (total > sent) {
                ok = bus_.transfer(nullptr, stash_.data() + sent, total - sent);
            }
            stash_len_ = ok ? total : 0;
        } else if (total > sent) {
            // Same as the Rust driver: dropped without an error, because the
            // write itself succeeded.
            ok = discard(total - sent);
        }
        if (!ok) {
            deselect();
            return fail(Error::Kind::kBus);
        }
    }

    deselect();
    return {};
}

} // namespace ping::shtp
