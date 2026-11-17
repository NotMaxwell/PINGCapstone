/// @file
/// Moving SHTP cargos to and from the BNO085 over SPI.
///
/// Every SHTP transfer is a cargo: a four byte header (total length with the
/// continuation flag in bit 15, channel, sequence number) and a payload. CEVA's
/// sh2 library builds and parses cargos; this is the part underneath it that
/// gets them on and off the wire, with the same bus behaviour as the Rust
/// driver's `SpiTransport` (`src/bno085/transport.rs`), which has run on the
/// hardware:
///
/// - Before each write, WAKE (PS0) is pulsed low until the hub answers by
///   pulling INT low, for at most @ref kWakeTimeoutMs.
/// - A hardware reset holds RST low for @ref kResetPulseMs with WAKE and CS
///   high (PS0 high at reset selects SPI), then waits up to @ref kBootTimeoutMs
///   for INT.
/// - Chip select is driven by hand, so a read looks at the header before it
///   knows how many more bytes to clock in while CS stays low.
/// - SPI is full duplex: a write also clocks in whatever cargo the hub had
///   waiting. That cargo is read to its end and stashed for the next read.
/// - A cargo too large for the caller's buffer is clocked in and thrown away,
///   so the hub moves on, and reported.
///
/// Nothing here touches hardware: the bus is the @ref SpiBus interface, which
/// `bno085_hal.cpp` implements on ESP-IDF and the host tests fake.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>

namespace ping::shtp {

inline constexpr std::size_t kHeaderLen = 4;
/// Largest cargo kept. Matches sh2's `SH2_HAL_MAX_TRANSFER_IN`, so unlike the
/// Rust driver (256 bytes) the reset advertisement fits and is passed on;
/// sh2 ignores it.
inline constexpr std::size_t kMaxCargo = 1024;
/// Largest cargo sent. Matches sh2's `SH2_HAL_MAX_TRANSFER_OUT`.
inline constexpr std::size_t kMaxWrite = 128;

inline constexpr std::uint32_t kWakeTimeoutMs = 50;
inline constexpr std::uint32_t kResetPulseMs = 10;
inline constexpr std::uint32_t kBootTimeoutMs = 500;

/// The SPI bus and the BNO085's control lines. Pin levels are `true` for high.
struct SpiBus {
    SpiBus() = default;
    SpiBus(const SpiBus&) = delete;
    SpiBus& operator=(const SpiBus&) = delete;
    virtual ~SpiBus() = default;

    virtual void set_cs(bool high) = 0;
    virtual void set_wake(bool high) = 0;
    virtual void set_reset(bool high) = 0;
    [[nodiscard]] virtual bool int_high() = 0;
    /// Returns true as soon as INT is low, false if it is still high after
    /// `timeout_ms`. A zero timeout only samples the line.
    [[nodiscard]] virtual bool wait_int_low(std::uint32_t timeout_ms) = 0;
    /// Clocks `len` bytes each way with CS left as it is. A null `tx` sends
    /// zeros (an empty header, which the hub ignores); a null `rx` discards
    /// what comes in. Returns false if the bus failed.
    [[nodiscard]] virtual bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t len) = 0;
    virtual void delay_ms(std::uint32_t ms) = 0;
    /// A free-running microsecond counter; it may wrap.
    [[nodiscard]] virtual std::uint32_t now_us() = 0;
};

struct Header {
    /// Total length, header included, without the continuation flag.
    std::uint16_t cargo_len;
    bool continuation;
    std::uint8_t channel;
    std::uint8_t sequence;
};

/// Decodes a header, or returns false when it describes no payload: all zeros
/// (nothing to say), all ones (hub absent or asleep, bus pulled high) or a
/// length no longer than the header.
bool parse_header(std::span<const std::uint8_t, kHeaderLen> bytes, Header& out);

struct Error {
    enum class Kind : std::uint8_t {
        /// The SPI transfer failed.
        kBus,
        /// The hub did not pull INT low in time (wake or boot).
        kTimeout,
        /// A cargo of @ref cargo_len bytes did not fit and was discarded. The
        /// bus is still in step; nothing else is wrong.
        kCargoTooLarge,
        /// The caller asked to send a frame shorter than a header or longer
        /// than @ref kMaxWrite.
        kBadLength,
    };
    Kind kind;
    std::uint16_t cargo_len = 0;
};

const char* to_string(Error::Kind kind);

class SpiTransport {
  public:
    explicit SpiTransport(SpiBus& bus) : bus_(bus) {}

    /// Hardware reset, then waits for the hub to signal it has booted.
    std::expected<void, Error> reset();

    /// Reads the next cargo into `buf`, header included, and returns its
    /// length, or 0 when the hub has nothing to send. If INT is high, waits
    /// up to `wait_ms` for it to fall first.
    std::expected<std::size_t, Error> read(std::span<std::uint8_t> buf, std::uint32_t wait_ms = 0);

    /// Sends one complete cargo, header included. Any cargo clocked in at the
    /// same time is stashed for the next @ref read, replacing one already
    /// stashed, so callers that care should drain the stash first (see
    /// @ref has_stash).
    std::expected<void, Error> write(std::span<const std::uint8_t> frame);

    [[nodiscard]] bool has_stash() const { return stash_len_ > 0; }

  private:
    std::expected<void, Error> wake();
    bool discard(std::size_t count);
    void deselect() { bus_.set_cs(true); }

    SpiBus& bus_;
    std::array<std::uint8_t, kMaxCargo> stash_{};
    std::size_t stash_len_ = 0;
};

} // namespace ping::shtp
