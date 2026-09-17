/// @file
/// Driver for the CEVA/Hillcrest BNO085 nine axis sensor hub over I²C.
///
/// The BNO085 is not a register mapped IMU. It runs SH-2 sensor fusion
/// firmware on its own microcontroller and speaks a packet protocol: you ask
/// it to start producing a *report* at some rate, and it pushes reports at you
/// until told otherwise. This header implements that protocol on top of any
/// type satisfying @ref ping::hal::I2cBus.
///
/// # Interrupt pin
///
/// The hub drives its `INT` line low when it has a packet waiting. Reading
/// when nothing is pending is harmless — @ref Bno085::read_event returns an
/// empty optional — but polling wastes bus bandwidth and, on some boards,
/// reads while the hub is asleep are unreliable. Wire `INT` up and wait on a
/// falling edge before each read.
///
/// # Blocking, not async
///
/// The Rust original is `async`: each transfer is a future, and the IMU task
/// awaits the interrupt pin. Here every call blocks the calling FreeRTOS
/// task instead. ESP-IDF's I²C driver yields while a transaction is in
/// flight and @ref ping::hal::Clock::delay_ms is a `vTaskDelay`, so a
/// blocked IMU task costs the rest of the firmware nothing — the task *is*
/// the future.
///
/// # Example
///
/// @code
/// ping::bno085::Bno085 imu{bus, ping::bno085::Address::kDefault, clock};
/// auto id = imu.init();
/// if (!id) { /* ... */ }
/// imu.enable_report(SensorId::kRotationVector, 10'000); // 100 Hz
///
/// for (;;) {
///     int_line.wait(portMAX_DELAY);
///     while (auto event = imu.read_event()) {
///         if (!*event) break;                       // nothing left waiting
///         if (auto* rv = std::get_if<RotationVectorData>(&(*event)->data)) {
///             // ...
///         }
///     }
/// }
/// @endcode

#pragma once

#include "ping/bno085/reports.hpp"
#include "ping/bno085/shtp.hpp"
#include "ping/hal/bus.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <optional>
#include <span>

namespace ping::bno085 {

/// Largest cargo the driver will buffer.
///
/// A packet holds a timestamp report and however many sensor reports were due
/// at once, so tens of bytes in normal use and a few hundred only under
/// aggressive batching. The reset advertisement is the one genuinely large
/// packet the hub sends, and that is discarded rather than parsed.
inline constexpr std::size_t kMaxCargo = 256;

/// Largest single I²C transfer the driver will attempt.
///
/// Cargos longer than this are read in several transactions, each of which
/// the hub prefixes with a fresh four byte header.
inline constexpr std::size_t kMaxTransfer = 64;

/// Enough for the longest command this driver sends (set feature, 17 bytes)
/// plus its SHTP header.
inline constexpr std::size_t kMaxWrite = 32;

/// @name Control channel report IDs
/// @{
inline constexpr std::uint8_t kReportCommandRequest = 0xF2;
inline constexpr std::uint8_t kReportProductIdResponse = 0xF8;
inline constexpr std::uint8_t kReportProductIdRequest = 0xF9;
inline constexpr std::uint8_t kReportTimestampRebase = 0xFA;
inline constexpr std::uint8_t kReportBaseTimestamp = 0xFB;
inline constexpr std::uint8_t kReportSetFeatureCommand = 0xFD;
/// @}

/// Both the request we send on the executable channel and the notification
/// the hub sends back once it has rebooted.
inline constexpr std::uint8_t kExecutableReset = 0x01;

/// @name Commands carried by a command request report
/// @{
inline constexpr std::uint8_t kCommandTare = 0x03;
inline constexpr std::uint8_t kCommandSaveDcd = 0x06;
inline constexpr std::uint8_t kCommandMeCalibrate = 0x07;
/// @}

inline constexpr std::uint8_t kTareNow = 0x00;
inline constexpr std::uint8_t kTarePersist = 0x01;

/// Tare about the X axis.
inline constexpr std::uint8_t kTareAxisX = 1 << 0;
/// Tare about the Y axis.
inline constexpr std::uint8_t kTareAxisY = 1 << 1;
/// Tare about the Z axis.
inline constexpr std::uint8_t kTareAxisZ = 1 << 2;
/// Tare about all three axes.
inline constexpr std::uint8_t kTareAxisAll = kTareAxisX | kTareAxisY | kTareAxisZ;

/// How long the hub takes to come up after a reset.
inline constexpr std::uint32_t kBootDelayMs = 100;
/// How long to wait between polls when the hub has nothing to say.
inline constexpr std::uint32_t kPollIntervalMs = 2;
/// How long to keep waiting for an expected packet before giving up.
inline constexpr std::uint32_t kResponseTimeoutMs = 1'000;

/// I²C address, selected on the board by the ADR/SA0 pin.
enum class Address : std::uint8_t {
    /// ADR pulled low, the usual case.
    kDefault = 0x4A,
    /// ADR pulled high.
    kAlternate = 0x4B,
};

/// What the driver can fail at.
///
/// The Rust original is an enum with payloads; this is the same thing with
/// the payload fields sitting alongside a discriminant, which keeps the type
/// trivially copyable and cheap to return by value.
struct Error {
    enum class Kind : std::uint8_t {
        /// The underlying I²C bus returned an error; see @ref bus.
        kI2c,
        /// A cargo arrived that is larger than @ref kMaxCargo. It has been
        /// read and discarded so the bus stays in sync; nothing else is
        /// wrong. See @ref cargo_len.
        kCargoTooLarge,
        /// The hub did not send an expected packet in time.
        kTimeout,
        /// A packet arrived that does not fit the protocol.
        kProtocol,
    };

    Kind kind;
    /// Meaningful when `kind == Kind::kI2c`.
    hal::BusError bus{};
    /// Meaningful when `kind == Kind::kCargoTooLarge`.
    std::uint16_t cargo_len{};

    static constexpr Error i2c(hal::BusError error) noexcept { return {.kind = Kind::kI2c, .bus = error}; }
    static constexpr Error cargo_too_large(std::uint16_t len) noexcept
    {
        return {.kind = Kind::kCargoTooLarge, .cargo_len = len};
    }
    static constexpr Error timeout() noexcept { return {.kind = Kind::kTimeout}; }
    static constexpr Error protocol() noexcept { return {.kind = Kind::kProtocol}; }

    friend bool operator==(const Error&, const Error&) = default;
};

/// A human readable name for an error, for logging.
constexpr const char* to_string(Error::Kind kind) noexcept
{
    switch (kind) {
    case Error::Kind::kI2c:
        return "i2c";
    case Error::Kind::kCargoTooLarge:
        return "cargo-too-large";
    case Error::Kind::kTimeout:
        return "timeout";
    case Error::Kind::kProtocol:
        return "protocol";
    }
    return "unknown";
}

/// Which reset the hub is reporting in its product ID response.
enum class ResetCause : std::uint8_t {
    kNotApplicable = 0,
    kPowerOn = 1,
    kInternalSystemReset = 2,
    kWatchdogTimeout = 3,
    kExternalReset = 4,
    kOther = 5,
};

constexpr ResetCause reset_cause_from(std::uint8_t value) noexcept
{
    return value <= 4 ? static_cast<ResetCause>(value) : ResetCause::kOther;
}

constexpr const char* to_string(ResetCause cause) noexcept
{
    switch (cause) {
    case ResetCause::kNotApplicable:
        return "n/a";
    case ResetCause::kPowerOn:
        return "power-on";
    case ResetCause::kInternalSystemReset:
        return "internal";
    case ResetCause::kWatchdogTimeout:
        return "watchdog";
    case ResetCause::kExternalReset:
        return "external";
    case ResetCause::kOther:
        return "other";
    }
    return "unknown";
}

/// The hub's identification, returned by @ref Bno085::product_id.
struct ProductId {
    ResetCause reset_cause;
    std::uint8_t sw_version_major;
    std::uint8_t sw_version_minor;
    std::uint16_t sw_version_patch;
    std::uint32_t sw_part_number;
    std::uint32_t sw_build_number;
};

/// Which orientation output a tare should be applied to.
enum class TareBasis : std::uint8_t {
    kRotationVector = 0,
    kGameRotationVector = 1,
    kGeomagneticRotationVector = 2,
    kGyroIntegratedRotationVector = 3,
    kArvrStabilizedRotationVector = 4,
    kArvrStabilizedGameRotationVector = 5,
};

/// Everything the hub needs to know to start producing a report.
///
/// @ref ReportConfig::periodic covers the common case; the remaining fields
/// matter only for change-sensitivity filtering, batching and wake behaviour.
struct ReportConfig {
    SensorId sensor;
    /// How often to report, in microseconds. Zero disables the sensor.
    std::uint32_t interval_us;
    /// How long the hub may batch reports before delivering them, in
    /// microseconds. Zero delivers each report as it is produced.
    std::uint32_t batch_interval_us = 0;
    /// Threshold below which a change is not worth reporting, in the sensor's
    /// own units.
    std::uint16_t change_sensitivity = 0;
    /// Treat @ref change_sensitivity as a fraction of the last value rather
    /// than an absolute amount.
    bool change_sensitivity_relative = false;
    /// Apply @ref change_sensitivity at all.
    bool change_sensitivity_enabled = false;
    /// Let this sensor wake the host.
    bool wake_enabled = false;
    /// Keep this sensor running even when the hub would otherwise sleep.
    bool always_on_enabled = false;
    /// Sensor specific configuration word; zero for all the motion sensors.
    std::uint32_t sensor_specific = 0;

    /// A plain periodic report at `interval_us`, with no filtering or
    /// batching.
    static constexpr ReportConfig periodic(SensorId sensor, std::uint32_t interval_us) noexcept
    {
        return ReportConfig{.sensor = sensor, .interval_us = interval_us};
    }

    [[nodiscard]] constexpr std::uint8_t flags() const noexcept
    {
        std::uint8_t bits = 0;
        if (change_sensitivity_relative) {
            bits |= 1 << 0;
        }
        if (change_sensitivity_enabled) {
            bits |= 1 << 1;
        }
        if (wake_enabled) {
            bits |= 1 << 2;
        }
        if (always_on_enabled) {
            bits |= 1 << 3;
        }
        return bits;
    }
};

/// A BNO085 on an I²C bus.
///
/// @tparam Bus   anything satisfying @ref ping::hal::I2cBus — the real
///               peripheral on the chip, a recording fake in the host tests.
/// @tparam Timer anything satisfying @ref ping::hal::Clock.
template <hal::I2cBus Bus, hal::Clock Timer>
class Bno085 {
  public:
    /// Borrows a bus and a clock. Nothing is sent until @ref init is called.
    ///
    /// Both references must outlive the driver; in practice both are owned by
    /// the task that owns the driver, or are file-scope singletons.
    Bno085(Bus& bus, Address address, Timer& clock) noexcept
        : bus_(&bus), clock_(&clock), address_(static_cast<std::uint8_t>(address))
    {
    }

    /// Resets the hub and reads back its identification.
    ///
    /// No reports are enabled by a reset, so follow this with
    /// @ref enable_report for each sensor you want.
    std::expected<ProductId, Error> init()
    {
        if (auto reset = soft_reset(); !reset) {
            return std::unexpected(reset.error());
        }
        return product_id();
    }

    /// Asks the hub to reboot, then waits for it to announce that it has.
    ///
    /// The hub sends an advertisement packet on its way up. That packet is
    /// larger than the driver's buffer and describes only things the driver
    /// already knows, so it is read and discarded.
    std::expected<void, Error> soft_reset()
    {
        const std::array<std::uint8_t, 1> payload{kExecutableReset};
        if (auto sent = write_cargo(shtp::Channel::kExecutable, payload); !sent) {
            return sent;
        }
        clock_->delay_ms(kBootDelayMs);

        // A reset resets the hub's sequence numbers too.
        sequence_.fill(0);
        discard_buffered();

        std::uint32_t waited = 0;
        std::optional<Error> last_error;

        for (;;) {
            auto cargo = read_cargo();
            if (cargo) {
                if (cargo->has_value()) {
                    const shtp::Header& header = **cargo;
                    if (header.is_from(shtp::Channel::kExecutable) &&
                        buf_[shtp::kHeaderLen] == kExecutableReset) {
                        return {};
                    }
                } else {
                    clock_->delay_ms(kPollIntervalMs);
                    waited += kPollIntervalMs;
                }
            } else if (cargo.error().kind == Error::Kind::kCargoTooLarge) {
                // The advertisement is expected to be too large. It carries
                // nothing the driver needs, and reading it kept the bus in
                // sync, so this is not a failure.
            } else {
                // The hub may not answer at all while it is rebooting.
                last_error = cargo.error();
                clock_->delay_ms(kPollIntervalMs);
                waited += kPollIntervalMs;
            }

            if (waited >= kResponseTimeoutMs) {
                return std::unexpected(last_error.value_or(Error::timeout()));
            }
        }
    }

    /// Asks the hub what it is and which reset it last went through.
    std::expected<ProductId, Error> product_id()
    {
        const std::array<std::uint8_t, 2> request{kReportProductIdRequest, 0};
        if (auto sent = write_cargo(shtp::Channel::kControl, request); !sent) {
            return std::unexpected(sent.error());
        }

        auto len = await_control_report(kReportProductIdResponse, 16);
        if (!len) {
            return std::unexpected(len.error());
        }

        const std::uint8_t* payload = &buf_[shtp::kHeaderLen];
        return ProductId{
            .reset_cause = reset_cause_from(payload[1]),
            .sw_version_major = payload[2],
            .sw_version_minor = payload[3],
            .sw_version_patch = static_cast<std::uint16_t>(payload[12] | (payload[13] << 8)),
            .sw_part_number = le32(payload + 4),
            .sw_build_number = le32(payload + 8),
        };
    }

    /// Starts a sensor reporting every `interval_us` microseconds.
    ///
    /// The hub rounds the interval to what the sensor can actually do; ask for
    /// 10'000 µs to get roughly 100 Hz. Enabling a sensor that is already
    /// enabled just changes its rate.
    std::expected<void, Error> enable_report(SensorId sensor, std::uint32_t interval_us)
    {
        return set_feature(ReportConfig::periodic(sensor, interval_us));
    }

    /// Stops a sensor reporting.
    std::expected<void, Error> disable_report(SensorId sensor)
    {
        return set_feature(ReportConfig::periodic(sensor, 0));
    }

    /// Configures a report in full, for the cases @ref enable_report does not
    /// cover.
    std::expected<void, Error> set_feature(const ReportConfig& config)
    {
        std::array<std::uint8_t, 17> payload{};
        payload[0] = kReportSetFeatureCommand;
        payload[1] = static_cast<std::uint8_t>(config.sensor);
        payload[2] = config.flags();
        store_le16(&payload[3], config.change_sensitivity);
        store_le32(&payload[5], config.interval_us);
        store_le32(&payload[9], config.batch_interval_us);
        store_le32(&payload[13], config.sensor_specific);

        return write_cargo(shtp::Channel::kControl, payload);
    }

    /// Reads the next sensor report, if one is waiting.
    ///
    /// One packet often carries several reports, so call this in a loop until
    /// it yields an empty optional before going back to waiting on the `INT`
    /// pin. Packets that are not sensor reports are consumed and skipped.
    std::expected<std::optional<Event>, Error> read_event()
    {
        for (;;) {
            if (auto event = next_buffered_event()) {
                return event;
            }

            auto cargo = read_cargo();
            if (!cargo) {
                return std::unexpected(cargo.error());
            }
            if (!cargo->has_value()) {
                return std::optional<Event>{};
            }

            const shtp::Header& header = **cargo;
            cargo_len_ = header.cargo_len;
            cursor_ = shtp::kHeaderLen;
            channel_ = header.channel;
        }
    }

    /// Zeroes the current orientation, so that where the sensor points now
    /// becomes the reference the chosen output reports against.
    ///
    /// The tare is lost on reset unless @ref persist_tare follows.
    std::expected<void, Error> tare(std::uint8_t axes, TareBasis basis)
    {
        return send_command(kCommandTare,
                            {kTareNow, axes, static_cast<std::uint8_t>(basis), 0, 0, 0, 0, 0, 0});
    }

    /// Writes the current tare to flash so it survives a reset.
    std::expected<void, Error> persist_tare()
    {
        return send_command(kCommandTare, {kTarePersist, 0, 0, 0, 0, 0, 0, 0, 0});
    }

    /// Chooses which sensors the hub keeps calibrating as it runs.
    ///
    /// The hub calibrates the accelerometer and magnetometer by default. Gyro
    /// calibration is worth enabling if the device spends time still.
    std::expected<void, Error> configure_calibration(bool accelerometer, bool gyroscope, bool magnetometer)
    {
        return send_command(kCommandMeCalibrate,
                            {
                                static_cast<std::uint8_t>(accelerometer),
                                static_cast<std::uint8_t>(gyroscope),
                                static_cast<std::uint8_t>(magnetometer),
                                0, // subcommand 0: configure
                                0, // planar accelerometer calibration
                                0,
                                0,
                                0,
                                0,
                            });
    }

    /// Saves the running calibration to flash.
    ///
    /// Worth doing once the hub reports @ref Accuracy::kHigh, so the next boot
    /// starts from a good calibration instead of relearning it.
    std::expected<void, Error> save_calibration() { return send_command(kCommandSaveDcd, {}); }

    /// Sends a raw command request. See the SH-2 Reference Manual for the
    /// command numbers and their parameters.
    std::expected<void, Error> send_command(std::uint8_t command, const std::array<std::uint8_t, 9>& params)
    {
        std::array<std::uint8_t, 12> payload{};
        payload[0] = kReportCommandRequest;
        payload[1] = command_sequence_;
        payload[2] = command;
        std::copy(params.begin(), params.end(), payload.begin() + 3);
        ++command_sequence_; // wraps, which is what the protocol expects

        return write_cargo(shtp::Channel::kControl, payload);
    }

  private:
    static constexpr std::uint32_t le32(const std::uint8_t* p) noexcept
    {
        return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
               (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
    }

    static constexpr void store_le16(std::uint8_t* p, std::uint16_t value) noexcept
    {
        p[0] = static_cast<std::uint8_t>(value);
        p[1] = static_cast<std::uint8_t>(value >> 8);
    }

    static constexpr void store_le32(std::uint8_t* p, std::uint32_t value) noexcept
    {
        p[0] = static_cast<std::uint8_t>(value);
        p[1] = static_cast<std::uint8_t>(value >> 8);
        p[2] = static_cast<std::uint8_t>(value >> 16);
        p[3] = static_cast<std::uint8_t>(value >> 24);
    }

    /// Pulls the next report out of the buffered cargo, if there is one left.
    std::optional<Event> next_buffered_event()
    {
        const auto channel = shtp::channel_from(channel_);
        if (!channel) {
            // Advertisements, command responses and the like.
            discard_buffered();
            return std::nullopt;
        }

        switch (*channel) {
        // The gyro-integrated rotation vector gets a channel to itself and
        // fills the whole payload: no report ID, no timestamp, no status.
        case shtp::Channel::kInputGyroRv: {
            if (cursor_ >= cargo_len_) {
                return std::nullopt;
            }
            const std::span<const std::uint8_t> payload{&buf_[shtp::kHeaderLen], cargo_len_ - shtp::kHeaderLen};
            cursor_ = cargo_len_;

            const std::size_t expected = report_len(SensorId::kGyroIntegratedRotationVector);
            if (payload.size() < expected) {
                return std::nullopt;
            }
            return decode_gyro_integrated_rv(payload);
        }

        case shtp::Channel::kInputNormal:
        case shtp::Channel::kInputWake: {
            while (cursor_ < cargo_len_) {
                const std::uint8_t id = buf_[cursor_];
                const std::size_t remaining = cargo_len_ - cursor_;

                // Timestamp reports are not sensor data; they set the
                // reference the reports after them are measured against.
                if (id == kReportBaseTimestamp || id == kReportTimestampRebase) {
                    if (remaining < 5) {
                        break;
                    }
                    const auto ticks = static_cast<std::int32_t>(le32(&buf_[cursor_ + 1]));
                    reference_delta_ = id == kReportBaseTimestamp
                                           ? -ticks
                                           : static_cast<std::int32_t>(
                                                 static_cast<std::uint32_t>(reference_delta_) +
                                                 static_cast<std::uint32_t>(ticks));
                    cursor_ += 5;
                    continue;
                }

                // Without a length for this report there is no way to find
                // where the next one starts, so the rest of the packet has to
                // go.
                const auto sensor = sensor_from(id);
                if (!sensor) {
                    break;
                }
                const std::size_t len = report_len(*sensor);
                if (len > remaining) {
                    break;
                }

                const std::size_t start = cursor_;
                Event event = decode(*sensor, std::span{&buf_[start], len}, reference_delta_);
                cursor_ = start + len;
                return event;
            }

            discard_buffered();
            return std::nullopt;
        }

        default:
            // Advertisements, command responses and the like.
            discard_buffered();
            return std::nullopt;
        }
    }

    void discard_buffered() noexcept
    {
        cargo_len_ = 0;
        cursor_ = 0;
    }

    /// Reads whatever cargo the hub has waiting into @ref buf_.
    ///
    /// Yields an empty optional when the hub has nothing to send. Every read
    /// transaction comes back with a four byte header in front of it, so a
    /// four byte read peeks the length without consuming any payload, and each
    /// chunk after the first has a header to skip.
    std::expected<std::optional<shtp::Header>, Error> read_cargo()
    {
        std::array<std::uint8_t, shtp::kHeaderLen> head{};
        if (auto read = bus_->read(address_, head); !read) {
            return std::unexpected(Error::i2c(read.error()));
        }

        const auto header = shtp::parse_header(std::span<const std::uint8_t, shtp::kHeaderLen>{head});
        if (!header) {
            return std::optional<shtp::Header>{};
        }

        const std::size_t total = header->cargo_len;
        if (total > kMaxCargo) {
            if (auto discarded = discard_cargo(total); !discarded) {
                return std::unexpected(discarded.error());
            }
            return std::unexpected(Error::cargo_too_large(header->cargo_len));
        }

        std::size_t remaining = total;
        std::size_t written = 0;
        bool first = true;

        while (remaining > 0) {
            const std::size_t want =
                first ? std::min(remaining, kMaxTransfer) : std::min(remaining + shtp::kHeaderLen, kMaxTransfer);

            if (auto read = bus_->read(address_, std::span{scratch_.data(), want}); !read) {
                return std::unexpected(Error::i2c(read.error()));
            }

            // The first chunk's header is the cargo's own; later chunks carry
            // a repeat of it that is not part of the payload.
            const std::size_t offset = first ? 0 : shtp::kHeaderLen;
            const std::size_t chunk_len = want - offset;

            std::memcpy(&buf_[written], &scratch_[offset], chunk_len);
            written += chunk_len;
            remaining -= chunk_len;
            first = false;
        }

        return header;
    }

    /// Reads a cargo the driver has no room for and throws it away, so the
    /// hub moves on to the next one.
    std::expected<void, Error> discard_cargo(std::size_t total)
    {
        std::size_t remaining = total;
        bool first = true;

        while (remaining > 0) {
            const std::size_t want =
                first ? std::min(remaining, kMaxTransfer) : std::min(remaining + shtp::kHeaderLen, kMaxTransfer);

            if (auto read = bus_->read(address_, std::span{scratch_.data(), want}); !read) {
                return std::unexpected(Error::i2c(read.error()));
            }

            remaining -= first ? want : want - shtp::kHeaderLen;
            first = false;
        }

        return {};
    }

    /// Waits for a particular report to arrive on the control channel,
    /// discarding anything else that turns up first. Yields the length of the
    /// payload, which is left at `buf_[kHeaderLen..]`.
    std::expected<std::size_t, Error> await_control_report(std::uint8_t report_id, std::size_t min_len)
    {
        std::uint32_t waited = 0;

        for (;;) {
            auto cargo = read_cargo();
            if (cargo) {
                if (cargo->has_value()) {
                    const shtp::Header& header = **cargo;
                    if (header.is_from(shtp::Channel::kControl) && header.payload_len() >= min_len &&
                        buf_[shtp::kHeaderLen] == report_id) {
                        return header.payload_len();
                    }
                } else {
                    clock_->delay_ms(kPollIntervalMs);
                    waited += kPollIntervalMs;
                }
            } else if (cargo.error().kind == Error::Kind::kCargoTooLarge) {
                // Keep waiting; the bus is still in sync.
            } else {
                return std::unexpected(cargo.error());
            }

            if (waited >= kResponseTimeoutMs) {
                return std::unexpected(Error::timeout());
            }
        }
    }

    /// Frames a payload and sends it on `channel`.
    std::expected<void, Error> write_cargo(shtp::Channel channel, std::span<const std::uint8_t> payload)
    {
        // The Rust original asserts this in debug builds. Returning an error
        // instead costs nothing and keeps a release build from walking off
        // the end of `frame` should a future caller pass something longer.
        if (payload.size() + shtp::kHeaderLen > kMaxWrite) {
            return std::unexpected(Error::protocol());
        }

        std::uint8_t& sequence = sequence_[static_cast<std::size_t>(channel)];
        const auto header = shtp::encode_header(payload.size(), channel, sequence);
        ++sequence; // wraps, which is what the protocol expects

        std::array<std::uint8_t, kMaxWrite> frame{};
        std::copy(header.begin(), header.end(), frame.begin());
        std::copy(payload.begin(), payload.end(), frame.begin() + shtp::kHeaderLen);

        if (auto written = bus_->write(address_, std::span{frame.data(), shtp::kHeaderLen + payload.size()});
            !written) {
            return std::unexpected(Error::i2c(written.error()));
        }
        return {};
    }

    Bus* bus_;
    Timer* clock_;
    std::uint8_t address_;

    /// SHTP sequence numbers, one per channel.
    std::array<std::uint8_t, shtp::kChannelCount> sequence_{};
    /// Sequence number for command requests, which count separately.
    std::uint8_t command_sequence_ = 0;
    /// The cargo most recently read, header included.
    std::array<std::uint8_t, kMaxCargo> buf_{};
    /// Landing area for one I²C transfer. Kept here rather than on the stack
    /// so it does not inflate the frame of every call that reads.
    std::array<std::uint8_t, kMaxTransfer> scratch_{};
    /// Length of that cargo, or zero when the buffer holds nothing.
    std::size_t cargo_len_ = 0;
    /// How far through the buffered cargo @ref read_event has walked.
    std::size_t cursor_ = 0;
    /// The channel the buffered cargo arrived on.
    std::uint8_t channel_ = 0;
    /// Timestamp correction from the last base timestamp report, in the hub's
    /// 100 µs ticks. Added to a report's delay to place the sample in time.
    std::int32_t reference_delta_ = 0;
};

} // namespace ping::bno085
