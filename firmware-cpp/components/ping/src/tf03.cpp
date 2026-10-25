#include "ping/tf03.hpp"

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

#include <algorithm>
#include <variant>

namespace ping::tf03 {
namespace {

// Command IDs.
constexpr std::uint8_t kCommandFirmwareVersion = 0x01;
constexpr std::uint8_t kCommandSystemReset = 0x02;
constexpr std::uint8_t kCommandFrameRate = 0x03;
constexpr std::uint8_t kCommandTrigger = 0x04;
constexpr std::uint8_t kCommandBaudRate = 0x06;
constexpr std::uint8_t kCommandOutputEnable = 0x07;
constexpr std::uint8_t kCommandRestoreDefaults = 0x10;
constexpr std::uint8_t kCommandSaveSettings = 0x11;
constexpr std::uint8_t kCommandOutOfRange = 0x4F;

/// UART baud rates the TF03 accepts.
///
/// The two editions of the manual disagree at the fast end, so only the rates
/// both list are allowed. An unsupported rate is not refused by the TF03; it
/// silently falls back to 115200 and the link is lost.
constexpr std::array<std::uint32_t, 15> kSupportedBaudRates{
    9'600,   14'400,  19'200,  38'400,  56'000,  57'600,  115'200,   128'000,
    230'400, 256'000, 460'800, 512'000, 750'000, 921'600, 1'000'000,
};

/// Room the ESP-IDF driver keeps for received bytes between reads.
constexpr int kRxRingBuffer = 2048;

std::unexpected<Error> fail(Error::Kind kind) { return std::unexpected(Error{.kind = kind}); }

/// Whether `hz` is of the form `n × 10^k` with `n` in 1..=9 and `k` in 0..=3.
bool is_supported_frame_rate(std::uint16_t hz)
{
    return std::ranges::any_of(std::array{1, 10, 100, 1000}, [hz](int scale) {
        return hz % scale == 0 && hz / scale >= 1 && hz / scale <= 9;
    });
}

/// For commands the TF03 acknowledges with a single status byte.
std::expected<void, Error> check_status(const Response& response)
{
    if (response.payload.len != 1) {
        return fail(Error::Kind::kProtocol);
    }
    if (const std::uint8_t status = response.payload.bytes[0]; status != 0) {
        return std::unexpected(Error{.kind = Error::Kind::kRejected, .status = status});
    }
    return {};
}

/// For commands the TF03 acknowledges by sending the command back.
std::expected<void, Error> check_echo(const Command& command, const Response& response)
{
    if (response.payload == command.payload()) {
        return {};
    }
    return fail(Error::Kind::kProtocol);
}

std::array<std::uint8_t, 2> le16(std::uint16_t value)
{
    return {static_cast<std::uint8_t>(value), static_cast<std::uint8_t>(value >> 8)};
}

} // namespace

const char* to_string(Heard heard)
{
    switch (heard) {
    case Heard::kNothing:
        return "nothing";
    case Heard::kNoise:
        return "noise";
    case Heard::kOwnCommand:
        return "own command";
    case Heard::kMeasurements:
        return "measurements";
    }
    return "?";
}

const char* to_string(const Error& error)
{
    switch (error.kind) {
    case Error::Kind::kUart:
        return esp_err_to_name(error.esp);
    case Error::Kind::kTimeout:
        return "timeout";
    case Error::Kind::kNoReply:
        switch (error.heard) {
        case Heard::kNothing:
            return "no reply, heard nothing";
        case Heard::kNoise:
            return "no reply, heard noise";
        case Heard::kOwnCommand:
            return "no reply, heard own command";
        case Heard::kMeasurements:
            return "no reply, heard measurements";
        }
        return "no reply";
    case Error::Kind::kRejected:
        return "rejected";
    case Error::Kind::kProtocol:
        return "protocol";
    case Error::Kind::kUnsupported:
        return "unsupported";
    }
    return "?";
}

esp_err_t Tf03::begin(gpio_num_t tx, gpio_num_t rx, int baud)
{
    uart_config_t config{};
    config.baud_rate = baud;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;
    if (esp_err_t err = uart_driver_install(port_, kRxRingBuffer, 0, 0, nullptr, 0); err != ESP_OK) {
        return err;
    }
    if (esp_err_t err = uart_param_config(port_, &config); err != ESP_OK) {
        return err;
    }
    return uart_set_pin(port_, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

Measurement Tf03::measurement(const DataFrame& frame) const
{
    const bool usable = frame.distance_cm < out_of_range_cm_ && frame.strength >= kMinReliableStrength;
    return Measurement{
        .distance_cm = usable ? std::optional{frame.distance_cm} : std::nullopt,
        .raw_distance_cm = frame.distance_cm,
        .strength = frame.strength,
    };
}

std::expected<Measurement, Error> Tf03::read(std::uint32_t timeout_ms)
{
    const std::int64_t deadline = esp_timer_get_time() + static_cast<std::int64_t>(timeout_ms) * 1000;
    for (;;) {
        auto packet = read_packet(deadline);
        if (!packet) {
            return std::unexpected(packet.error());
        }
        if (const auto* frame = std::get_if<DataFrame>(&*packet)) {
            return measurement(*frame);
        }
    }
}

std::expected<Packet, Error> Tf03::read_packet(std::int64_t deadline_us)
{
    for (;;) {
        while (rx_start_ < rx_end_) {
            if (auto packet = parser_.push(rx_buf_[rx_start_++])) {
                Heard heard = Heard::kMeasurements;
                if (const auto* response = std::get_if<Response>(&*packet)) {
                    heard = is_own_command(*response) ? Heard::kOwnCommand : Heard::kNoise;
                }
                heard_ = std::max(heard_, heard);
                return *packet;
            }
        }

        const std::int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) {
            return fail(Error::Kind::kTimeout);
        }

        // Wait for one byte, then take whatever else has already arrived, so
        // a read never sits waiting to fill the whole buffer.
        const auto wait = static_cast<TickType_t>(pdMS_TO_TICKS(remaining_us / 1000) + 1);
        int count = uart_read_bytes(port_, rx_buf_.data(), 1, wait);
        if (count > 0) {
            std::size_t buffered = 0;
            uart_get_buffered_data_len(port_, &buffered);
            buffered = std::min(buffered, rx_buf_.size() - 1);
            if (buffered > 0) {
                const int more = uart_read_bytes(port_, rx_buf_.data() + 1, buffered, 0);
                count += std::max(more, 0);
            }
        }

        if (count < 0) {
            // Bytes were probably lost, so whatever the parser holds would be
            // spliced onto an unrelated later packet.
            parser_.reset();
            return std::unexpected(Error{.kind = Error::Kind::kUart, .esp = ESP_FAIL});
        }
        if (count > 0) {
            heard_ = std::max(heard_, Heard::kNoise);
        }
        rx_start_ = 0;
        rx_end_ = static_cast<std::size_t>(count);
    }
}

bool Tf03::is_own_command(const Response& response) const
{
    return sent_ && response.id == sent_->id() && response.payload == sent_->payload();
}

std::expected<void, Error> Tf03::send(const Command& command)
{
    sent_ = command;
    heard_ = Heard::kNothing;
    const auto bytes = command.bytes();
    if (uart_write_bytes(port_, bytes.data(), bytes.size()) < 0) {
        return std::unexpected(Error{.kind = Error::Kind::kUart, .esp = ESP_FAIL});
    }
    if (esp_err_t err = uart_wait_tx_done(port_, pdMS_TO_TICKS(kResponseTimeoutMs)); err != ESP_OK) {
        return std::unexpected(Error{.kind = Error::Kind::kUart, .esp = err});
    }
    return {};
}

/// Sends a command and waits for the TF03's answer to it, skipping data
/// frames and answers to anything else.
///
/// A copy of the command itself is skipped too unless `echoed` says the TF03
/// answers by sending the command back. For those commands an echo on the
/// wire cannot be told from a real answer.
std::expected<Response, Error> Tf03::transact(const Command& command, bool echoed)
{
    if (auto sent = send(command); !sent) {
        return std::unexpected(sent.error());
    }

    const std::int64_t deadline = esp_timer_get_time() + static_cast<std::int64_t>(kResponseTimeoutMs) * 1000;
    for (;;) {
        auto packet = read_packet(deadline);
        if (!packet) {
            if (packet.error().kind == Error::Kind::kTimeout) {
                return std::unexpected(Error{.kind = Error::Kind::kNoReply, .heard = heard_});
            }
            // A UART error is skipped: the answer may still come through on
            // the reads that follow, and the deadline bounds the wait.
            continue;
        }
        if (const auto* response = std::get_if<Response>(&*packet);
            response && response->id == command.id() && (echoed || !is_own_command(*response))) {
            return *response;
        }
    }
}

std::expected<FirmwareVersion, Error> Tf03::firmware_version()
{
    auto response = transact(Command{kCommandFirmwareVersion}, false);
    if (!response) {
        return std::unexpected(response.error());
    }
    if (response->payload.len != 3) {
        return fail(Error::Kind::kProtocol);
    }
    const auto& b = response->payload.bytes;
    return FirmwareVersion{.major = b[2], .minor = b[1], .patch = b[0]};
}

std::expected<void, Error> Tf03::system_reset()
{
    return transact(Command{kCommandSystemReset}, false).and_then(check_status);
}

std::expected<void, Error> Tf03::set_frame_rate(std::uint16_t hz)
{
    if (!is_supported_frame_rate(hz)) {
        return fail(Error::Kind::kUnsupported);
    }
    const Command command{kCommandFrameRate, le16(hz)};
    return transact(command, true).and_then([&](const Response& r) { return check_echo(command, r); });
}

std::expected<void, Error> Tf03::set_output_enabled(bool enabled)
{
    const std::array<std::uint8_t, 1> payload{static_cast<std::uint8_t>(enabled)};
    const Command command{kCommandOutputEnable, payload};
    return transact(command, true).and_then([&](const Response& r) { return check_echo(command, r); });
}

std::expected<Measurement, Error> Tf03::trigger()
{
    if (auto sent = send(Command{kCommandTrigger}); !sent) {
        return std::unexpected(sent.error());
    }
    auto reading = read(kResponseTimeoutMs);
    if (!reading && reading.error().kind == Error::Kind::kTimeout) {
        return std::unexpected(Error{.kind = Error::Kind::kNoReply, .heard = heard_});
    }
    return reading;
}

std::expected<void, Error> Tf03::set_baud_rate(std::uint32_t baud)
{
    if (std::ranges::find(kSupportedBaudRates, baud) == kSupportedBaudRates.end()) {
        return fail(Error::Kind::kUnsupported);
    }
    const std::array<std::uint8_t, 4> payload{
        static_cast<std::uint8_t>(baud),
        static_cast<std::uint8_t>(baud >> 8),
        static_cast<std::uint8_t>(baud >> 16),
        static_cast<std::uint8_t>(baud >> 24),
    };
    const Command command{kCommandBaudRate, payload};
    return transact(command, true).and_then([&](const Response& r) { return check_echo(command, r); });
}

std::expected<void, Error> Tf03::set_out_of_range_threshold(std::uint16_t threshold_cm)
{
    auto done = transact(Command{kCommandOutOfRange, le16(threshold_cm)}, false).and_then(check_status);
    if (done) {
        out_of_range_cm_ = threshold_cm;
    }
    return done;
}

std::expected<void, Error> Tf03::save_settings()
{
    return transact(Command{kCommandSaveSettings}, false).and_then(check_status);
}

std::expected<void, Error> Tf03::restore_defaults()
{
    auto done = transact(Command{kCommandRestoreDefaults}, false).and_then(check_status);
    if (done) {
        out_of_range_cm_ = kDefaultOutOfRangeCm;
    }
    return done;
}

} // namespace ping::tf03
