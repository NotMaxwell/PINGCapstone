#include "ping/tf03.hpp"

#include <algorithm>
#include <array>
#include <initializer_list>
#include <variant>

#include "esp_check.h"

namespace ping::tf03 {

namespace {

using protocol::Command;
using protocol::DataFrame;
using protocol::Packet;
using protocol::Response;

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

/// UART baud rates the TF03 accepts. The two editions of the manual
/// disagree at the fast end, so only the rates both list are allowed. An
/// unsupported rate is not refused by the TF03; it silently falls back to
/// 115200 and the link is lost.
constexpr std::array<std::uint32_t, 15> kSupportedBaudRates{
    9'600,   14'400,  19'200,  38'400,  56'000,  57'600,  115'200,   128'000,
    230'400, 256'000, 460'800, 512'000, 750'000, 921'600, 1'000'000,
};

/// The manual gives a command no response within one second as failed.
constexpr TickType_t kResponseTimeout = pdMS_TO_TICKS(1000);

/// About a second of frames at 100 Hz.
constexpr int kRxBufferSize = 1024;

/// The UART driver reports data, overflows and line errors here.
constexpr int kEventQueueLen = 16;

constexpr TickType_t kTxTimeout = pdMS_TO_TICKS(100);

/// Whether `hz` is of the form n × 10^k with n in 1..9 and k in 0..3.
constexpr bool is_supported_frame_rate(std::uint16_t hz)
{
    for (const std::uint16_t scale : {1, 10, 100, 1000}) {
        if (hz % scale == 0 && hz / scale >= 1 && hz / scale <= 9) {
            return true;
        }
    }
    return false;
}

static_assert(is_supported_frame_rate(1) && is_supported_frame_rate(100) &&
              is_supported_frame_rate(200) && is_supported_frame_rate(9000));
static_assert(!is_supported_frame_rate(0) && !is_supported_frame_rate(11) &&
              !is_supported_frame_rate(250) && !is_supported_frame_rate(10'000));

/// A little-endian encoding of `value`, the TF03's byte order.
template <typename T>
constexpr std::array<std::uint8_t, sizeof(T)> little_endian(T value)
{
    std::array<std::uint8_t, sizeof(T)> bytes{};
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        bytes[i] = static_cast<std::uint8_t>(value >> (8 * i));
    }
    return bytes;
}

constexpr Error error(Error::Kind kind)
{
    return Error{.kind = kind};
}

/// For commands the TF03 acknowledges with a single status byte.
Result<> check_status(const Response& response)
{
    if (response.payload.len != 1) {
        return std::unexpected(error(Error::Kind::kProtocol));
    }
    if (const std::uint8_t status = response.payload.bytes[0]; status != 0) {
        return std::unexpected(Error{.kind = Error::Kind::kRejected, .status = status});
    }
    return {};
}

/// For commands the TF03 acknowledges by sending the command back.
Result<> check_echo(const Command& command, const Response& response)
{
    if (!std::ranges::equal(command.payload(), response.payload.view())) {
        return std::unexpected(error(Error::Kind::kProtocol));
    }
    return {};
}

}  // namespace

const char* Error::message() const
{
    switch (kind) {
    case Kind::kUart: return "UART error";
    case Kind::kOverflow: return "receive buffer overflowed";
    case Kind::kTimeout: return "timed out";
    case Kind::kRejected: return "rejected by the TF03";
    case Kind::kProtocol: return "unexpected response";
    case Kind::kUnsupported: return "not supported";
    }
    return "?";
}

Tf03::Tf03(const Config& config) : uart_{config.uart}, out_of_range_cm_{config.out_of_range_cm}
{
    uart_config_t uart_config{};
    uart_config.baud_rate = config.baud_rate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(uart_, kRxBufferSize, 0, kEventQueueLen, &events_, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_, config.tx_pin, config.rx_pin, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

Tf03::~Tf03()
{
    uart_driver_delete(uart_);
}

Result<Measurement> Tf03::read(TickType_t timeout)
{
    const TickType_t start = xTaskGetTickCount();
    for (;;) {
        TickType_t remaining = portMAX_DELAY;
        if (timeout != portMAX_DELAY) {
            const TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout) {
                return std::unexpected(error(Error::Kind::kTimeout));
            }
            remaining = timeout - elapsed;
        }
        auto packet = read_packet(remaining);
        if (!packet) {
            return std::unexpected(packet.error());
        }
        if (const auto* frame = std::get_if<DataFrame>(&*packet)) {
            return measurement(*frame);
        }
    }
}

Result<FirmwareVersion> Tf03::firmware_version()
{
    auto response = transact(Command{kCommandFirmwareVersion, {}});
    if (!response) {
        return std::unexpected(response.error());
    }
    const auto payload = response->payload.view();
    if (payload.size() != 3) {
        return std::unexpected(error(Error::Kind::kProtocol));
    }
    return FirmwareVersion{.major = payload[2], .minor = payload[1], .patch = payload[0]};
}

Result<> Tf03::system_reset()
{
    return transact(Command{kCommandSystemReset, {}}).and_then(check_status);
}

Result<> Tf03::set_frame_rate(std::uint16_t hz)
{
    if (!is_supported_frame_rate(hz)) {
        return std::unexpected(error(Error::Kind::kUnsupported));
    }
    const Command command{kCommandFrameRate, little_endian(hz)};
    return transact(command).and_then(
        [&](const Response& response) { return check_echo(command, response); });
}

Result<> Tf03::set_output_enabled(bool enabled)
{
    const Command command{kCommandOutputEnable,
                          std::array<std::uint8_t, 1>{static_cast<std::uint8_t>(enabled)}};
    return transact(command).and_then(
        [&](const Response& response) { return check_echo(command, response); });
}

Result<Measurement> Tf03::trigger()
{
    return send(Command{kCommandTrigger, {}}).and_then([&] { return read(kResponseTimeout); });
}

Result<> Tf03::set_baud_rate(std::uint32_t baud)
{
    if (!std::ranges::contains(kSupportedBaudRates, baud)) {
        return std::unexpected(error(Error::Kind::kUnsupported));
    }
    const Command command{kCommandBaudRate, little_endian(baud)};
    return transact(command).and_then(
        [&](const Response& response) { return check_echo(command, response); });
}

Result<> Tf03::set_host_baud_rate(std::uint32_t baud)
{
    if (uart_set_baudrate(uart_, baud) != ESP_OK) {
        return std::unexpected(error(Error::Kind::kUart));
    }
    return {};
}

Result<> Tf03::set_out_of_range_threshold(std::uint16_t threshold_cm)
{
    return transact(Command{kCommandOutOfRange, little_endian(threshold_cm)})
        .and_then(check_status)
        .transform([&] { out_of_range_cm_ = threshold_cm; });
}

Result<> Tf03::save_settings()
{
    return transact(Command{kCommandSaveSettings, {}}).and_then(check_status);
}

Result<> Tf03::restore_defaults()
{
    return transact(Command{kCommandRestoreDefaults, {}})
        .and_then(check_status)
        .transform([&] { out_of_range_cm_ = kDefaultOutOfRangeCm; });
}

Result<Packet> Tf03::read_packet(TickType_t timeout)
{
    const TickType_t start = xTaskGetTickCount();
    for (;;) {
        // Hand the parser whatever the last read brought; a packet may end
        // part way through, and the rest waits here for the next call.
        while (rx_start_ < rx_end_) {
            if (auto packet = parser_.push(rx_buf_[rx_start_++])) {
                return *packet;
            }
        }

        if (overflowed()) {
            // Bytes were lost, so whatever the parser holds would be spliced
            // onto an unrelated later packet.
            uart_flush_input(uart_);
            parser_.reset();
            return std::unexpected(error(Error::Kind::kOverflow));
        }

        TickType_t remaining = portMAX_DELAY;
        if (timeout != portMAX_DELAY) {
            const TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout) {
                return std::unexpected(error(Error::Kind::kTimeout));
            }
            remaining = timeout - elapsed;
        }

        // Take everything already buffered, or wait for at least one byte.
        std::size_t available = 0;
        uart_get_buffered_data_len(uart_, &available);
        const std::size_t wanted = std::clamp<std::size_t>(available, 1, rx_buf_.size());
        const int count = uart_read_bytes(uart_, rx_buf_.data(), wanted, remaining);
        if (count < 0) {
            parser_.reset();
            return std::unexpected(error(Error::Kind::kUart));
        }
        rx_start_ = 0;
        rx_end_ = static_cast<std::size_t>(count);
    }
}

Result<Response> Tf03::transact(const Command& command)
{
    if (auto sent = send(command); !sent) {
        return std::unexpected(sent.error());
    }

    const TickType_t start = xTaskGetTickCount();
    for (;;) {
        const TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= kResponseTimeout) {
            return std::unexpected(error(Error::Kind::kTimeout));
        }
        auto packet = read_packet(kResponseTimeout - elapsed);
        if (!packet) {
            // The data stream keeps coming while we wait; an overflow in it
            // says nothing about the response.
            if (packet.error().kind == Error::Kind::kOverflow) {
                continue;
            }
            return std::unexpected(packet.error());
        }
        if (const auto* response = std::get_if<Response>(&*packet);
            response != nullptr && response->id == command.id()) {
            return *response;
        }
    }
}

Result<> Tf03::send(const Command& command)
{
    const auto bytes = command.bytes();
    if (uart_write_bytes(uart_, bytes.data(), bytes.size()) != static_cast<int>(bytes.size()) ||
        uart_wait_tx_done(uart_, kTxTimeout) != ESP_OK) {
        return std::unexpected(error(Error::Kind::kUart));
    }
    return {};
}

Measurement Tf03::measurement(const DataFrame& frame) const
{
    const bool usable =
        frame.distance_cm < out_of_range_cm_ && frame.strength >= kMinReliableStrength;
    return Measurement{
        .distance_cm = usable ? std::optional{frame.distance_cm} : std::nullopt,
        .raw_distance_cm = frame.distance_cm,
        .strength = frame.strength,
    };
}

bool Tf03::overflowed()
{
    bool lost = false;
    uart_event_t event{};
    while (xQueueReceive(events_, &event, 0) == pdTRUE) {
        if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
            lost = true;
        }
    }
    return lost;
}

}  // namespace ping::tf03
