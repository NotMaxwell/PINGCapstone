#include "ping/gps.hpp"

#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>
#include <optional>

namespace ping::gps {
namespace {

constexpr const char* kTag = "gps";

/// Room the driver keeps for received bytes between reads: about half a
/// second of the receiver's full output at 38400 baud.
constexpr int kRxRingBuffer = 2048;

/// Deep enough that overflow events are not pushed out by data events while
/// the caller is busy elsewhere; the driver drops events when it is full.
constexpr int kEventQueueDepth = 64;

/// How long the receiver may take to answer a poll or acknowledge a
/// configuration. u-blox answers within a navigation epoch; one second covers
/// the slowest rate this configures.
constexpr std::int64_t kResponseTimeoutUs = 1'000'000;

constexpr std::uint32_t kTxDoneTimeoutMs = 100;

} // namespace

const char* to_string(Error error)
{
    switch (error) {
    case Error::kNotStarted:
        return "not started";
    case Error::kUart:
        return "uart error";
    case Error::kTimeout:
        return "timeout";
    case Error::kRejected:
        return "rejected";
    case Error::kUnsupported:
        return "unsupported";
    case Error::kOverflow:
        return "overflow";
    }
    return "?";
}

esp_err_t Gps::begin(gpio_num_t tx, gpio_num_t rx, int baud)
{
    if (started_) {
        return ESP_ERR_INVALID_STATE;
    }
    uart_config_t config{};
    config.baud_rate = baud;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_driver_install(port_, kRxRingBuffer, 0, kEventQueueDepth, &events_, 0);
    if (err != ESP_OK) {
        events_ = nullptr;
        return err;
    }
    err = uart_param_config(port_, &config);
    if (err == ESP_OK) {
        err = uart_set_pin(port_, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        // Leave the port free for a retry.
        if (const esp_err_t del = uart_driver_delete(port_); del != ESP_OK) {
            ESP_LOGE(kTag, "uart_driver_delete: %s", esp_err_to_name(del));
        }
        events_ = nullptr;
        return err;
    }
    parser_.reset();
    rx_pos_ = rx_len_ = 0;
    started_ = true;
    return ESP_OK;
}

bool Gps::handle_events()
{
    bool lost = false;
    uart_event_t event;
    while (xQueueReceive(events_, &event, 0) == pdTRUE) {
        if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
            lost = true;
        }
    }
    if (lost) {
        if (const esp_err_t err = uart_flush_input(port_); err != ESP_OK) {
            ESP_LOGE(kTag, "uart_flush_input: %s", esp_err_to_name(err));
        }
        // A partial frame would be spliced onto an unrelated later one.
        parser_.reset();
        rx_pos_ = rx_len_ = 0;
        // Flushing can race with bytes still arriving; the stale events are
        // gone with the queue drain above.
        ESP_LOGW(kTag, "receive overflow, input flushed");
    }
    return lost;
}

Gps::Wait Gps::next_frame(std::int64_t deadline_us, ubx::Frame& out)
{
    if (const auto frame = parser_.poll()) {
        out = *frame;
        return Wait::kFrame;
    }
    for (;;) {
        while (rx_pos_ < rx_len_) {
            if (const auto frame = parser_.push(rx_[rx_pos_++])) {
                out = *frame;
                return Wait::kFrame;
            }
        }
        if (handle_events()) {
            return Wait::kOverflow;
        }

        const std::int64_t remaining_us = deadline_us - esp_timer_get_time();
        const std::uint32_t remaining_ms = remaining_us > 0 ? static_cast<std::uint32_t>((remaining_us + 999) / 1000) : 0;
        const int n = uart_read_bytes(port_, rx_.data(), static_cast<std::uint32_t>(rx_.size()),
                                      pdMS_TO_TICKS(remaining_ms));
        if (n < 0) {
            ESP_LOGE(kTag, "uart_read_bytes failed");
            return Wait::kUartError;
        }
        if (n == 0) {
            if (remaining_ms == 0 || esp_timer_get_time() >= deadline_us) {
                return Wait::kTimeout;
            }
            continue;
        }
        rx_pos_ = 0;
        rx_len_ = static_cast<std::size_t>(n);
    }
}

bool Gps::send(std::span<const std::uint8_t> bytes)
{
    const int written = uart_write_bytes(port_, bytes.data(), bytes.size());
    if (written < 0 || static_cast<std::size_t>(written) != bytes.size()) {
        ESP_LOGE(kTag, "uart_write_bytes wrote %d of %u", written, static_cast<unsigned>(bytes.size()));
        return false;
    }
    if (const esp_err_t err = uart_wait_tx_done(port_, pdMS_TO_TICKS(kTxDoneTimeoutMs)); err != ESP_OK) {
        ESP_LOGE(kTag, "uart_wait_tx_done: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

std::expected<Version, Error> Gps::configure(std::uint8_t rate_hz)
{
    if (!started_) {
        return std::unexpected(Error::kNotStarted);
    }
    const auto config = ubx::build_configure(rate_hz);
    if (!config) {
        return std::unexpected(Error::kUnsupported);
    }

    if (!send(config->view())) {
        return std::unexpected(Error::kUart);
    }
    ubx::Frame frame{};
    const std::int64_t ack_deadline = esp_timer_get_time() + kResponseTimeoutUs;
    for (bool acknowledged = false; !acknowledged;) {
        switch (next_frame(ack_deadline, frame)) {
        case Wait::kFrame:
            if (const auto ack = ubx::decode_ack(frame);
                ack && ack->cls == ubx::kClassCfg && ack->id == ubx::kIdCfgValset) {
                if (!ack->accepted) {
                    return std::unexpected(Error::kRejected);
                }
                acknowledged = true;
            }
            break;
        case Wait::kOverflow:
            break;
        case Wait::kTimeout:
            return std::unexpected(Error::kTimeout);
        case Wait::kUartError:
            return std::unexpected(Error::kUart);
        }
    }

    const auto poll = ubx::build_poll_mon_ver();
    if (!send(poll.view())) {
        return std::unexpected(Error::kUart);
    }
    const std::int64_t version_deadline = esp_timer_get_time() + kResponseTimeoutUs;
    for (;;) {
        switch (next_frame(version_deadline, frame)) {
        case Wait::kFrame:
            if (const auto version = ubx::decode_mon_ver(frame)) {
                return *version;
            }
            continue;
        case Wait::kOverflow:
            continue;
        case Wait::kTimeout:
            return std::unexpected(Error::kTimeout);
        case Wait::kUartError:
            return std::unexpected(Error::kUart);
        }
    }
}

std::expected<Solution, Error> Gps::read(std::uint32_t timeout_ms)
{
    if (!started_) {
        return std::unexpected(Error::kNotStarted);
    }
    const std::int64_t deadline = esp_timer_get_time() + static_cast<std::int64_t>(timeout_ms) * 1000;
    ubx::Frame frame{};
    for (;;) {
        switch (next_frame(deadline, frame)) {
        case Wait::kFrame:
            if (const auto solution = ubx::decode_nav_pvt(frame)) {
                return *solution;
            }
            continue;
        case Wait::kOverflow:
            return std::unexpected(Error::kOverflow);
        case Wait::kTimeout:
            return std::unexpected(Error::kTimeout);
        case Wait::kUartError:
            return std::unexpected(Error::kUart);
        }
    }
}

} // namespace ping::gps
