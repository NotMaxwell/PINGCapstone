#include "ping/display.hpp"

#include "ping/constants.hpp"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdarg>
#include <cstdio>
#include <string>

namespace ping {
namespace {

constexpr const char* kTag = "display";

/// How long one I²C transaction may take before the driver gives up.
constexpr int kI2cTimeoutMs = 50;

constexpr std::uint8_t kDisplayNormal = 0xA6;
constexpr std::uint8_t kDisplayInverted = 0xA7;

} // namespace

Text::Text(const char* text) { std::snprintf(buf_.data(), buf_.size(), "%s", text); }

void Text::print(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    std::vsnprintf(buf_.data(), buf_.size(), format, args);
    va_end(args);
}

esp_err_t Display::begin()
{
    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = kLcdI2c;
    bus_config.sda_io_num = kLcdSda;
    bus_config.scl_io_num = kLcdScl;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    // The modules have their own pull-ups; these only help.
    bus_config.flags.enable_internal_pullup = true;
    if (esp_err_t err = i2c_new_master_bus(&bus_config, &bus_); err != ESP_OK) {
        return err;
    }

    i2c_device_config_t device_config{};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = kLcdAddress;
    device_config.scl_speed_hz = kLcdI2cFrequencyHz;
    return i2c_master_bus_add_device(bus_, &device_config, &device_);
}

std::uint8_t Display::byte_cb(u8x8_t* u8x8, std::uint8_t msg, std::uint8_t arg_int, void* arg_ptr)
{
    auto* self = static_cast<Display*>(u8x8_GetUserPtr(u8x8));
    switch (msg) {
    case U8X8_MSG_BYTE_INIT:
    case U8X8_MSG_BYTE_SET_DC:
        return 1;
    case U8X8_MSG_BYTE_START_TRANSFER:
        self->tx_len_ = 0;
        return 1;
    case U8X8_MSG_BYTE_SEND: {
        const auto* data = static_cast<const std::uint8_t*>(arg_ptr);
        for (std::uint8_t i = 0; i < arg_int && self->tx_len_ < self->tx_.size(); ++i) {
            self->tx_[self->tx_len_++] = data[i];
        }
        return 1;
    }
    case U8X8_MSG_BYTE_END_TRANSFER: {
        const esp_err_t err = i2c_master_transmit(self->device_, self->tx_.data(), self->tx_len_, kI2cTimeoutMs);
        if (self->error_ == ESP_OK) {
            self->error_ = err;
        }
        return 1;
    }
    default:
        return 0;
    }
}

std::uint8_t Display::gpio_and_delay_cb(u8x8_t* /*u8x8*/, std::uint8_t msg, std::uint8_t arg_int, void* /*arg_ptr*/)
{
    if (msg == U8X8_MSG_DELAY_MILLI) {
        vTaskDelay(pdMS_TO_TICKS(arg_int) + 1);
    }
    // No reset or chip select pins on an I²C module; nothing else to do.
    return 1;
}

esp_err_t Display::init()
{
    if (kLcdController == LcdController::kSh1106) {
        u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2_, U8G2_R0, byte_cb, gpio_and_delay_cb);
    } else {
        u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2_, U8G2_R0, byte_cb, gpio_and_delay_cb);
    }
    u8g2_SetUserPtr(&u8g2_, this);
    // u8g2 addresses devices with the read/write bit in place.
    u8g2_SetI2CAddress(&u8g2_, kLcdAddress << 1);

    error_ = ESP_OK;
    u8g2_InitDisplay(&u8g2_);
    u8g2_SetPowerSave(&u8g2_, 0);
    u8g2_SetFont(&u8g2_, u8g2_font_8x13_tf);
    u8g2_SetFontPosTop(&u8g2_);
    ready_ = error_ == ESP_OK;
    if (!ready_) {
        return error_;
    }
    return clear();
}

esp_err_t Display::clear()
{
    error_ = ESP_OK;
    u8g2_ClearBuffer(&u8g2_);
    u8g2_SendBuffer(&u8g2_);
    return error_;
}

esp_err_t Display::set_inverted(bool inverted)
{
    error_ = ESP_OK;
    u8x8_t* u8x8 = u8g2_GetU8x8(&u8g2_);
    u8x8_cad_StartTransfer(u8x8);
    u8x8_cad_SendCmd(u8x8, inverted ? kDisplayInverted : kDisplayNormal);
    u8x8_cad_EndTransfer(u8x8);
    return error_;
}

esp_err_t Display::write_line(std::size_t row, const char* text)
{
    if (row >= kRows) {
        return ESP_OK;
    }
    const auto top = static_cast<u8g2_uint_t>(16 * row);

    u8g2_SetDrawColor(&u8g2_, 0);
    u8g2_DrawBox(&u8g2_, 0, top, 128, 16);
    u8g2_SetDrawColor(&u8g2_, 1);
    u8g2_DrawUTF8(&u8g2_, 0, top, text);

    // Send only the two eight-pixel tile rows this line covers.
    error_ = ESP_OK;
    u8g2_UpdateDisplayArea(&u8g2_, 0, static_cast<std::uint8_t>(2 * row), 16, 2);
    return error_;
}

void Display::show(std::initializer_list<Text> lines)
{
    std::string joined;
    for (const Text& line : lines) {
        if (!joined.empty()) {
            joined += " | ";
        }
        joined += line.c_str();
    }
    ESP_LOGI(kTag, "%s", joined.c_str());

    if (!ready_) {
        // Try to bring a display back that was unplugged or never answered.
        if (init() != ESP_OK) {
            failing_ = true;
            return;
        }
    }

    for (std::size_t row = 0; row < kRows; ++row) {
        const char* text = row < lines.size() ? std::data(lines)[row].c_str() : "";
        if (esp_err_t err = write_line(row, text); err != ESP_OK) {
            if (!failing_) {
                ESP_LOGW(kTag, "write failed: %s", esp_err_to_name(err));
            }
            failing_ = true;
            ready_ = false;
            return;
        }
    }
    if (failing_) {
        ESP_LOGI(kTag, "display back");
        failing_ = false;
    }
}

Addresses Display::scan()
{
    Addresses found;
    for (std::uint16_t address = 0x08; address <= 0x77; ++address) {
        if (i2c_master_probe(bus_, address, kI2cTimeoutMs) == ESP_OK) {
            found.set(address);
        }
    }
    return found;
}

bool Display::present() { return i2c_master_probe(bus_, kLcdAddress, kI2cTimeoutMs) == ESP_OK; }

} // namespace ping
