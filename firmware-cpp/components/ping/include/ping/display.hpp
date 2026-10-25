/// @file
/// A 128 by 64 monochrome OLED with an SH1106 or SSD1306 controller, on I²C,
/// showing lines of text.
///
/// Drawing, fonts and the controllers' init sequences come from u8g2. This
/// file connects u8g2 to ESP-IDF's `i2c_master` driver and lays text out the
/// way the Rust firmware's `ssd1306` module does: the 8 by 13 pixel font on
/// rows two pages (16 pixels) apart, @ref kRows lines of @ref kColumns
/// characters. The font covers ISO 8859-1 and text is UTF-8, so `°` and `±`
/// show as themselves.
///
/// Only the two pages of a row are sent when that row changes: 260 bytes,
/// about 6 ms at 400 kHz.
///
/// # Example
///
/// @code
/// ping::Display display;
/// ESP_ERROR_CHECK(display.begin());
/// display.init();
/// display.show({"distance", ping::Text("%u cm", 1234U)});
/// @endcode

#pragma once

#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <u8g2.h>

#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

namespace ping {

/// Characters per line.
inline constexpr std::size_t kColumns = 16;
/// Lines of text, each two pages tall.
inline constexpr std::size_t kRows = 4;

/// One line of text formatted printf-style into a buffer of its own, so a
/// list of them can be handed to @ref Display::show as temporaries.
class Text {
  public:
    // A plain string, without formatting.
    Text(const char* text); // NOLINT(google-explicit-constructor): lets show() take string literals.
    Text(const char* format, auto first, auto... rest) { print(format, first, rest...); }

    [[nodiscard]] const char* c_str() const { return buf_.data(); }

  private:
    void print(const char* format, ...) __attribute__((format(printf, 2, 3)));

    /// A little more than a screen line's worth of bytes; UTF-8 takes up to
    /// two per Latin-1 character, and the screen cuts off what is too wide.
    std::array<char, 4 * kColumns> buf_{};
};

/// Paces the demos' display updates. The sensors report faster than anyone
/// can read, and each rewrite holds up the task for a few milliseconds.
class RefreshTimer {
  public:
    explicit RefreshTimer(std::uint32_t period_ms) : period_us_(static_cast<std::int64_t>(period_ms) * 1000) {}

    /// Whether it is time to show the next reading, at most once a period.
    bool due()
    {
        const std::int64_t now = esp_timer_get_time();
        if (now < next_us_) {
            return false;
        }
        next_us_ = now + period_us_;
        return true;
    }

  private:
    std::int64_t period_us_;
    std::int64_t next_us_ = 0;
};

/// The I²C addresses that answered a @ref Display::scan.
using Addresses = std::bitset<128>;

/// The board's display (see @ref ping/constants.hpp for its pins, address
/// and controller).
class Display {
  public:
    /// Sets up the I²C bus. Nothing is sent.
    esp_err_t begin();

    /// Sets the panel up, blanks it and turns it on. Safe to call again at
    /// any time.
    esp_err_t init();

    /// Blanks the whole screen.
    esp_err_t clear();

    /// Swaps light and dark across the whole screen, or swaps them back.
    /// Takes effect straight away and leaves the picture alone.
    esp_err_t set_inverted(bool inverted);

    /// Replaces line `row` with `text`. Rows past the bottom are ignored, and
    /// text past the right edge is cut off.
    esp_err_t write_line(std::size_t row, const char* text);

    /// Shows `lines` one per row, blanking the rows below them, and logs
    /// them on one line. A failed display write is logged once and otherwise
    /// ignored, so a loose display never stops a sensor being read.
    void show(std::initializer_list<Text> lines);

    /// Asks every ordinary I²C address in turn whether anything is there.
    ///
    /// For finding out why the display does not answer: an empty bus points
    /// at power or wiring, and a device somewhere else is usually the display
    /// at another address.
    Addresses scan();

    /// Whether the display answers at its address.
    bool present();

  private:
    static std::uint8_t byte_cb(u8x8_t* u8x8, std::uint8_t msg, std::uint8_t arg_int, void* arg_ptr);
    static std::uint8_t gpio_and_delay_cb(u8x8_t* u8x8, std::uint8_t msg, std::uint8_t arg_int, void* arg_ptr);

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t device_ = nullptr;
    u8g2_t u8g2_{};
    bool ready_ = false;
    bool failing_ = false;

    /// One I²C write as u8g2 builds it: a control byte and up to a page of
    /// data.
    std::array<std::uint8_t, 160> tx_{};
    std::size_t tx_len_ = 0;
    /// The first error the I²C driver returned during the current u8g2 call.
    esp_err_t error_ = ESP_OK;
};

} // namespace ping
