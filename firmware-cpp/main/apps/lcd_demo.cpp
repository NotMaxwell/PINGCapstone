/// @file
/// Standalone demo for the OLED display, which checks its wiring step by
/// step.
///
/// 1. Idle levels: with the internal pull-downs on, SDA and SCL read high only
///    if a powered module's pull-ups are on them.
/// 2. An I²C scan, which tells a display at the wrong address from no display
///    at all.
/// 3. Once the display answers, a count, the character set and the screen
///    inverted every other second, so a working panel is plain to see.
///
/// Steps 2 and 3 repeat, so changes to the wiring show up live.

#include "ping/constants.hpp"
#include "ping/display.hpp"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>
#include <string>

using namespace ping;

namespace {

constexpr const char* kTag = "lcd";

/// Step 1. The internal pull-downs (about 45 kΩ) lose to the module's
/// pull-ups (a few kΩ), so a line reads high only if a powered module is on
/// it.
void check_idle_levels()
{
    const struct {
        const char* name;
        gpio_num_t pin;
    } lines[] = {{"SDA (GPIO1)", kLcdSda}, {"SCL (GPIO2)", kLcdScl}};

    for (const auto& line : lines) {
        gpio_reset_pin(line.pin);
        gpio_set_direction(line.pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(line.pin, GPIO_PULLDOWN_ONLY);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    for (const auto& line : lines) {
        if (gpio_get_level(line.pin) != 0) {
            ESP_LOGI(kTag, "%s idles high: pulled up, good", line.name);
        } else {
            ESP_LOGW(kTag,
                     "%s idles LOW: check the display's VCC and GND, this wire, and that the module has pull-up "
                     "resistors",
                     line.name);
        }
    }
    for (const auto& line : lines) {
        gpio_reset_pin(line.pin);
    }
}

std::string list(const Addresses& found)
{
    std::string text;
    char one[8];
    for (std::size_t address = 0; address < found.size(); ++address) {
        if (found.test(address)) {
            std::snprintf(one, sizeof one, "%s0x%02x", text.empty() ? "" : ", ", static_cast<unsigned>(address));
            text += one;
        }
    }
    return text.empty() ? "none" : text;
}

} // namespace

extern "C" void app_main()
{
    check_idle_levels();

    static Display lcd;
    ESP_ERROR_CHECK(lcd.begin());

    bool ready = false;
    std::uint32_t count = 0;
    for (;;) {
        if (!lcd.present()) {
            ready = false;
            const Addresses found = lcd.scan();
            if (found.none()) {
                ESP_LOGW(kTag, "I2C scan: nothing answers. Check VCC to 3V3, GND, SDA to GPIO1 (J3-4) and SCL to "
                               "GPIO2 (J3-5); swapped SDA and SCL is the usual mistake");
            } else {
                ESP_LOGW(kTag,
                         "I2C scan: nothing at 0x%02x, but found %s. Set kLcdAddress in ping/constants.hpp to the "
                         "display's address (0x3C or 0x3D)",
                         kLcdAddress, list(found).c_str());
            }
        } else {
            if (!ready) {
                ESP_LOGI(kTag, "I2C scan: display answers at 0x%02x", kLcdAddress);
                if (const esp_err_t err = lcd.init(); err == ESP_OK) {
                    ready = true;
                } else {
                    ESP_LOGW(kTag, "display init failed: %s", esp_err_to_name(err));
                }
            }

            if (ready) {
                ++count;
                const bool inverted = count % 2 == 0;
                esp_err_t err = lcd.set_inverted(inverted);
                const Text counter{"count %lu", static_cast<unsigned long>(count)};
                const char* rows[] = {"PING display", counter.c_str(), "0123456789ABCDEF", "12.5° ±0.1 m"};
                for (std::size_t row = 0; row < kRows && err == ESP_OK; ++row) {
                    err = lcd.write_line(row, rows[row]);
                }
                if (err == ESP_OK) {
                    ESP_LOGI(kTag, "wrote count %lu, %s", static_cast<unsigned long>(count),
                             inverted ? "inverted" : "normal");
                } else {
                    ESP_LOGW(kTag, "display write failed: %s", esp_err_to_name(err));
                    ready = false;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
