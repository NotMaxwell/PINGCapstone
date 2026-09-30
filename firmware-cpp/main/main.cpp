// PING: aim the device at something, press the button, and it reports that
// thing's GPS coordinate.
//
// The Arduino core for ESP32 runs here as an ESP-IDF component, and supplies
// the HAL the sensor libraries are written against: Wire for the IMU and
// HardwareSerial for the GPS and LiDAR. Each sensor runs in its own FreeRTOS
// task and keeps its latest reading in a shared slot (sensors.cpp):
//
// * the NEO-M9N GPS gives the device's own position,
// * the BNO085 IMU gives its orientation, and from that the direction the
//   LiDAR points: a compass bearing and an elevation angle,
// * the TF03 LiDAR gives the distance to whatever it points at.
//
// When the button is pressed, this task takes the latest reading from each
// and walks the LiDAR's range out from the GPS position along that direction
// (sighting.cpp, and components/geo). The result goes to the console on the
// USB serial port, one line per press, alongside the diagnostics.
//
// Before trusting the bearings, set kLidarBoresight in config.hpp to how the
// LiDAR is mounted against the IMU, and kMagneticDeclinationDeg to where the
// device is used.

#include <array>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include <Arduino.h>

#include "config.hpp"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "rtos.hpp"
#include "sensors.hpp"
#include "sighting.hpp"

namespace ping {

namespace {

/// A push button to ground, with the pin's pull-up holding it high.
class Button {
public:
    explicit Button(int pin) : pin_{pin}, edge_{xSemaphoreCreateBinary()}
    {
        configASSERT(edge_ != nullptr);
        pinMode(pin, INPUT_PULLUP);
        attachInterruptArg(pin, on_edge, this, CHANGE);
    }

    Button(const Button&) = delete;
    Button& operator=(const Button&) = delete;

    [[nodiscard]] bool is_pressed() const { return digitalRead(pin_) == LOW; }

    /// Blocks until the button is down, which may be already.
    void wait_until_pressed() { wait_for(true); }

    /// Blocks until the button is up, which may be already.
    void wait_until_released() { wait_for(false); }

private:
    void wait_for(bool pressed)
    {
        while (is_pressed() != pressed) {
            xSemaphoreTake(edge_, portMAX_DELAY);
        }
    }

    static void IRAM_ATTR on_edge(void* arg)
    {
        auto& self = *static_cast<Button*>(arg);
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(self.edge_, &woken);
        portYIELD_FROM_ISR(woken);
    }

    int pin_;
    SemaphoreHandle_t edge_;
};

/// Waits for the button to be let go and to stay up past any contact bounce.
void wait_for_release(Button& button)
{
    for (;;) {
        button.wait_until_released();
        rtos::sleep(config::kDebounce);
        if (!button.is_pressed()) {
            return;
        }
    }
}

/// Sends one result line to the computer on the USB serial port. With
/// nothing reading at the other end, ESP-IDF's console driver drops it after
/// a short timeout rather than blocking.
void send_line(std::uint32_t press, std::string_view result, bool located)
{
    std::printf("target #%" PRIu32 ": %s%.*s\n", press, located ? "" : "none, ",
                static_cast<int>(result.size()), result.data());
    std::fflush(stdout);
}

[[noreturn]] void run()
{
    // Brings up the Arduino core: its peripheral manager, and the pin and
    // interrupt bookkeeping Wire, HardwareSerial and attachInterrupt use.
    initArduino();

    static Button button{config::kButtonPin};

    // Each task opens its own sensor; the wiring is in config.hpp.
    sensors::start_gps();
    sensors::start_imu();
    sensors::start_lidar();

    std::uint32_t presses = 0;
    for (;;) {
        button.wait_until_pressed();
        // Take the readings from when the button went down, before pressing
        // it has had time to jog where the device points.
        const auto sighting = take_sighting();
        rtos::sleep(config::kDebounce);
        if (!button.is_pressed()) {
            // Too short to be a press: noise, or contacts bouncing.
            continue;
        }

        ++presses;
        std::array<char, 320> text{};
        send_line(presses, sighting ? describe(*sighting, text) : describe(sighting.error(), text),
                  sighting.has_value());

        // One coordinate per press, however long the button is held.
        wait_for_release(button);
    }
}

}  // namespace

}  // namespace ping

extern "C" void app_main()
{
    ping::run();
}
