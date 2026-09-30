// PING: aim the device at something, press the button, and it reports that
// thing's GPS coordinate.
//
// Each sensor runs in its own FreeRTOS task and keeps its latest reading in
// a shared slot (sensors.cpp):
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

#include "config.hpp"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ping/bno085.hpp"
#include "ping/neo_m9n.hpp"
#include "ping/tf03.hpp"
#include "rtos.hpp"
#include "sensors.hpp"
#include "sighting.hpp"

namespace ping {

namespace {

// Stack sizes, in bytes. ubxlib keeps sizeable buffers on the stack, and
// every task formats log lines.
constexpr std::uint32_t kGpsStack = 8192;
constexpr std::uint32_t kImuStack = 6144;
constexpr std::uint32_t kLidarStack = 6144;

/// The sensor tasks run above app_main's priority of 1, so a button press
/// never holds up a reading.
constexpr UBaseType_t kSensorPriority = 5;

/// A push button to ground, with the pin's pull-up holding it high.
class Button {
public:
    explicit Button(gpio_num_t pin) : pin_{pin}, edge_{xSemaphoreCreateBinary()}
    {
        configASSERT(edge_ != nullptr);
        gpio_config_t pin_config{};
        pin_config.pin_bit_mask = 1ULL << pin;
        pin_config.mode = GPIO_MODE_INPUT;
        pin_config.pull_up_en = GPIO_PULLUP_ENABLE;
        pin_config.intr_type = GPIO_INTR_ANYEDGE;
        ESP_ERROR_CHECK(gpio_config(&pin_config));
        ESP_ERROR_CHECK(gpio_isr_handler_add(pin, on_edge, this));
    }

    Button(const Button&) = delete;
    Button& operator=(const Button&) = delete;

    [[nodiscard]] bool is_pressed() const { return gpio_get_level(pin_) == 0; }

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

    gpio_num_t pin_;
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
    // The GPIO interrupt service is shared by the IMU's INT pin and the
    // button.
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // The sensors are wired as in the Rust demo binaries; see config.hpp.
    static neo_m9n::NeoM9n gps{{
        .uart = config::kGpsUart,
        .tx_pin = config::kGpsTxPin,
        .rx_pin = config::kGpsRxPin,
        .baud_rate = neo_m9n::kDefaultBaudRate,
    }};
    static bno085::Bno085 imu{{
        .port = config::kImuI2c,
        .sda = config::kImuSdaPin,
        .scl = config::kImuSclPin,
        .interrupt = config::kImuIntPin,
        .address = bno085::Address::kDefault,
        .bus_hz = 400'000,
    }};
    static tf03::Tf03 lidar{{
        .uart = config::kLidarUart,
        .tx_pin = config::kLidarTxPin,
        .rx_pin = config::kLidarRxPin,
        .baud_rate = tf03::kDefaultBaudRate,
        .out_of_range_cm = tf03::kDefaultOutOfRangeCm,
    }};
    static Button button{config::kButtonPin};

    rtos::spawn<sensors::gps_task>("gps", gps, kGpsStack, kSensorPriority);
    rtos::spawn<sensors::imu_task>("imu", imu, kImuStack, kSensorPriority);
    rtos::spawn<sensors::lidar_task>("lidar", lidar, kLidarStack, kSensorPriority);

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
