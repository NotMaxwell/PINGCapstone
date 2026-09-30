/// @file
/// The BNO085 IMU, through CEVA's own SH-2 driver.
///
/// The BNO085 is not a register-mapped IMU. It runs sensor fusion on its own
/// microcontroller and speaks CEVA's SH-2 protocol over SHTP: the host asks
/// for a *report* at some rate and the hub pushes reports until told
/// otherwise. All of that protocol comes from CEVA's library, vendored in
/// vendor/sh2. What this class adds is the ESP-IDF half the library asks its
/// user to write, an `sh2_Hal_t` over the I2C master driver, and a small C++
/// face on the library's C API.
///
/// # Interrupt pin
///
/// The hub pulls INT low while it has a packet waiting. The HAL only touches
/// the bus once INT is low, and otherwise blocks on it for a moment, which
/// also keeps SH-2's own busy-wait loops from starving other tasks.
///
/// # One at a time
///
/// CEVA's library keeps its state in globals, so only one Bno085 may exist.

#pragma once

#include <cstdint>
#include <expected>
#include <optional>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

extern "C" {
#include "sh2.h"
#include "sh2_hal.h"
}

namespace ping::bno085 {

/// I2C address, selected on the board by the ADR/SA0 pin.
enum class Address : std::uint8_t {
    kDefault = 0x4A,    ///< ADR pulled low, the usual case.
    kAlternate = 0x4B,  ///< ADR pulled high.
};

/// How the BNO085 is wired.
struct Config {
    i2c_port_num_t port = I2C_NUM_0;
    gpio_num_t sda = GPIO_NUM_NC;
    gpio_num_t scl = GPIO_NUM_NC;
    /// The hub's INT line: open drain and active low. The pin's pull-up is
    /// switched on.
    gpio_num_t interrupt = GPIO_NUM_NC;
    Address address = Address::kDefault;
    std::uint32_t bus_hz = 400'000;
};

/// A failure reported by CEVA's library: one of the `SH2_ERR*` codes.
struct Error {
    int code;

    /// A static, human-readable description, for log lines.
    [[nodiscard]] const char* message() const;
};

template <typename T = void>
using Result = std::expected<T, Error>;

/// How far the hub trusts a report, from the low two bits of its status.
/// For the rotation vector this is the magnetometer's calibration.
enum class Accuracy : std::uint8_t {
    kUnreliable = 0,
    kLow = 1,
    kMedium = 2,
    kHigh = 3,
};

[[nodiscard]] const char* to_string(Accuracy accuracy);

/// A unit quaternion.
struct Quaternion {
    float i;
    float j;
    float k;
    float real;
};

/// A vector in the sensor's frame: the axes printed on the breakout.
struct Vec3 {
    float x;
    float y;
    float z;
};

/// One rotation vector report.
struct Orientation {
    /// Turns a vector in the sensor's frame into the world frame: X east,
    /// Y magnetic north, Z up (Android's convention, which SH-2 follows).
    Quaternion quaternion;
    /// The hub's estimate of its heading error, radians.
    float heading_accuracy_rad;
    /// How well calibrated the magnetometer is.
    Accuracy calibration;
};

/// Why the hub last reset, from its product ID response.
enum class ResetCause : std::uint8_t {
    kNotApplicable = 0,
    kPowerOn = 1,
    kInternalSystemReset = 2,
    kWatchdogTimeout = 3,
    kExternalReset = 4,
    kOther = 5,
};

[[nodiscard]] const char* to_string(ResetCause cause);

/// The hub's firmware identification.
struct ProductId {
    ResetCause reset_cause;
    std::uint8_t version_major;
    std::uint8_t version_minor;
    std::uint16_t version_patch;
    std::uint32_t part_number;
    std::uint32_t build_number;
};

class Bno085 {
public:
    /// Sets up the I2C bus and the interrupt pin. Nothing is sent yet.
    /// Aborts on a configuration ESP-IDF rejects, such as a pin that does
    /// not exist, since there is nothing to do about that at run time.
    explicit Bno085(const Config& config);
    ~Bno085();

    Bno085(const Bno085&) = delete;
    Bno085& operator=(const Bno085&) = delete;

    /// Resets the hub and opens an SH-2 session with it. Call again, after
    /// stop(), to start over.
    Result<ProductId> start();

    /// Closes the SH-2 session. Reports the hub is still sending are left
    /// unread; start() resets it anyway.
    void stop();

    /// Asks for rotation vector reports every `interval_us` microseconds:
    /// the orientation fused from accelerometer, gyroscope and magnetometer,
    /// referenced to magnetic north.
    Result<> enable_rotation_vector(std::uint32_t interval_us);

    /// Waits up to `timeout` for the hub to raise INT, then reads everything
    /// it has waiting. Returns false if INT stayed high the whole time.
    bool poll(TickType_t timeout);

    /// The newest rotation vector read by poll() since the last call.
    std::optional<Orientation> take_orientation();

    /// Whether the hub has reset since start() finished. A reset stops every
    /// report, so the caller should start() it again.
    bool take_reset();

private:
    /// sh2 hands its HAL callbacks a pointer to the sh2_Hal_t it was given.
    /// Wrapping it in a standard-layout struct lets them find their way back
    /// to the Bno085 it belongs to.
    struct Hal {
        sh2_Hal_t hal;
        Bno085* self;
    };

    static int hal_open(sh2_Hal_t* hal);
    static void hal_close(sh2_Hal_t* hal);
    static int hal_read(sh2_Hal_t* hal, std::uint8_t* buffer, unsigned len, std::uint32_t* t_us);
    static int hal_write(sh2_Hal_t* hal, std::uint8_t* buffer, unsigned len);
    static std::uint32_t hal_time_us(sh2_Hal_t* hal);
    static void on_async_event(void* cookie, sh2_AsyncEvent_t* event);
    static void on_sensor_event(void* cookie, sh2_SensorEvent_t* event);
    static void on_interrupt(void* arg);

    static Bno085& from(sh2_Hal_t* hal);

    [[nodiscard]] bool interrupt_asserted() const;
    bool wait_for_interrupt(TickType_t timeout);
    int soft_reset();
    int read_packet(std::uint8_t* buffer, unsigned len, std::uint32_t* t_us);

    Hal hal_;
    gpio_num_t interrupt_pin_;
    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t device_ = nullptr;
    SemaphoreHandle_t interrupt_signal_ = nullptr;
    bool open_ = false;
    bool reset_seen_ = false;
    std::optional<Orientation> orientation_;
};

}  // namespace ping::bno085
