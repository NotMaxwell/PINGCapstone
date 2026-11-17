/// @file
/// The BNO085's bus on ESP-IDF, and the HAL CEVA's sh2 library runs on.
///
/// @ref EspSpiBus implements @ref ping::shtp::SpiBus with the `spi_master`
/// driver (no hardware CS, mode 3) and GPIOs for CS, WAKE and RST; INT falling
/// edges give a binary semaphore, so waiting for the hub blocks the task
/// instead of polling. @ref Hal adapts @ref ping::shtp::SpiTransport to
/// `sh2_Hal_t`.

#pragma once

#include "ping/shtp_spi.hpp"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern "C" {
#include <sh2_hal.h>
}

#include <cstdint>
#include <optional>

namespace ping::bno085 {

struct Pins {
    spi_host_device_t host;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sck;
    gpio_num_t cs;
    gpio_num_t intr;
    gpio_num_t reset;
    gpio_num_t wake;
    std::uint32_t frequency_hz;
};

class EspSpiBus final : public shtp::SpiBus {
  public:
    explicit EspSpiBus(const Pins& pins) : pins_(pins) {}
    ~EspSpiBus() override { deinit(); }

    /// Releases anything a previous call set up, then configures the pins,
    /// the SPI bus and the INT interrupt from scratch.
    esp_err_t init();
    void deinit();

    void set_cs(bool high) override;
    void set_wake(bool high) override;
    void set_reset(bool high) override;
    bool int_high() override;
    bool wait_int_low(std::uint32_t timeout_ms) override;
    bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t len) override;
    void delay_ms(std::uint32_t ms) override;
    std::uint32_t now_us() override;

  private:
    static void on_int(void* arg);

    Pins pins_;
    spi_device_handle_t device_ = nullptr;
    bool bus_ready_ = false;
    bool isr_added_ = false;
    SemaphoreHandle_t int_falling_ = nullptr;
    StaticSemaphore_t int_falling_storage_{};
};

/// What the HAL saw go wrong underneath sh2, which sh2 itself only reports as
/// a generic status code (or not at all, for reads).
struct HalError {
    enum class Kind : std::uint8_t {
        /// ESP-IDF refused to set up the bus or pins; see @ref esp.
        kInit,
        /// The hub did not pull INT low after a hardware reset.
        kBootTimeout,
        /// The hub did not answer WAKE before a write.
        kWakeTimeout,
        /// An SPI transfer failed.
        kBus,
    };
    Kind kind;
    esp_err_t esp = ESP_OK;
};

class Hal {
  public:
    explicit Hal(const Pins& pins);
    Hal(const Hal&) = delete;
    Hal& operator=(const Hal&) = delete;

    /// The table to hand to `sh2_open`.
    sh2_Hal_t* sh2() { return &glue_.hal; }

    /// How long one sh2 read may block waiting for INT. sh2 polls in tight
    /// loops; a short block here lets them sleep on the semaphore instead.
    void set_read_wait_ms(std::uint32_t ms) { read_wait_ms_ = ms; }

    /// Most sh2 operations wait for the hub's answer with no timeout at all.
    /// Once an armed deadline passes, the next read hands sh2 a made-up
    /// "reset complete" cargo, which is sh2's own way to abort an operation
    /// (it then returns SH2_ERR); @ref aborted reports that it happened.
    void arm_deadline(std::uint32_t timeout_ms);
    void disarm_deadline()
    {
        deadline_armed_ = false;
        aborted_ = false;
    }
    [[nodiscard]] bool aborted() const { return aborted_; }

    /// The first error since the last call, if any.
    std::optional<HalError> take_error();

    /// Cargos too large for sh2's buffer, read and thrown away.
    [[nodiscard]] std::uint32_t discarded_cargos() const { return discarded_; }

    std::uint32_t now_us() { return bus_.now_us(); }

  private:
    // sh2 calls back with the sh2_Hal_t pointer it was given; `hal` being the
    // first member of a standard-layout struct makes the cast back valid.
    struct Glue {
        sh2_Hal_t hal;
        Hal* self;
    };
    static Hal& from(sh2_Hal_t* hal) { return *reinterpret_cast<Glue*>(hal)->self; }

    static int open(sh2_Hal_t* hal);
    static void close(sh2_Hal_t* hal);
    static int read(sh2_Hal_t* hal, std::uint8_t* buffer, unsigned len, std::uint32_t* t_us);
    static int write(sh2_Hal_t* hal, std::uint8_t* buffer, unsigned len);
    static std::uint32_t get_time_us(sh2_Hal_t* hal);

    void record(HalError error);

    EspSpiBus bus_;
    shtp::SpiTransport transport_;
    Glue glue_;
    std::uint32_t read_wait_ms_ = 5;
    bool deadline_armed_ = false;
    bool aborted_ = false;
    std::uint32_t deadline_start_us_ = 0;
    std::uint32_t deadline_us_ = 0;
    std::optional<HalError> error_;
    std::uint32_t discarded_ = 0;
};

} // namespace ping::bno085
