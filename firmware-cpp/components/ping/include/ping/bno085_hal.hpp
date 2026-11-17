/// @file
/// CEVA's SPI HAL for sh2 (@ref ping/shtp_spi.hpp) on ESP-IDF.
///
/// @ref EspSpiBus supplies what the STM32 HAL supplied to CEVA's code: the
/// `spi_master` driver in mode 3 with chip select driven by hand
/// (`spics_io_num = -1`), GPIOs for CSN, PS0/WAKE and RSTN, a GPIO interrupt
/// on INTN's falling edge, and `esp_timer` for microseconds. @ref Hal wraps it
/// all as the `sh2_Hal_t` that `sh2_open` takes.

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
    EspSpiBus(const Pins& pins, shtp::SpiHal& hal) : pins_(pins), hal_(hal) {}
    ~EspSpiBus() override { deinit_hw(); }

    /// What ESP-IDF returned when @ref init_hw last failed, ESP_OK otherwise.
    [[nodiscard]] esp_err_t init_error() const { return init_error_; }

    bool init_hw() override;
    void deinit_hw() override;
    void rstn(bool high) override;
    void ps0_waken(bool high) override;
    void csn(bool high) override;
    bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t len) override;
    void enable_ints() override;
    void disable_ints() override;
    void wait_int(std::uint32_t timeout_us) override;
    void delay_us(std::uint32_t us) override;
    std::uint32_t time_now_us() override;

  private:
    static void on_intn(void* arg);
    esp_err_t setup();

    Pins pins_;
    shtp::SpiHal& hal_;
    spi_device_handle_t device_ = nullptr;
    bool bus_ready_ = false;
    bool isr_added_ = false;
    esp_err_t init_error_ = ESP_OK;
    SemaphoreHandle_t intn_ = nullptr;
    StaticSemaphore_t intn_storage_{};
};

/// The `sh2_Hal_t` for one BNO08x.
class Hal {
  public:
    explicit Hal(const Pins& pins);
    Hal(const Hal&) = delete;
    Hal& operator=(const Hal&) = delete;

    /// The table to hand to `sh2_open`.
    sh2_Hal_t* sh2() { return &glue_.hal; }
    shtp::SpiHal& spi() { return spi_; }
    [[nodiscard]] const shtp::SpiHal& spi() const { return spi_; }
    [[nodiscard]] esp_err_t init_error() const { return bus_.init_error(); }
    std::uint32_t now_us() { return spi_.get_time_us(); }

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

    shtp::SpiHal spi_;
    EspSpiBus bus_;
    Glue glue_;
};

} // namespace ping::bno085
