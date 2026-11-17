// The hardware layer under CEVA's SPI HAL (shtp_spi.cpp), standing in for the
// STM32 HAL calls in CEVA's sh2-demo-nucleo app/spi_hal.c. Deviations from
// that file at this layer:
//
// - Timer: esp_timer instead of TIM2 (both a 32-bit microsecond count).
// - The INTN ISR records the time, calls SpiHal::on_intn and gives a binary
//   semaphore; it starts no transfer (see shtp_spi.cpp for why).
// - enable_ints treats INTN already low as an edge: the STM32 EXTI latches an
//   edge while its interrupt is masked, and the ESP32 GPIO interrupt is not
//   relied on to.
// - SPI without DMA, in 64-byte chunks under one CSN assertion: with DMA,
//   ESP-IDF copies unaligned buffers through a heap bounce buffer on every
//   transfer.

#include "ping/bno085_hal.hpp"

#include <esp_attr.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/task.h>

extern "C" {
#include <sh2_err.h>
}

#include <algorithm>
#include <array>

namespace ping::bno085 {

namespace {

constexpr const char* kTag = "bno085";

static_assert(shtp::kSh2Ok == SH2_OK && shtp::kSh2Err == SH2_ERR && shtp::kSh2ErrBadParam == SH2_ERR_BAD_PARAM &&
              shtp::kSh2ErrIo == SH2_ERR_IO && shtp::kSh2ErrTimeout == SH2_ERR_TIMEOUT);
static_assert(shtp::kMaxTransferIn == SH2_HAL_MAX_TRANSFER_IN && shtp::kMaxTransferOut == SH2_HAL_MAX_TRANSFER_OUT);

constexpr std::size_t kChunk = 64;

TickType_t ticks_for_us(std::uint32_t us)
{
    const std::uint32_t ms = (us + 999U) / 1000U;
    const TickType_t ticks = pdMS_TO_TICKS(ms);
    return ms > 0 && ticks == 0 ? 1 : ticks;
}

std::uint32_t level(bool high) { return high ? 1U : 0U; }

} // namespace

// ---------------------------------------------------------------------------
// EspSpiBus
// ---------------------------------------------------------------------------

void IRAM_ATTR EspSpiBus::on_intn(void* arg)
{
    auto* self = static_cast<EspSpiBus*>(arg);
    self->hal_.on_intn(static_cast<std::uint32_t>(esp_timer_get_time()));
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(self->intn_, &woken);
    portYIELD_FROM_ISR(woken);
}

bool EspSpiBus::init_hw()
{
    deinit_hw();
    init_error_ = setup();
    if (init_error_ != ESP_OK) {
        deinit_hw();
        return false;
    }
    return true;
}

esp_err_t EspSpiBus::setup()
{
    if (intn_ == nullptr) {
        intn_ = xSemaphoreCreateBinaryStatic(&intn_storage_);
    }

    // hal_init_gpio / hal_init_spi: PS0_WAKEN, RSTN and CSN start low. The
    // levels are latched before the pins become outputs.
    gpio_set_level(pins_.wake, 0);
    gpio_set_level(pins_.reset, 0);
    gpio_set_level(pins_.cs, 0);
    gpio_config_t outputs{};
    outputs.pin_bit_mask = (1ULL << pins_.cs) | (1ULL << pins_.wake) | (1ULL << pins_.reset);
    outputs.mode = GPIO_MODE_OUTPUT;
    outputs.intr_type = GPIO_INTR_DISABLE;
    esp_err_t err = gpio_config(&outputs);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "control pins: %s", esp_err_to_name(err));
        return err;
    }

    gpio_config_t input{};
    input.pin_bit_mask = 1ULL << pins_.intr;
    input.mode = GPIO_MODE_INPUT;
    input.pull_up_en = GPIO_PULLUP_ENABLE;
    input.intr_type = GPIO_INTR_NEGEDGE;
    err = gpio_config(&input);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "INTN pin: %s", esp_err_to_name(err));
        return err;
    }
    // Masked until enable_ints, as CEVA leaves the EXTI IRQ until then.
    gpio_intr_disable(pins_.intr);

    // Another driver may have installed the shared ISR service already; that
    // is fine, and it is never uninstalled here for the same reason.
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(kTag, "GPIO ISR service: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_isr_handler_add(pins_.intr, on_intn, this);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "INTN handler: %s", esp_err_to_name(err));
        return err;
    }
    isr_added_ = true;

    spi_bus_config_t bus{};
    bus.mosi_io_num = pins_.mosi;
    bus.miso_io_num = pins_.miso;
    bus.sclk_io_num = pins_.sck;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = static_cast<int>(kChunk);
    err = spi_bus_initialize(pins_.host, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "SPI bus: %s", esp_err_to_name(err));
        return err;
    }
    bus_ready_ = true;

    spi_device_interface_config_t device{};
    device.mode = 3; // CPOL high, CPHA second edge, as CEVA configures SPI1
    device.clock_speed_hz = static_cast<int>(pins_.frequency_hz);
    device.spics_io_num = -1; // CSN is driven by hand (SPI_NSS_SOFT)
    device.queue_size = 1;
    err = spi_bus_add_device(pins_.host, &device, &device_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "SPI device: %s", esp_err_to_name(err));
        device_ = nullptr;
        return err;
    }
    return ESP_OK;
}

void EspSpiBus::deinit_hw()
{
    if (isr_added_) {
        gpio_intr_disable(pins_.intr);
        if (const esp_err_t err = gpio_isr_handler_remove(pins_.intr); err != ESP_OK) {
            ESP_LOGW(kTag, "removing INTN handler: %s", esp_err_to_name(err));
        }
        isr_added_ = false;
    }
    if (device_ != nullptr) {
        if (const esp_err_t err = spi_bus_remove_device(device_); err != ESP_OK) {
            ESP_LOGW(kTag, "removing SPI device: %s", esp_err_to_name(err));
        }
        device_ = nullptr;
    }
    if (bus_ready_) {
        if (const esp_err_t err = spi_bus_free(pins_.host); err != ESP_OK) {
            ESP_LOGW(kTag, "freeing SPI bus: %s", esp_err_to_name(err));
        }
        bus_ready_ = false;
    }
}

void EspSpiBus::rstn(bool high) { gpio_set_level(pins_.reset, level(high)); }
void EspSpiBus::ps0_waken(bool high) { gpio_set_level(pins_.wake, level(high)); }
void EspSpiBus::csn(bool high) { gpio_set_level(pins_.cs, level(high)); }

bool EspSpiBus::transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t len)
{
    if (device_ == nullptr) {
        return false;
    }
    std::size_t done = 0;
    while (done < len) {
        const std::size_t n = std::min(len - done, kChunk);
        spi_transaction_t t{};
        t.length = n * 8;
        t.tx_buffer = tx + done;
        t.rx_buffer = rx + done;
        if (const esp_err_t err = spi_device_polling_transmit(device_, &t); err != ESP_OK) {
            ESP_LOGE(kTag, "SPI transfer: %s", esp_err_to_name(err));
            return false;
        }
        done += n;
    }
    return true;
}

void EspSpiBus::enable_ints()
{
    if (!isr_added_) {
        return;
    }
    gpio_intr_enable(pins_.intr);
    if (gpio_get_level(pins_.intr) == 0) {
        hal_.on_intn(time_now_us());
        xSemaphoreGive(intn_);
    }
}

void EspSpiBus::disable_ints()
{
    if (isr_added_) {
        gpio_intr_disable(pins_.intr);
    }
}

void EspSpiBus::wait_int(std::uint32_t timeout_us)
{
    if (intn_ != nullptr && timeout_us > 0) {
        xSemaphoreTake(intn_, ticks_for_us(timeout_us));
    }
}

void EspSpiBus::delay_us(std::uint32_t us) { vTaskDelay(ticks_for_us(us)); }

std::uint32_t EspSpiBus::time_now_us() { return static_cast<std::uint32_t>(esp_timer_get_time()); }

// ---------------------------------------------------------------------------
// Hal
// ---------------------------------------------------------------------------

Hal::Hal(const Pins& pins)
    : spi_(bus_), bus_(pins, spi_),
      glue_{.hal = {.open = open, .close = close, .read = read, .write = write, .getTimeUs = get_time_us},
            .self = this}
{
}

int Hal::open(sh2_Hal_t* hal) { return from(hal).spi_.open(); }
void Hal::close(sh2_Hal_t* hal) { from(hal).spi_.close(); }
int Hal::read(sh2_Hal_t* hal, std::uint8_t* buffer, unsigned len, std::uint32_t* t_us)
{
    return from(hal).spi_.read(buffer, len, t_us);
}
int Hal::write(sh2_Hal_t* hal, std::uint8_t* buffer, unsigned len) { return from(hal).spi_.write(buffer, len); }
std::uint32_t Hal::get_time_us(sh2_Hal_t* hal) { return from(hal).spi_.get_time_us(); }

} // namespace ping::bno085
