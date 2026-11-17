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
#include <cstring>

namespace ping::bno085 {

namespace {

constexpr const char* kTag = "bno085";

static_assert(shtp::kMaxCargo == SH2_HAL_MAX_TRANSFER_IN);
static_assert(shtp::kMaxWrite == SH2_HAL_MAX_TRANSFER_OUT);

// Without DMA a transaction is limited to the 64-byte hardware FIFO, but any
// buffer works: with DMA, ESP-IDF would copy unaligned or odd-length buffers
// through a heap-allocated bounce buffer on every transfer.
constexpr std::size_t kChunk = 64;

const std::array<std::uint8_t, kChunk> kZeros{};
std::array<std::uint8_t, kChunk> g_sink{};

TickType_t ticks_for(std::uint32_t ms)
{
    const TickType_t ticks = pdMS_TO_TICKS(ms);
    return ms > 0 && ticks == 0 ? 1 : ticks;
}

std::uint32_t level(bool high) { return high ? 1U : 0U; }

} // namespace

// ---------------------------------------------------------------------------
// EspSpiBus
// ---------------------------------------------------------------------------

void IRAM_ATTR EspSpiBus::on_int(void* arg)
{
    auto* self = static_cast<EspSpiBus*>(arg);
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(self->int_falling_, &woken);
    portYIELD_FROM_ISR(woken);
}

esp_err_t EspSpiBus::init()
{
    deinit();

    if (int_falling_ == nullptr) {
        int_falling_ = xSemaphoreCreateBinaryStatic(&int_falling_storage_);
    }

    // Levels are latched before the pins become outputs, so CS and WAKE never
    // glitch low. PS0 (WAKE) high is what selects SPI at the next reset.
    gpio_set_level(pins_.cs, 1);
    gpio_set_level(pins_.wake, 1);
    gpio_set_level(pins_.reset, 1);
    gpio_config_t outputs{};
    outputs.pin_bit_mask = (1ULL << pins_.cs) | (1ULL << pins_.wake) | (1ULL << pins_.reset);
    outputs.mode = GPIO_MODE_OUTPUT;
    outputs.intr_type = GPIO_INTR_DISABLE;
    esp_err_t err = gpio_config(&outputs);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "control pins: %s", esp_err_to_name(err));
        return err;
    }
    gpio_set_level(pins_.cs, 1);
    gpio_set_level(pins_.wake, 1);
    gpio_set_level(pins_.reset, 1);

    gpio_config_t input{};
    input.pin_bit_mask = 1ULL << pins_.intr;
    input.mode = GPIO_MODE_INPUT;
    input.pull_up_en = GPIO_PULLUP_ENABLE;
    input.intr_type = GPIO_INTR_NEGEDGE;
    err = gpio_config(&input);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "INT pin: %s", esp_err_to_name(err));
        return err;
    }

    // Another driver may have installed the shared ISR service already; that
    // is fine, and it is never uninstalled here for the same reason.
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(kTag, "GPIO ISR service: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_isr_handler_add(pins_.intr, on_int, this);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "INT handler: %s", esp_err_to_name(err));
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
    device.mode = 3;
    device.clock_speed_hz = static_cast<int>(pins_.frequency_hz);
    // CS is driven by hand: a read must see the header before it knows how
    // long to keep CS low.
    device.spics_io_num = -1;
    device.queue_size = 1;
    err = spi_bus_add_device(pins_.host, &device, &device_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "SPI device: %s", esp_err_to_name(err));
        device_ = nullptr;
        return err;
    }
    return ESP_OK;
}

void EspSpiBus::deinit()
{
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
    if (isr_added_) {
        if (const esp_err_t err = gpio_isr_handler_remove(pins_.intr); err != ESP_OK) {
            ESP_LOGW(kTag, "removing INT handler: %s", esp_err_to_name(err));
        }
        isr_added_ = false;
    }
}

void EspSpiBus::set_cs(bool high) { gpio_set_level(pins_.cs, level(high)); }
void EspSpiBus::set_wake(bool high) { gpio_set_level(pins_.wake, level(high)); }
void EspSpiBus::set_reset(bool high) { gpio_set_level(pins_.reset, level(high)); }
bool EspSpiBus::int_high() { return gpio_get_level(pins_.intr) != 0; }

bool EspSpiBus::wait_int_low(std::uint32_t timeout_ms)
{
    if (!int_high()) {
        return true;
    }
    if (timeout_ms == 0 || int_falling_ == nullptr) {
        return false;
    }
    // Clear a stale edge, then look again before blocking, so an edge between
    // the first look and the take is not missed.
    xSemaphoreTake(int_falling_, 0);
    if (!int_high()) {
        return true;
    }
    const bool fell = xSemaphoreTake(int_falling_, ticks_for(timeout_ms)) == pdTRUE;
    return fell || !int_high();
}

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
        t.tx_buffer = tx != nullptr ? tx + done : kZeros.data();
        t.rx_buffer = rx != nullptr ? rx + done : g_sink.data();
        if (const esp_err_t err = spi_device_polling_transmit(device_, &t); err != ESP_OK) {
            ESP_LOGE(kTag, "SPI transfer: %s", esp_err_to_name(err));
            return false;
        }
        done += n;
    }
    return true;
}

void EspSpiBus::delay_ms(std::uint32_t ms) { vTaskDelay(ticks_for(ms)); }

std::uint32_t EspSpiBus::now_us() { return static_cast<std::uint32_t>(esp_timer_get_time()); }

// ---------------------------------------------------------------------------
// Hal
// ---------------------------------------------------------------------------

Hal::Hal(const Pins& pins)
    : bus_(pins), transport_(bus_),
      glue_{.hal = {.open = open, .close = close, .read = read, .write = write, .getTimeUs = get_time_us},
            .self = this}
{
}

void Hal::record(HalError error)
{
    if (!error_) {
        error_ = error;
    }
}

std::optional<HalError> Hal::take_error()
{
    auto error = error_;
    error_.reset();
    return error;
}

void Hal::arm_deadline(std::uint32_t timeout_ms)
{
    deadline_armed_ = true;
    aborted_ = false;
    deadline_start_us_ = bus_.now_us();
    deadline_us_ = timeout_ms * 1000U;
}

int Hal::open(sh2_Hal_t* hal)
{
    Hal& self = from(hal);
    if (const esp_err_t err = self.bus_.init(); err != ESP_OK) {
        self.record({.kind = HalError::Kind::kInit, .esp = err});
        return SH2_ERR_IO;
    }
    if (const auto reset = self.transport_.reset(); !reset) {
        ESP_LOGE(kTag, "no INT within %u ms of reset", static_cast<unsigned>(shtp::kBootTimeoutMs));
        self.record({.kind = HalError::Kind::kBootTimeout});
        return SH2_ERR_TIMEOUT;
    }
    return SH2_OK;
}

void Hal::close(sh2_Hal_t* hal)
{
    // Held in reset until the next open, as sh2_hal.h asks; the bus itself is
    // rebuilt by the next open.
    from(hal).bus_.set_reset(false);
}

int Hal::read(sh2_Hal_t* hal, std::uint8_t* buffer, unsigned len, std::uint32_t* t_us)
{
    Hal& self = from(hal);
    *t_us = self.bus_.now_us();

    if (self.deadline_armed_ && *t_us - self.deadline_start_us_ >= self.deadline_us_) {
        self.deadline_armed_ = false;
        self.aborted_ = true;
        // Executable channel, "reset complete": sh2 aborts its operation.
        constexpr std::array<std::uint8_t, 5> kResetComplete{5, 0, 1, 0, 1};
        if (len >= kResetComplete.size()) {
            std::memcpy(buffer, kResetComplete.data(), kResetComplete.size());
            return static_cast<int>(kResetComplete.size());
        }
    }

    const auto got = self.transport_.read({buffer, len}, self.read_wait_ms_);
    if (got) {
        return static_cast<int>(*got);
    }
    if (got.error().kind == shtp::Error::Kind::kCargoTooLarge) {
        ++self.discarded_;
        ESP_LOGW(kTag, "discarded a %u-byte cargo", static_cast<unsigned>(got.error().cargo_len));
    } else {
        self.record({.kind = HalError::Kind::kBus});
    }
    return 0;
}

int Hal::write(sh2_Hal_t* hal, std::uint8_t* buffer, unsigned len)
{
    Hal& self = from(hal);
    // A write would overwrite the stash with whatever it clocks in. Returning
    // 0 makes sh2 service (which reads the stash) and try again.
    if (self.transport_.has_stash()) {
        return 0;
    }
    const auto sent = self.transport_.write({buffer, len});
    if (sent) {
        return static_cast<int>(len);
    }
    switch (sent.error().kind) {
    case shtp::Error::Kind::kTimeout:
        self.record({.kind = HalError::Kind::kWakeTimeout});
        return SH2_ERR_TIMEOUT;
    case shtp::Error::Kind::kBadLength:
        return SH2_ERR_BAD_PARAM;
    case shtp::Error::Kind::kBus:
    case shtp::Error::Kind::kCargoTooLarge:
        break;
    }
    self.record({.kind = HalError::Kind::kBus});
    return SH2_ERR_IO;
}

std::uint32_t Hal::get_time_us(sh2_Hal_t* hal) { return from(hal).bus_.now_us(); }

} // namespace ping::bno085
