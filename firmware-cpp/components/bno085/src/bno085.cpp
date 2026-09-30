#include "ping/bno085.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <utility>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"

extern "C" {
#include "sh2_SensorValue.h"
#include "sh2_err.h"
}

namespace ping::bno085 {

namespace {

constexpr const char* kTag = "bno085";

/// Longest a single I2C transaction may take, clock stretching included.
constexpr int kBusTimeoutMs = 50;

/// How long the hub may hold SCL low. The BNO08x stretches the clock while
/// it prepares data, longer than ESP-IDF's default allows.
constexpr std::uint32_t kClockStretchUs = 10'000;

/// How long the HAL's read blocks on INT before reporting "nothing yet".
/// SH-2 polls the HAL in a loop while it waits for a response, and this is
/// what turns that loop from a spin into a sleep.
constexpr TickType_t kIdleWait = pdMS_TO_TICKS(2);

/// The hub takes about 100 ms to boot after a reset; this leaves margin, as
/// CEVA's and Adafruit's reference HALs do.
constexpr TickType_t kResetDelay = pdMS_TO_TICKS(300);
constexpr int kResetAttempts = 5;
constexpr TickType_t kResetRetryDelay = pdMS_TO_TICKS(30);

/// Most packets poll() reads before giving other tasks a turn. At 100 Hz the
/// hub has one or two waiting; a hub stuck holding INT low would otherwise
/// keep poll() spinning.
constexpr int kMaxPacketsPerPoll = 32;

/// SHTP's four byte header: length (little endian, top bit marks a
/// continuation), channel, sequence number.
constexpr std::size_t kHeaderLen = 4;

/// A reset command on SHTP channel 1, the executable channel.
constexpr std::array<std::uint8_t, 5> kSoftReset{5, 0, 1, 0, 1};

bool g_instance_exists = false;

/// esp_timer time of the latest falling edge on INT, in the 32 bits SH-2
/// wants. Written from the ISR, so kept outside the class as a lock-free
/// atomic rather than behind a mutex.
std::atomic<std::uint32_t> g_interrupt_time_us{0};

}  // namespace

const char* Error::message() const
{
    switch (code) {
    case SH2_OK: return "ok";
    case SH2_ERR: return "general error";
    case SH2_ERR_BAD_PARAM: return "bad parameter";
    case SH2_ERR_OP_IN_PROGRESS: return "operation in progress";
    case SH2_ERR_IO: return "I2C error";
    case SH2_ERR_HUB: return "error reported by hub";
    case SH2_ERR_TIMEOUT: return "timed out";
    default: return "unknown error";
    }
}

const char* to_string(Accuracy accuracy)
{
    switch (accuracy) {
    case Accuracy::kUnreliable: return "unreliable";
    case Accuracy::kLow: return "low";
    case Accuracy::kMedium: return "medium";
    case Accuracy::kHigh: return "high";
    }
    return "?";
}

const char* to_string(ResetCause cause)
{
    switch (cause) {
    case ResetCause::kNotApplicable: return "not applicable";
    case ResetCause::kPowerOn: return "power on";
    case ResetCause::kInternalSystemReset: return "internal system reset";
    case ResetCause::kWatchdogTimeout: return "watchdog timeout";
    case ResetCause::kExternalReset: return "external reset";
    case ResetCause::kOther: return "other";
    }
    return "?";
}

Bno085::Bno085(const Config& config)
    : hal_{.hal = {.open = hal_open,
                   .close = hal_close,
                   .read = hal_read,
                   .write = hal_write,
                   .getTimeUs = hal_time_us},
           .self = this},
      interrupt_pin_{config.interrupt}
{
    assert(!g_instance_exists && "CEVA's SH-2 library supports one hub at a time");
    g_instance_exists = true;

    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = config.port;
    bus_config.sda_io_num = config.sda;
    bus_config.scl_io_num = config.scl;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    // Breakouts carry their own pull-ups; these only help.
    bus_config.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_));

    i2c_device_config_t device_config{};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = static_cast<std::uint16_t>(config.address);
    device_config.scl_speed_hz = config.bus_hz;
    device_config.scl_wait_us = kClockStretchUs;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_, &device_config, &device_));

    interrupt_signal_ = xSemaphoreCreateBinary();
    assert(interrupt_signal_ != nullptr);

    // INT is open drain and active low, so it needs the pull-up.
    gpio_config_t pin_config{};
    pin_config.pin_bit_mask = 1ULL << config.interrupt;
    pin_config.mode = GPIO_MODE_INPUT;
    pin_config.pull_up_en = GPIO_PULLUP_ENABLE;
    pin_config.intr_type = GPIO_INTR_NEGEDGE;
    ESP_ERROR_CHECK(gpio_config(&pin_config));

    // Another driver may have installed the shared GPIO ISR service already.
    const esp_err_t installed = gpio_install_isr_service(0);
    if (installed != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(installed);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(config.interrupt, on_interrupt, this));
}

Bno085::~Bno085()
{
    stop();
    gpio_isr_handler_remove(interrupt_pin_);
    vSemaphoreDelete(interrupt_signal_);
    i2c_master_bus_rm_device(device_);
    i2c_del_master_bus(bus_);
    g_instance_exists = false;
}

Result<ProductId> Bno085::start()
{
    stop();
    orientation_.reset();

    if (const int status = sh2_open(&hal_.hal, on_async_event, this); status != SH2_OK) {
        return std::unexpected(Error{status});
    }
    open_ = true;
    sh2_setSensorCallback(on_sensor_event, this);

    sh2_ProductIds_t ids{};
    if (const int status = sh2_getProdIds(&ids); status != SH2_OK) {
        return std::unexpected(Error{status});
    }
    if (ids.numEntries == 0) {
        return std::unexpected(Error{SH2_ERR});
    }

    // The reset start() itself caused has been announced by now; only a later
    // one means trouble.
    reset_seen_ = false;

    const sh2_ProductId_t& id = ids.entry[0];
    const auto cause = id.resetCause <= static_cast<std::uint8_t>(ResetCause::kExternalReset)
                           ? static_cast<ResetCause>(id.resetCause)
                           : ResetCause::kOther;
    return ProductId{
        .reset_cause = cause,
        .version_major = id.swVersionMajor,
        .version_minor = id.swVersionMinor,
        .version_patch = id.swVersionPatch,
        .part_number = id.swPartNumber,
        .build_number = id.swBuildNumber,
    };
}

void Bno085::stop()
{
    if (open_) {
        sh2_close();
        open_ = false;
    }
}

Result<> Bno085::enable_rotation_vector(std::uint32_t interval_us)
{
    sh2_SensorConfig_t config{};
    config.reportInterval_us = interval_us;
    if (const int status = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config); status != SH2_OK) {
        return std::unexpected(Error{status});
    }
    return {};
}

bool Bno085::poll(TickType_t timeout)
{
    if (!wait_for_interrupt(timeout)) {
        return false;
    }
    // The hub holds INT low until everything waiting has been read. Each
    // sh2_service() reads one packet and hands its reports to the callbacks.
    int packets = 0;
    do {
        sh2_service();
    } while (open_ && interrupt_asserted() && ++packets < kMaxPacketsPerPoll);
    return true;
}

std::optional<Orientation> Bno085::take_orientation()
{
    return std::exchange(orientation_, std::nullopt);
}

bool Bno085::take_reset()
{
    return std::exchange(reset_seen_, false);
}

Bno085& Bno085::from(sh2_Hal_t* hal)
{
    // hal is the first member of a standard-layout Hal, so the two share an
    // address.
    return *reinterpret_cast<Hal*>(hal)->self;
}

bool Bno085::interrupt_asserted() const
{
    return gpio_get_level(interrupt_pin_) == 0;
}

bool Bno085::wait_for_interrupt(TickType_t timeout)
{
    const TickType_t start = xTaskGetTickCount();
    for (;;) {
        if (interrupt_asserted()) {
            return true;
        }
        const TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout) {
            return false;
        }
        // An edge between the level check and here leaves the semaphore
        // given, so this returns at once and the loop sees INT low. A stale
        // edge from earlier just costs one more trip round the loop.
        xSemaphoreTake(interrupt_signal_, timeout - elapsed);
    }
}

int Bno085::soft_reset()
{
    for (int attempt = 0; attempt < kResetAttempts; ++attempt) {
        if (i2c_master_transmit(device_, kSoftReset.data(), kSoftReset.size(), kBusTimeoutMs) == ESP_OK) {
            vTaskDelay(kResetDelay);
            return SH2_OK;
        }
        vTaskDelay(kResetRetryDelay);
    }
    ESP_LOGW(kTag, "no answer at the hub's I2C address");
    return SH2_ERR_IO;
}

int Bno085::read_packet(std::uint8_t* buffer, unsigned len, std::uint32_t* t_us)
{
    // Every read starts with the packet's header, so read that first to learn
    // how long the packet is, then read the whole packet, header again and
    // all, in a second transaction.
    std::array<std::uint8_t, kHeaderLen> header{};
    if (i2c_master_receive(device_, header.data(), header.size(), kBusTimeoutMs) != ESP_OK) {
        return SH2_ERR_IO;
    }
    const unsigned packet_len = (header[0] | (header[1] << 8)) & 0x7FFFU;
    if (packet_len <= kHeaderLen) {
        return 0;
    }
    if (packet_len > len) {
        // Larger than SH-2's receive buffer, which is sized for anything the
        // hub sends. Read what fits so the hub moves on, and drop it.
        ESP_LOGW(kTag, "dropping a %u byte packet", packet_len);
        i2c_master_receive(device_, buffer, len, kBusTimeoutMs);
        return 0;
    }
    if (i2c_master_receive(device_, buffer, packet_len, kBusTimeoutMs) != ESP_OK) {
        return SH2_ERR_IO;
    }
    *t_us = g_interrupt_time_us.load(std::memory_order_relaxed);
    return static_cast<int>(packet_len);
}

int Bno085::hal_open(sh2_Hal_t* hal)
{
    Bno085& self = from(hal);
    // Forget edges from before the reset.
    xSemaphoreTake(self.interrupt_signal_, 0);
    return self.soft_reset();
}

void Bno085::hal_close(sh2_Hal_t* /*hal*/)
{
    // With no RST line wired there is nothing to hold the hub in; the next
    // open() resets it over I2C.
}

int Bno085::hal_read(sh2_Hal_t* hal, std::uint8_t* buffer, unsigned len, std::uint32_t* t_us)
{
    Bno085& self = from(hal);
    if (!self.wait_for_interrupt(kIdleWait)) {
        return 0;
    }
    return self.read_packet(buffer, len, t_us);
}

int Bno085::hal_write(sh2_Hal_t* hal, std::uint8_t* buffer, unsigned len)
{
    Bno085& self = from(hal);
    if (i2c_master_transmit(self.device_, buffer, len, kBusTimeoutMs) != ESP_OK) {
        return SH2_ERR_IO;
    }
    return static_cast<int>(len);
}

std::uint32_t Bno085::hal_time_us(sh2_Hal_t* /*hal*/)
{
    return static_cast<std::uint32_t>(esp_timer_get_time());
}

void Bno085::on_async_event(void* cookie, sh2_AsyncEvent_t* event)
{
    auto& self = *static_cast<Bno085*>(cookie);
    if (event->eventId == SH2_RESET) {
        self.reset_seen_ = true;
    } else if (event->eventId == SH2_SHTP_EVENT) {
        ESP_LOGD(kTag, "SHTP event %u", event->shtpEvent);
    }
}

void Bno085::on_sensor_event(void* cookie, sh2_SensorEvent_t* event)
{
    auto& self = *static_cast<Bno085*>(cookie);
    sh2_SensorValue_t value{};
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        return;
    }
    if (value.sensorId == SH2_ROTATION_VECTOR) {
        const sh2_RotationVectorWAcc_t& rv = value.un.rotationVector;
        self.orientation_ = Orientation{
            .quaternion = {.i = rv.i, .j = rv.j, .k = rv.k, .real = rv.real},
            .heading_accuracy_rad = rv.accuracy,
            .calibration = static_cast<Accuracy>(value.status & 0x03U),
        };
    }
}

void IRAM_ATTR Bno085::on_interrupt(void* arg)
{
    auto& self = *static_cast<Bno085*>(arg);
    g_interrupt_time_us.store(static_cast<std::uint32_t>(esp_timer_get_time()),
                              std::memory_order_relaxed);
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(self.interrupt_signal_, &woken);
    portYIELD_FROM_ISR(woken);
}

}  // namespace ping::bno085
