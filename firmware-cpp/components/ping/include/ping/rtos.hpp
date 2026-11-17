/// @file
/// Small FreeRTOS helpers the applications share: time, delays, tasks that
/// the task watchdog looks after, and the latest-reading slot each sensor task
/// writes and the button press reads.

#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>
#include <mutex>
#include <optional>

namespace ping::rtos {

/// Milliseconds since boot.
inline std::int64_t now_ms() { return esp_timer_get_time() / 1000; }

/// Blocks the calling task for at least `ms` milliseconds (and always for at
/// least one tick, so a zero delay still lets lower priority tasks run).
inline void delay_ms(std::uint32_t ms)
{
    const TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks > 0 ? ticks : 1);
}

/// Starts `entry` as a FreeRTOS task. Returns false, after logging why, if
/// there was no memory for it.
inline bool spawn(TaskFunction_t entry, const char* name, std::uint32_t stack_bytes, UBaseType_t priority,
                  void* arg = nullptr)
{
    if (xTaskCreate(entry, name, stack_bytes, arg, priority, nullptr) != pdPASS) {
        ESP_LOGE("rtos", "no memory to start task %s", name);
        return false;
    }
    return true;
}

/// Puts the calling task under the task watchdog: if it stops calling
/// @ref Watchdog::feed for `CONFIG_ESP_TASK_WDT_TIMEOUT_S`, the chip panics
/// and restarts. A driver call that hangs despite its timeouts then costs a
/// reboot instead of a device that silently stops reporting.
class Watchdog {
  public:
    Watchdog()
    {
        const esp_err_t err = esp_task_wdt_add(nullptr);
        subscribed_ = err == ESP_OK;
        if (!subscribed_) {
            ESP_LOGW("rtos", "task watchdog unavailable for %s: %s", pcTaskGetName(nullptr), esp_err_to_name(err));
        }
    }
    ~Watchdog()
    {
        if (subscribed_) {
            esp_task_wdt_delete(nullptr);
        }
    }
    Watchdog(const Watchdog&) = delete;
    Watchdog& operator=(const Watchdog&) = delete;

    void feed() const
    {
        if (subscribed_) {
            esp_task_wdt_reset();
        }
    }

  private:
    bool subscribed_ = false;
};

/// How old a reading was when asked for, for explaining a stale one.
struct Age {
    /// Milliseconds since the reading arrived; empty if none ever has.
    std::optional<std::int64_t> ms;
};

/// The latest reading from one sensor, and when it arrived. The sensor's task
/// overwrites it; the button press and the display copy it out.
template <typename T>
class Latest {
  public:
    explicit constexpr Latest(const char* sensor) : sensor_(sensor) {}

    void set(const T& value)
    {
        const std::lock_guard lock(mutex_);
        reading_ = value;
        at_ms_ = now_ms();
    }

    /// The reading, if it is no older than `max_age_ms`. Otherwise empty,
    /// with `age` (if given) saying how old the last one was.
    std::optional<T> get(std::uint32_t max_age_ms, Age* age = nullptr)
    {
        const std::lock_guard lock(mutex_);
        const std::int64_t elapsed = now_ms() - at_ms_;
        if (reading_ && elapsed <= static_cast<std::int64_t>(max_age_ms)) {
            return reading_;
        }
        if (age != nullptr) {
            age->ms = reading_ ? std::optional(elapsed) : std::nullopt;
        }
        return std::nullopt;
    }

    [[nodiscard]] const char* sensor() const { return sensor_; }

  private:
    const char* sensor_;
    std::mutex mutex_;
    std::optional<T> reading_;
    std::int64_t at_ms_ = 0;
};

} // namespace ping::rtos
