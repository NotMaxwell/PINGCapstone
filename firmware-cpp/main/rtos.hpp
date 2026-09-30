/// @file
/// Small conveniences over FreeRTOS and ESP-IDF: chrono durations for
/// ticks and the clock, and task creation.

#pragma once

#include <chrono>
#include <cstdint>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace ping::rtos {

/// Time since boot, from esp_timer.
inline std::chrono::microseconds now()
{
    return std::chrono::microseconds{esp_timer_get_time()};
}

/// A duration in FreeRTOS ticks, rounded up so short waits never become
/// zero.
constexpr TickType_t ticks(std::chrono::milliseconds duration)
{
    const auto count = duration.count();
    return static_cast<TickType_t>((count * configTICK_RATE_HZ + 999) / 1000);
}

inline void sleep(std::chrono::milliseconds duration)
{
    vTaskDelay(ticks(duration));
}

/// Runs `body` in a new FreeRTOS task.
template <auto Body>
void spawn(const char* name, std::uint32_t stack_bytes, UBaseType_t priority)
{
    const BaseType_t created = xTaskCreate(
        [](void* /*context*/) {
            Body();
            vTaskDelete(nullptr);
        },
        name, stack_bytes, nullptr, priority, nullptr);
    configASSERT(created == pdPASS);
}

}  // namespace ping::rtos
