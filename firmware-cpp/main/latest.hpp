/// @file
/// The latest reading from one sensor, and when it arrived. The sensor's
/// task overwrites it; a button press copies it out.

#pragma once

#include <chrono>
#include <expected>
#include <optional>

#include "freertos/FreeRTOS.h"
#include "rtos.hpp"

namespace ping {

/// A sensor that has not reported recently, or at all.
struct Stale {
    const char* sensor;
    /// How old its last reading is, if it has ever sent one.
    std::optional<std::chrono::milliseconds> age;
};

template <typename T>
class Latest {
public:
    explicit Latest(const char* sensor) : sensor_{sensor} {}

    Latest(const Latest&) = delete;
    Latest& operator=(const Latest&) = delete;

    void set(const T& value)
    {
        const Reading reading{.at = rtos::now(), .value = value};
        taskENTER_CRITICAL(&lock_);
        reading_ = reading;
        taskEXIT_CRITICAL(&lock_);
    }

    /// The reading, as long as it is no older than `max_age`.
    std::expected<T, Stale> get(std::chrono::milliseconds max_age) const
    {
        taskENTER_CRITICAL(&lock_);
        const std::optional<Reading> reading = reading_;
        taskEXIT_CRITICAL(&lock_);

        if (!reading) {
            return std::unexpected(Stale{.sensor = sensor_, .age = std::nullopt});
        }
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(rtos::now() - reading->at);
        if (age > max_age) {
            return std::unexpected(Stale{.sensor = sensor_, .age = age});
        }
        return reading->value;
    }

private:
    struct Reading {
        std::chrono::microseconds at;
        T value;
    };

    const char* sensor_;
    // A spinlock critical section rather than a mutex: the copy inside is a
    // hundred bytes at most, and it works from either core.
    mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
    std::optional<Reading> reading_;
};

}  // namespace ping
