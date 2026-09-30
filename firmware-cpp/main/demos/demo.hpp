/// @file
/// What the single-sensor demos share: start the Arduino core and one of
/// the application's sensor tasks, then print its readings as they arrive.
///
/// The demos run the same task code as the application, so a sensor that
/// works in its demo works in the application too. Output goes to the
/// console on the USB serial port, next to the task's own log lines.

#pragma once

#include <chrono>
#include <optional>

#include <Arduino.h>

#include "latest.hpp"
#include "rtos.hpp"

namespace ping::demo {

/// How often the demo looks for a new reading.
inline constexpr std::chrono::milliseconds kPollInterval{5};

/// Starts the Arduino core and a sensor task with `start`, then calls
/// `print` with each new reading in `slot`, but no more often than
/// `min_interval`. Never returns.
template <typename T, typename Print>
[[noreturn]] void run(void (*start)(), const Latest<T>& slot, std::chrono::milliseconds min_interval,
                      Print print)
{
    initArduino();
    start();

    std::optional<std::chrono::microseconds> last_seen;
    std::optional<std::chrono::microseconds> last_printed;
    for (;;) {
        rtos::sleep(kPollInterval);
        const auto reading = slot.peek();
        if (!reading || reading->at == last_seen) {
            continue;
        }
        last_seen = reading->at;
        if (last_printed && reading->at - *last_printed < min_interval) {
            continue;
        }
        last_printed = reading->at;
        print(reading->value);
    }
}

}  // namespace ping::demo
