/// @file
/// Firmware entry point: bring up the Arduino core (the HAL the vendor
/// drivers use), then start one task per sensor plus the micro-ROS task.
///
/// This is the C++/ROS 2 counterpart of the Rust tree's src/bin/main.rs and
/// src/bin/imu_demo.rs.

#include "sensors.hpp"

#include <Arduino.h>

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

struct TaskSpec {
    TaskFunction_t entry;
    const char* name;
    std::uint32_t stack_bytes;
    UBaseType_t priority;
    BaseType_t core;
};

// Wi-Fi runs on core 0, so the networking task shares it and the sensor
// tasks get core 1 to themselves.
constexpr TaskSpec kTasks[] = {
    {ping::ros_task, "ros", 16 * 1024, 5, 0},
    {ping::imu_task, "imu", 6 * 1024, 6, 1},
    {ping::lidar_task, "lidar", 4 * 1024, 6, 1},
    {ping::gnss_task, "gnss", 6 * 1024, 4, 1},
};

} // namespace

extern "C" void app_main()
{
    initArduino();

    for (const auto& task : kTasks) {
        xTaskCreatePinnedToCore(task.entry, task.name, task.stack_bytes, nullptr, task.priority, nullptr,
                                task.core);
    }
}
