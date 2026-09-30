// Standalone demo for the TF03 LiDAR: runs the application's LiDAR task on
// its own and prints the measured distance. Build it with the lidar-demo
// preset.
//
// The TF03 streams 100 measurements a second; this prints one in ten or so.
// Nothing here changes the TF03's configuration, which it keeps in flash.

#include <chrono>
#include <cstdio>

#include "demos/demo.hpp"
#include "sensors.hpp"

extern "C" void app_main()
{
    using namespace ping;
    demo::run(sensors::start_lidar, readings::lidar, std::chrono::milliseconds{100}, [](const Range& range) {
        if (range.distance_cm) {
            std::printf("distance %u cm (strength %u)\n", *range.distance_cm, range.strength);
        } else {
            std::printf("out of range (raw %u cm, strength %u)\n", range.raw_distance_cm, range.strength);
        }
    });
}
