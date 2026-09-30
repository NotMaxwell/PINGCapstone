// Standalone demo for the BNO085 IMU: runs the application's IMU task on its
// own and prints the rotation vector. Build it with the imu-demo preset.
//
// The hub reports 100 times a second; printing every report would swamp the
// console, so this prints at most ten a second.

#include <chrono>
#include <cstdio>

#include "demos/demo.hpp"
#include "sensors.hpp"

extern "C" void app_main()
{
    using namespace ping;
    demo::run(sensors::start_imu, readings::imu, std::chrono::milliseconds{100},
              [](const Orientation& orientation) {
                  const geo::Quaternion& q = orientation.quaternion;
                  std::printf("quat i=%.4f j=%.4f k=%.4f r=%.4f (+/- %.3f rad, %s)\n",
                              static_cast<double>(q.i), static_cast<double>(q.j),
                              static_cast<double>(q.k), static_cast<double>(q.real),
                              static_cast<double>(orientation.heading_accuracy_rad),
                              to_string(orientation.calibration));
              });
}
