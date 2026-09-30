/// @file
/// One FreeRTOS task per sensor, each keeping its latest reading where a
/// button press can pick it up.

#pragma once

#include "latest.hpp"
#include "ping/bno085.hpp"
#include "ping/neo_m9n.hpp"
#include "ping/tf03.hpp"

namespace ping {

namespace readings {

extern Latest<neo_m9n::Solution> gps;
extern Latest<bno085::Orientation> imu;
extern Latest<tf03::Measurement> lidar;

}  // namespace readings

namespace sensors {

/// Keeps readings::gps up to date. Never returns.
void gps_task(neo_m9n::NeoM9n& gps);

/// Keeps readings::imu up to date. Never returns.
void imu_task(bno085::Bno085& imu);

/// Keeps readings::lidar up to date. Never returns.
void lidar_task(tf03::Tf03& lidar);

}  // namespace sensors

}  // namespace ping
