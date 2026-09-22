/// @file
/// Entry points for the sensor tasks. Each runs forever, owns its vendor
/// driver, and publishes readings into the mailboxes in samples.hpp.

#pragma once

namespace ping {

void imu_task(void* arg);
void lidar_task(void* arg);
void gnss_task(void* arg);
void ros_task(void* arg);

} // namespace ping
