/// @file
/// Board wiring and sensor rates for the ESP32-S3-DevKitC-1-N8R8.
///
/// Pins avoided on purpose: GPIO26-32 (SPI flash), GPIO33-37 (octal PSRAM on
/// the N8R8), GPIO0/3/45/46 (strapping), GPIO19/20 (native USB), and
/// GPIO43/44 (UART0, the console).

#pragma once

#include <cstdint>

namespace ping::config {

// BNO085 on I2C — same pins as the Rust imu_demo.
inline constexpr int kImuSda = 8;
inline constexpr int kImuScl = 9;
inline constexpr int kImuReset = 11; ///< Active low; lets us recover a wedged hub.
inline constexpr std::uint32_t kImuI2cHz = 400'000;
inline constexpr std::uint8_t kImuAddress = 0x4A; ///< 0x4B if the DI/ADR pad is pulled high.
inline constexpr std::uint32_t kImuReportIntervalUs = 10'000; ///< 100 Hz

// Benewake TF03 on UART1. Its UART is 3.3 V LVTTL; power it from 5-24 V.
inline constexpr int kLidarUart = 1;
inline constexpr int kLidarRx = 18; ///< ← TF03 TX
inline constexpr int kLidarTx = 17; ///< → TF03 RX
inline constexpr std::uint32_t kLidarBaud = 115'200;

// u-blox NEO-M9N on UART2. The SparkFun breakout ships at 38400 baud.
inline constexpr int kGnssUart = 2;
inline constexpr int kGnssRx = 16; ///< ← NEO-M9N TX
inline constexpr int kGnssTx = 15; ///< → NEO-M9N RX
inline constexpr std::uint32_t kGnssBaud = 38'400;
inline constexpr std::uint8_t kGnssNavRateHz = 10;

// ROS 2 names (REP-105 frames).
inline constexpr const char* kNodeName = "ping_sensor_node";
inline constexpr const char* kImuTopic = "imu/data";
inline constexpr const char* kRangeTopic = "lidar/range";
inline constexpr const char* kFixTopic = "gps/fix";
inline constexpr const char* kImuFrame = "imu_link";
inline constexpr const char* kLidarFrame = "lidar_link";
inline constexpr const char* kGnssFrame = "gps_link";

} // namespace ping::config
