/// @file
/// Board wiring and fixed settings shared by every application.
///
/// Everything in here is decided once, by the hardware or by the design, and
/// does not change while the firmware runs. Change a pin or a setting here and
/// every application that uses it follows. It mirrors `src/constants.rs` in
/// the Rust firmware; keep the two in step.
///
/// # Pin map
///
/// For the ESP32-S3-DevKitC-1-N8R8, following the project's pin map
/// spreadsheet (`PING_Pin_Map.xlsx`). Every sensor wires to header J1; the
/// display's two signal pins are on J3. Board TX goes to the sensor's RX and
/// board RX to the sensor's TX. All grounds are common.
///
/// | Device  | Signal              | GPIO | Header | Bus                    |
/// |---------|---------------------|------|--------|------------------------|
/// | Button  | trigger, to GND     | 4    | J1-4   | input, pull-up         |
/// | NEO-M9N | RESET_N (optional)  | 6    | J1-6   | output                 |
/// | NEO-M9N | TIMEPULSE (optional)| 7    | J1-7   | input                  |
/// | NEO-M9N | RXD                 | 15   | J1-8   | UART2 TX, 38400 8N1    |
/// | NEO-M9N | TXD                 | 16   | J1-9   | UART2 RX, 38400 8N1    |
/// | TF03    | blue, UART RxD      | 17   | J1-10  | UART1 TX, 115200 8N1   |
/// | TF03    | brown, UART TxD     | 18   | J1-11  | UART1 RX, 115200 8N1   |
/// | BNO085  | P0 (PS0 / WAKE)     | 8    | J1-12  | output                 |
/// | BNO085  | RST                 | 9    | J1-15  | output, active low     |
/// | BNO085  | CS                  | 10   | J1-16  | SPI2 CS, mode 3        |
/// | BNO085  | DI (MOSI)           | 11   | J1-17  | SPI2 MOSI              |
/// | BNO085  | SCL (SCK in SPI)    | 12   | J1-18  | SPI2 SCK, 1 MHz        |
/// | BNO085  | SDA (MISO in SPI)   | 13   | J1-19  | SPI2 MISO              |
/// | BNO085  | INT                 | 14   | J1-20  | input, pull-up         |
/// | Display | SDA                 | 1    | J3-4   | I2C0 SDA, 400 kHz      |
/// | Display | SCL                 | 2    | J3-5   | I2C0 SCL, 400 kHz      |
///
/// See `src/constants.rs` for power, the pins kept clear on purpose and the
/// TF03 cable colours.
///
/// The display is a 128 by 64 OLED module with an SH1106 controller, at I2C
/// address 0x3C.

#pragma once

#include "ping/geo.hpp"

#include <driver/gpio.h>
#include <driver/i2c_types.h>
#include <driver/spi_common.h>
#include <driver/uart.h>

#include <cstdint>

namespace ping {

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------

/// Trigger button, wired to ground; uses the internal pull-up. GPIO0 (BOOT)
/// is avoided because holding it during a reset starts the ROM bootloader.
inline constexpr gpio_num_t kButton = GPIO_NUM_4;

// NEO-M9N GPS. Board TX to the GPS's RXD, board RX to its TXD.
inline constexpr gpio_num_t kGpsTx = GPIO_NUM_15;
inline constexpr gpio_num_t kGpsRx = GPIO_NUM_16;
// Optional: hard reset (active low) and the one-pulse-per-second output.
inline constexpr gpio_num_t kGpsReset = GPIO_NUM_6;
inline constexpr gpio_num_t kGpsTimepulse = GPIO_NUM_7;

// TF03 LiDAR. Board TX to the blue UART RxD wire, board RX to the brown UART
// TxD wire.
inline constexpr gpio_num_t kLidarTx = GPIO_NUM_17;
inline constexpr gpio_num_t kLidarRx = GPIO_NUM_18;

// BNO085 IMU on SPI2's dedicated (IO MUX) pins. Comments name the Adafruit
// breakout's printed labels.
/// Breakout CS.
inline constexpr gpio_num_t kImuCs = GPIO_NUM_10;
/// Breakout DI: data into the sensor.
inline constexpr gpio_num_t kImuMosi = GPIO_NUM_11;
/// Breakout SCL, which is the SPI clock in SPI mode.
inline constexpr gpio_num_t kImuSck = GPIO_NUM_12;
/// Breakout SDA, which is data out of the sensor in SPI mode.
inline constexpr gpio_num_t kImuMiso = GPIO_NUM_13;
/// Breakout INT: active-low data-ready line.
inline constexpr gpio_num_t kImuInt = GPIO_NUM_14;
/// Breakout RST: active-low reset.
inline constexpr gpio_num_t kImuReset = GPIO_NUM_9;
/// Breakout P0 (PS0/WAKE). Held high so the hub comes up in SPI mode; the
/// esp32_BNO08x library does not use it as a wake line.
inline constexpr gpio_num_t kImuWake = GPIO_NUM_8;

// I2C display, run from 3V3.
inline constexpr gpio_num_t kLcdSda = GPIO_NUM_1;
inline constexpr gpio_num_t kLcdScl = GPIO_NUM_2;

// ---------------------------------------------------------------------------
// Buses
// ---------------------------------------------------------------------------

// UART0 stays with the USB-to-UART bridge.
inline constexpr uart_port_t kLidarUart = UART_NUM_1;
inline constexpr uart_port_t kGpsUart = UART_NUM_2;
inline constexpr spi_host_device_t kImuSpi = SPI2_HOST;
inline constexpr i2c_port_num_t kLcdI2c = I2C_NUM_0;

/// The TF03's factory UART baud rate (8N1).
inline constexpr int kLidarBaudRate = 115'200;

/// The NEO-M9N's factory UART1 baud rate (8N1).
inline constexpr int kGpsBaudRate = 38'400;

/// SPI clock for the BNO085, in hertz. The datasheet's SPI timing gives room
/// above this; 1 MHz is a conservative starting point to raise once the link
/// is proven.
inline constexpr std::uint32_t kImuSpiFrequencyHz = 1'000'000;

/// I2C clock for the display, in hertz: the SSD1306's fast mode maximum.
inline constexpr std::uint32_t kLcdI2cFrequencyHz = 400'000;

/// The display's I2C address: 0x3C on most modules, 0x3D with the address
/// resistor on the back moved.
inline constexpr std::uint8_t kLcdAddress = 0x3C;

/// Which controller the display has. u8g2 knows where each one's glass sits
/// in its memory, so this replaces the Rust firmware's column offset: an
/// SH1106 there is a column offset of 2, an SSD1306 one of 0.
enum class LcdController : std::uint8_t { kSsd1306, kSh1106 };
inline constexpr LcdController kLcdController = LcdController::kSh1106;

// ---------------------------------------------------------------------------
// Sensor rates
// ---------------------------------------------------------------------------

/// How many positions a second the GPS computes.
inline constexpr std::uint8_t kGpsRateHz = 10;

/// How often the IMU reports its orientation: every 10 ms, i.e. 100 Hz.
inline constexpr std::uint32_t kImuReportIntervalUs = 10'000;

// ---------------------------------------------------------------------------
// Aiming
// ---------------------------------------------------------------------------

/// The direction the LiDAR looks, in the IMU's frame: the axes printed on the
/// BNO085 breakout. Set this to match how the two are mounted. The default is
/// +Y because that is where the BNO085 itself points: lying level with +Y
/// towards magnetic north, it reports no rotation at all.
inline constexpr geo::Vec3 kLidarBoresight{.x = 0.0F, .y = 1.0F, .z = 0.0F};

/// How far magnetic north lies east of true north where the device is used,
/// in degrees, negative for west. The IMU measures bearings from magnetic
/// north, but coordinates are laid out from true north. Look the value up at
/// <https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml>.
inline constexpr float kMagneticDeclinationDeg = 0.0F;

// ---------------------------------------------------------------------------
// Timing, in milliseconds
// ---------------------------------------------------------------------------

// Readings older than these when the button is pressed are not used. The GPS
// allowance covers a few missed solutions; the IMU and LiDAR report 100 times
// a second, so a quarter of a second without a report means something is
// wrong.
inline constexpr std::uint32_t kMaxGpsAgeMs = 2'000;
inline constexpr std::uint32_t kMaxImuAgeMs = 250;
inline constexpr std::uint32_t kMaxLidarAgeMs = 250;

/// A GPS this long without a solution has probably lost power, and with it
/// its configuration, which lives in RAM. The GPS task then configures it
/// again.
inline constexpr std::uint32_t kGpsSilenceMs = 3'000;

/// An IMU this long without a report has probably reset, which stops every
/// report. The IMU task then starts it again.
inline constexpr std::uint32_t kImuSilenceMs = 1'000;

/// A TF03 this long without a measurement is not sending any: it streams 100
/// a second.
inline constexpr std::uint32_t kLidarSilenceMs = 1'000;

/// How long the button has to stay down to count as pressed, or up to count
/// as released: longer than a switch's contact bounce, shorter than a tap.
inline constexpr std::uint32_t kDebounceMs = 30;

/// How often the demos rewrite the display. A reading that changes faster
/// than this is a blur anyway.
inline constexpr std::uint32_t kLcdRefreshMs = 200;

/// How long to wait before retrying a sensor that failed to come up.
inline constexpr std::uint32_t kRetryDelayMs = 1'000;

} // namespace ping
