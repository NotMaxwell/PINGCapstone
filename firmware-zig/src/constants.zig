//! Board wiring and fixed settings shared by every application.
//!
//! Everything in here is decided once, by the hardware or by the design, and
//! does not change while the firmware runs. Change a pin or a setting here and
//! every application that uses it follows. It mirrors `src/constants.rs` in
//! the Rust firmware; keep the two in step.
//!
//! # Pin map
//!
//! For the ESP32-S3-DevKitC-1-N8R8, following the project's pin map
//! spreadsheet (`PING_Pin_Map.xlsx`). Every sensor wires to header J1; the
//! display's two signal pins are on J3. Board TX goes to the sensor's RX and
//! board RX to the sensor's TX. All grounds are common.
//!
//! | Device  | Signal              | GPIO | Header | Bus                    |
//! |---------|---------------------|------|--------|------------------------|
//! | Button  | trigger, to GND     | 4    | J1-4   | input, pull-up         |
//! | NEO-M9N | RESET_N (optional)  | 6    | J1-6   | output                 |
//! | NEO-M9N | TIMEPULSE (optional)| 7    | J1-7   | input                  |
//! | NEO-M9N | RXD                 | 15   | J1-8   | UART2 TX, 38400 8N1    |
//! | NEO-M9N | TXD                 | 16   | J1-9   | UART2 RX, 38400 8N1    |
//! | TF03    | blue, UART RxD      | 17   | J1-10  | UART1 TX, 115200 8N1   |
//! | TF03    | brown, UART TxD     | 18   | J1-11  | UART1 RX, 115200 8N1   |
//! | BNO085  | P0 (PS0 / WAKE)     | 8    | J1-12  | output                 |
//! | BNO085  | RST                 | 9    | J1-15  | output, active low     |
//! | BNO085  | CS                  | 10   | J1-16  | SPI2 CS, mode 3        |
//! | BNO085  | DI (MOSI)           | 11   | J1-17  | SPI2 MOSI              |
//! | BNO085  | SCL (SCK in SPI)    | 12   | J1-18  | SPI2 SCK, 1 MHz        |
//! | BNO085  | SDA (MISO in SPI)   | 13   | J1-19  | SPI2 MISO              |
//! | BNO085  | INT                 | 14   | J1-20  | input, pull-up         |
//! | Display | SDA                 | 1    | J3-4   | I2C0 SDA, 400 kHz      |
//! | Display | SCL                 | 2    | J3-5   | I2C0 SCL, 400 kHz      |
//!
//! See `src/constants.rs` for power, the pins kept clear on purpose and the
//! TF03 cable colours.
//!
//! The display is a 128 by 64 OLED module with an SH1106 controller, at I2C
//! address 0x3C.

const idf = @import("idf");
const geo = @import("geo.zig");

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------

/// Trigger button, wired to ground; uses the internal pull-up. GPIO0 (BOOT)
/// is avoided because holding it during a reset starts the ROM bootloader.
pub const button: idf.gpio_num_t = idf.GPIO_NUM_4;

// NEO-M9N GPS. Board TX to the GPS's RXD, board RX to its TXD.
pub const gps_tx: idf.gpio_num_t = idf.GPIO_NUM_15;
pub const gps_rx: idf.gpio_num_t = idf.GPIO_NUM_16;
// Optional: hard reset (active low) and the one-pulse-per-second output.
pub const gps_reset: idf.gpio_num_t = idf.GPIO_NUM_6;
pub const gps_timepulse: idf.gpio_num_t = idf.GPIO_NUM_7;

// TF03 LiDAR. Board TX to the blue UART RxD wire, board RX to the brown UART
// TxD wire.
pub const lidar_tx: idf.gpio_num_t = idf.GPIO_NUM_17;
pub const lidar_rx: idf.gpio_num_t = idf.GPIO_NUM_18;

// BNO085 IMU on SPI2's dedicated (IO MUX) pins. Comments name the Adafruit
// breakout's printed labels.
/// Breakout CS.
pub const imu_cs: idf.gpio_num_t = idf.GPIO_NUM_10;
/// Breakout DI: data into the sensor.
pub const imu_mosi: idf.gpio_num_t = idf.GPIO_NUM_11;
/// Breakout SCL, which is the SPI clock in SPI mode.
pub const imu_sck: idf.gpio_num_t = idf.GPIO_NUM_12;
/// Breakout SDA, which is data out of the sensor in SPI mode.
pub const imu_miso: idf.gpio_num_t = idf.GPIO_NUM_13;
/// Breakout INT: active-low data-ready line.
pub const imu_int: idf.gpio_num_t = idf.GPIO_NUM_14;
/// Breakout RST: active-low reset.
pub const imu_reset: idf.gpio_num_t = idf.GPIO_NUM_9;
/// Breakout P0 (PS0/WAKE). High as the hub leaves reset selects SPI; pulsed
/// low afterwards to wake the hub before a write.
pub const imu_wake: idf.gpio_num_t = idf.GPIO_NUM_8;

// I2C display, run from 3V3.
pub const lcd_sda: idf.gpio_num_t = idf.GPIO_NUM_1;
pub const lcd_scl: idf.gpio_num_t = idf.GPIO_NUM_2;

// ---------------------------------------------------------------------------
// Buses
// ---------------------------------------------------------------------------

// UART0 stays with the USB-to-UART bridge.
pub const lidar_uart: idf.uart_port_t = idf.UART_NUM_1;
pub const gps_uart: idf.uart_port_t = idf.UART_NUM_2;
pub const imu_spi: idf.spi_host_device_t = idf.SPI2_HOST;
pub const lcd_i2c: idf.i2c_port_num_t = idf.I2C_NUM_0;

/// The TF03's factory UART baud rate (8N1).
pub const lidar_baud_rate = 115_200;

/// The NEO-M9N's factory UART1 baud rate (8N1).
pub const gps_baud_rate = 38_400;

/// SPI clock for the BNO085, in hertz. The datasheet's SPI timing gives room
/// above this; 1 MHz is a conservative starting point to raise once the link
/// is proven.
pub const imu_spi_frequency_hz = 1_000_000;

/// I2C clock for the display, in hertz: the SSD1306's fast mode maximum.
pub const lcd_i2c_frequency_hz = 400_000;

/// The display's I2C address: 0x3C on most modules, 0x3D with the address
/// resistor on the back moved.
pub const lcd_address: u8 = 0x3C;

/// Which controller the display has. u8g2 knows where each one's glass sits
/// in its memory, so this replaces the Rust firmware's column offset: an
/// SH1106 there is a column offset of 2, an SSD1306 one of 0.
pub const LcdController = enum { ssd1306, sh1106 };
pub const lcd_controller: LcdController = .sh1106;

// ---------------------------------------------------------------------------
// Sensor rates
// ---------------------------------------------------------------------------

/// How many positions a second the GPS computes.
pub const gps_rate_hz: u8 = 10;

/// How often the IMU reports its orientation: every 10 ms, i.e. 100 Hz.
pub const imu_report_interval_us: u32 = 10_000;

// ---------------------------------------------------------------------------
// Aiming
// ---------------------------------------------------------------------------

/// The direction the LiDAR looks, in the IMU's frame: the axes printed on the
/// BNO085 breakout. Set this to match how the two are mounted. The default is
/// +Y because that is where the BNO085 itself points: lying level with +Y
/// towards magnetic north, it reports no rotation at all.
pub const lidar_boresight: geo.Vec3 = .{ .x = 0.0, .y = 1.0, .z = 0.0 };

/// How far magnetic north lies east of true north where the device is used,
/// in degrees, negative for west. The IMU measures bearings from magnetic
/// north, but coordinates are laid out from true north. Look the value up at
/// <https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml>.
pub const magnetic_declination_deg: f32 = 0.0;

// ---------------------------------------------------------------------------
// Timing, in milliseconds
// ---------------------------------------------------------------------------

// Readings older than these when the button is pressed are not used. The GPS
// allowance covers a few missed solutions; the IMU and LiDAR report 100 times
// a second, so a quarter of a second without a report means something is
// wrong.
pub const max_gps_age_ms: u32 = 2_000;
pub const max_imu_age_ms: u32 = 250;
pub const max_lidar_age_ms: u32 = 250;

/// A GPS this long without a solution has probably lost power, and with it its
/// configuration, which lives in RAM. The GPS task then configures it again.
pub const gps_silence_ms: u32 = 3_000;

/// An IMU this long without a report has probably reset, which stops every
/// report. The IMU task then starts it again.
pub const imu_silence_ms: u32 = 1_000;

/// A TF03 this long without a measurement is not sending any: it streams 100
/// a second.
pub const lidar_silence_ms: u32 = 1_000;

/// How long the button has to stay down to count as pressed, or up to count
/// as released: longer than a switch's contact bounce, shorter than a tap.
pub const debounce_ms: u32 = 30;

/// How often the demos rewrite the display. A reading that changes faster
/// than this is a blur anyway.
pub const lcd_refresh_ms: u32 = 200;

/// How long to wait before retrying a sensor that failed to come up.
pub const retry_delay_ms: u32 = 1_000;
