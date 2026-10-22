//! Board wiring and fixed settings shared by every binary.
//!
//! Everything in here is decided once, by the hardware or by the design, and
//! does not change while the firmware runs. Change a pin or a setting here and
//! every binary that uses it follows.
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
//! Power and ground:
//!
//! | Device  | Pin                 | Board connection           | Header |
//! |---------|---------------------|----------------------------|--------|
//! | TF03    | red (VCC)           | fused 12 V line, after D1  | —      |
//! | NEO-M9N | VCC                 | 3V3                        | J1-1   |
//! | BNO085  | VIN, P1 (PS1)       | 3V3                        | J1-1   |
//! | Display | VCC                 | 3V3                        | J1-2   |
//! | Board   | 5V                  | buck converter, 5.1 V      | J1-21  |
//! | All     | GND, TF03 black     | common ground              | J1-22  |
//!
//! Left unconnected: the TF03's white (CAN_L) and green (CAN_H) wires; the NEO-M9N's D_SEL and SAFEBOOT_N, which the datasheet
//! says to leave open; and the BNO085 breakout's BT pin.
//!
//! Pins kept clear on purpose: GPIO0, 3, 45 and 46 are strapping pins read at
//! boot; GPIO19 and 20 are the USB port; GPIO43 and 44 are UART0; GPIO26–37
//! are the module's flash and octal PSRAM. GPIO5, 21 and 47 are free, and
//! GPIO5 can read analog voltages with Wi-Fi on, so it is the one to use for
//! battery monitoring.
//!
//! The TF03 is the 5–24 V UART/CAN version, whose six-wire cable follows
//! Benewake's 2024 *TF03 UART/CAN User Manual*. The older 5 V version and its
//! V1.0 manual use a seven-wire cable with brown as RxD and yellow as TxD;
//! on this one brown is TxD and blue is RxD, and there is no yellow wire.
//!
//! The TF03 takes 5–24 V, so it runs straight from the fused battery line,
//! keeping its 480 mA peak off the board's regulator. The board's own
//! regulator makes 3.3 V for the GPS, IMU and display, about 0.14 A in all.
//! F1 at the battery protects every wire after it, and D1 keeps a reversed
//! battery away from the LiDAR and the buck converter.
//!
//! The BNO085 signal names above are the labels printed on the Adafruit
//! BNO085 breakout. In SPI mode its SCL pin is the clock and SDA is data out
//! of the sensor. P0 and P1 both high selects SPI; P1 can take the board's
//! 3V3 or the breakout's own 3Vo pin, which is the same 3.3 V.
//!
//! The display is a 128 by 64 OLED module with an SH1106 controller, at I2C
//! address 0x3C. The pin map spreadsheet still describes a PCF8574 character
//! LCD at 0x27 on these pins; the pins are right, the device notes are not.
//!
//! # Using the pins
//!
//! esp-hal hands out each GPIO and peripheral as its own type, so a pin
//! cannot be stored in a `const`. [`board_pin!`](crate::board_pin) and
//! [`board_bus!`](crate::board_bus) name them instead, and expand to the
//! matching field of `esp_hal::peripherals::Peripherals`:
//!
//! ```ignore
//! let peripherals = esp_hal::init(config);
//! let uart = Uart::new(board_bus!(peripherals.LIDAR_UART), uart_config)?
//!     .with_tx(board_pin!(peripherals.LIDAR_TX))
//!     .with_rx(board_pin!(peripherals.LIDAR_RX));
//! ```

use embassy_time::Duration;

use crate::bno085::Vec3;

/// Takes the GPIO wired to a named signal out of esp-hal's `Peripherals`.
///
/// The arms below are the pin map; see the [`constants`](crate::constants)
/// module for the table and the reasoning.
#[macro_export]
macro_rules! board_pin {
    // Trigger button, wired to ground; uses the internal pull-up.
    ($p:ident . BUTTON) => {
        $p.GPIO4
    };

    // NEO-M9N GPS on UART2. Board TX to the GPS's RXD, board RX to its TXD.
    ($p:ident . GPS_TX) => {
        $p.GPIO15
    };
    ($p:ident . GPS_RX) => {
        $p.GPIO16
    };
    // Optional: hard reset (active low) and the one-pulse-per-second output.
    ($p:ident . GPS_RESET) => {
        $p.GPIO6
    };
    ($p:ident . GPS_TIMEPULSE) => {
        $p.GPIO7
    };

    // TF03 LiDAR on UART1, its dedicated pins. Board TX to the blue UART
    // RxD wire, board RX to the brown UART TxD wire.
    ($p:ident . LIDAR_TX) => {
        $p.GPIO17
    };
    ($p:ident . LIDAR_RX) => {
        $p.GPIO18
    };

    // BNO085 IMU on SPI2, its dedicated (IO MUX) pins. Comments name the
    // Adafruit breakout's printed labels.
    // Breakout CS.
    ($p:ident . IMU_CS) => {
        $p.GPIO10
    };
    // Breakout DI: data into the sensor.
    ($p:ident . IMU_MOSI) => {
        $p.GPIO11
    };
    // Breakout SCL, which is the SPI clock in SPI mode.
    ($p:ident . IMU_SCK) => {
        $p.GPIO12
    };
    // Breakout SDA, which is data out of the sensor in SPI mode.
    ($p:ident . IMU_MISO) => {
        $p.GPIO13
    };
    // Breakout INT: active-low data-ready line.
    ($p:ident . IMU_INT) => {
        $p.GPIO14
    };
    // Breakout RST: active-low reset.
    ($p:ident . IMU_RESET) => {
        $p.GPIO9
    };
    // Breakout P0 (PS0/WAKE): held high through reset to select SPI, then
    // pulsed low to wake the IMU before the board writes to it.
    ($p:ident . IMU_WAKE) => {
        $p.GPIO8
    };

    // I2C display (SSD1306 OLED), run from 3V3.
    ($p:ident . LCD_SDA) => {
        $p.GPIO1
    };
    ($p:ident . LCD_SCL) => {
        $p.GPIO2
    };
}

/// Takes the peripheral controller that drives a named bus out of esp-hal's
/// `Peripherals`.
#[macro_export]
macro_rules! board_bus {
    // UART0 stays with the USB-to-UART bridge.
    ($p:ident . LIDAR_UART) => {
        $p.UART1
    };
    ($p:ident . GPS_UART) => {
        $p.UART2
    };
    ($p:ident . IMU_SPI) => {
        $p.SPI2
    };
    ($p:ident . LCD_I2C) => {
        $p.I2C0
    };
}

// ---------------------------------------------------------------------------
// Bus settings
// ---------------------------------------------------------------------------

/// The TF03's factory UART baud rate (8N1).
pub const LIDAR_BAUD_RATE: u32 = 115_200;

/// The NEO-M9N's factory UART1 baud rate (8N1).
pub const GPS_BAUD_RATE: u32 = crate::neo_m9n::DEFAULT_BAUD_RATE;

/// SPI clock for the BNO085, in kilohertz. The datasheet's SPI timing gives
/// room above this; 1 MHz is a conservative starting point to raise once the
/// link is proven.
pub const IMU_SPI_FREQUENCY_KHZ: u32 = 1_000;

/// I2C clock for the display, in kilohertz: the SSD1306's fast mode maximum.
pub const LCD_I2C_FREQUENCY_KHZ: u32 = 400;

/// The display's I2C address: 0x3C on most SSD1306 modules, 0x3D with the
/// address resistor on the back moved.
pub const LCD_ADDRESS: u8 = 0x3C;

/// Where the glass starts in the display controller's memory: 0 for an
/// SSD1306 (most 0.96" modules), 2 for an SH1106 (most 1.3" modules). Text
/// shifted two pixels with garbage at one edge means this is wrong.
pub const LCD_COLUMN_OFFSET: u8 = 2;

// ---------------------------------------------------------------------------
// Sensor rates
// ---------------------------------------------------------------------------

/// How many positions a second the GPS computes.
pub const GPS_RATE_HZ: u8 = 10;

/// How often the IMU reports its orientation: every 10 ms, i.e. 100 Hz.
pub const IMU_REPORT_INTERVAL_US: u32 = 10_000;

// ---------------------------------------------------------------------------
// Aiming
// ---------------------------------------------------------------------------

/// The direction the LiDAR looks, in the IMU's frame: the axes printed on the
/// BNO085 breakout. Set this to match how the two are mounted. The default is
/// +Y because that is where the BNO085 itself points: lying level with +Y
/// towards magnetic north, it reports no rotation at all.
pub const LIDAR_BORESIGHT: Vec3 = Vec3 {
    x: 0.0,
    y: 1.0,
    z: 0.0,
};

/// How far magnetic north lies east of true north where the device is used,
/// in degrees, negative for west. The IMU measures bearings from magnetic
/// north, but coordinates are laid out from true north. Look the value up at
/// <https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml>.
pub const MAGNETIC_DECLINATION_DEG: f32 = 0.0;

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

// Readings older than these when the button is pressed are not used. The GPS
// allowance covers a few missed solutions; the IMU and LiDAR report 100 times
// a second, so a quarter of a second without a report means something is
// wrong.
pub const MAX_GPS_AGE: Duration = Duration::from_secs(2);
pub const MAX_IMU_AGE: Duration = Duration::from_millis(250);
pub const MAX_LIDAR_AGE: Duration = Duration::from_millis(250);

/// A GPS this long without a solution has probably lost power, and with it
/// its configuration, which lives in RAM. The GPS task then configures it
/// again.
pub const GPS_SILENCE: Duration = Duration::from_secs(3);

/// An IMU this long without a report has probably reset, which stops every
/// report. The IMU task then starts it again.
pub const IMU_SILENCE: Duration = Duration::from_secs(1);

/// How long the button has to stay down to count as pressed, or up to count
/// as released: longer than a switch's contact bounce, shorter than a tap.
pub const DEBOUNCE: Duration = Duration::from_millis(30);

/// How long a line may take to leave over the USB serial port. The port only
/// drains while a computer reads it, so with nothing at the other end a write
/// would otherwise wait forever.
pub const SERIAL_TIMEOUT: Duration = Duration::from_millis(100);

/// How often the demos rewrite the display. Each full rewrite holds the I2C
/// bus for about 25 ms, and a reading that changes faster than this is a blur
/// anyway.
pub const LCD_REFRESH: Duration = Duration::from_millis(200);

/// How long to wait before retrying a sensor that failed to come up.
pub const RETRY_DELAY: Duration = Duration::from_secs(1);

// ---------------------------------------------------------------------------
// Memory
// ---------------------------------------------------------------------------

/// Bytes of reclaimed RAM given to the heap allocator.
pub const HEAP_SIZE: usize = 73_744;
