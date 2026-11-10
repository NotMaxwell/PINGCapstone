//! PING: aim the device at something, press the button, and it reports that
//! thing's GPS coordinate.
//!
//! Each sensor runs in its own task and keeps its latest reading in a shared
//! slot:
//!
//! * the NEO-M9N GPS gives the device's own position,
//! * the BNO085 IMU gives its orientation, and from that the direction the
//!   LiDAR points: a compass bearing and an elevation angle,
//! * the TF03 LiDAR gives the distance to whatever it points at.
//!
//! When the button is pressed, the main task takes the latest reading from
//! each and walks the LiDAR's range out from the GPS position along that
//! direction (see [`ping_capstone::geo`]). Without a GPS fix it walks out
//! from [`FALLBACK_POSITION`] instead, if one is set, and says so on the
//! line. The result goes to the USB serial port, one line per press, where
//! any serial monitor on the computer at the other end of the cable shows
//! it. Each line is also echoed over RTT alongside the diagnostics, as in
//! the demo binaries.
//!
//! The display shows the live heading, compass calibration, LiDAR range and
//! GPS state, and after each press the target's coordinate for
//! [`TARGET_HOLD`].
//!
//! The bearings come from the compass in [`ping_capstone::mag_cal`], which
//! needs `imu_cal` to have measured `MAG_CALIBRATION`. They are measured
//! from true north using [`HEADING_CORRECTION_DEG`], which starts as the
//! magnetic declination in Huntsville. The `heading_cal` binary measures it
//! against bearings that are known, which also takes in mounting error, and
//! `docs/heading-calibration.md` is the procedure. [`LIDAR_BORESIGHT`] says
//! how the LiDAR is mounted against the IMU. Both live in
//! [`ping_capstone::constants`], with the pin map and every other fixed
//! setting.
//!
//! [`TARGET_HOLD`]: ping_capstone::constants::TARGET_HOLD

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use alloc::format;
use core::cell::Cell;
use core::fmt;

use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_io_async::Write;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use ping_capstone::bno085::{Accuracy, Bno085, Quaternion, SpiTransport};
use ping_capstone::constants::{
    DEBOUNCE, FALLBACK_POSITION, GPS_BAUD_RATE, HEADING_CORRECTION_DEG, HEAP_SIZE,
    IMU_SPI_FREQUENCY_KHZ, LCD_ADDRESS, LCD_COLUMN_OFFSET, LCD_I2C_FREQUENCY_KHZ, LIDAR_BAUD_RATE,
    LIDAR_BORESIGHT, MAX_GPS_AGE, MAX_IMU_AGE, MAX_LIDAR_AGE, SERIAL_TIMEOUT,
};
use ping_capstone::geo::{Aim, Position};
use ping_capstone::neo_m9n::{FixType, NeoM9n, Solution};
use ping_capstone::ssd1306::Ssd1306;
use ping_capstone::tf03::{Measurement, Tf03};
use ping_capstone::{board_bus, board_pin};
use rtt_target::rprintln;

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    rprintln!("{}", panic_info);
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_print!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: HEAP_SIZE);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    rprintln!("Embassy initialized!");

    // Unused for now; the web page that will serve coordinates needs it.
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    // Every pin and bus below comes from ping_capstone::constants, which
    // holds the pin map and the reasoning behind it.

    // GPS: board TX to the NEO-M9N's RXD, board RX to its TXD.
    let gps_uart = Uart::new(
        board_bus!(peripherals.GPS_UART),
        UartConfig::default().with_baudrate(GPS_BAUD_RATE),
    )
    .expect("Failed to configure the GPS UART")
    .with_tx(board_pin!(peripherals.GPS_TX))
    .with_rx(board_pin!(peripherals.GPS_RX))
    .into_async();

    // IMU: SPI mode 3. WAKE (PS0) starts high so the hub comes up in SPI mode
    // when the driver releases its reset line; chip select starts high, idle.
    // INT is open drain and active low, so it needs a pull-up.
    let imu_spi = Spi::new(
        board_bus!(peripherals.IMU_SPI),
        SpiConfig::default()
            .with_frequency(Rate::from_khz(IMU_SPI_FREQUENCY_KHZ))
            .with_mode(SpiMode::_3),
    )
    .expect("Failed to configure the IMU SPI bus")
    .with_sck(board_pin!(peripherals.IMU_SCK))
    .with_mosi(board_pin!(peripherals.IMU_MOSI))
    .with_miso(board_pin!(peripherals.IMU_MISO))
    .into_async();
    let imu = SpiTransport::new(
        imu_spi,
        Output::new(
            board_pin!(peripherals.IMU_CS),
            Level::High,
            OutputConfig::default(),
        ),
        Input::new(
            board_pin!(peripherals.IMU_INT),
            InputConfig::default().with_pull(Pull::Up),
        ),
        Output::new(
            board_pin!(peripherals.IMU_WAKE),
            Level::High,
            OutputConfig::default(),
        ),
        Output::new(
            board_pin!(peripherals.IMU_RESET),
            Level::High,
            OutputConfig::default(),
        ),
    );

    // LiDAR: board TX to the TF03's blue UART RxD wire, board RX to its
    // brown UART TxD wire.
    let lidar_uart = Uart::new(
        board_bus!(peripherals.LIDAR_UART),
        UartConfig::default().with_baudrate(LIDAR_BAUD_RATE),
    )
    .expect("Failed to configure the LiDAR UART")
    .with_tx(board_pin!(peripherals.LIDAR_TX))
    .with_rx(board_pin!(peripherals.LIDAR_RX))
    .into_async();

    // Trigger button, wired from its GPIO to ground. GPIO0 (BOOT) is avoided
    // because holding it during a reset starts the ROM bootloader.
    let mut button = Input::new(
        board_pin!(peripherals.BUTTON),
        InputConfig::default().with_pull(Pull::Up),
    );

    // The USB port the board is flashed through. The computer sees it as a
    // serial port, /dev/cu.usbmodem* on macOS or a COM port on Windows.
    let mut serial = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();

    // Display: the OLED on I2C0. The display task brings it up, and keeps
    // trying if it is missing, so a loose display never holds up the rest.
    let lcd_i2c = I2c::new(
        board_bus!(peripherals.LCD_I2C),
        I2cConfig::default().with_frequency(Rate::from_khz(LCD_I2C_FREQUENCY_KHZ)),
    )
    .expect("Failed to configure the display I2C bus")
    .with_sda(board_pin!(peripherals.LCD_SDA))
    .with_scl(board_pin!(peripherals.LCD_SCL))
    .into_async();
    let lcd = Ssd1306::new(lcd_i2c, LCD_ADDRESS, LCD_COLUMN_OFFSET);

    spawner.spawn(gps_task(NeoM9n::new(gps_uart)).expect("Failed to spawn the GPS task"));
    spawner.spawn(imu_task(Bno085::new(imu, Delay)).expect("Failed to spawn the IMU task"));
    spawner.spawn(lidar_task(Tf03::new(lidar_uart)).expect("Failed to spawn the LiDAR task"));
    spawner.spawn(display_task(lcd).expect("Failed to spawn the display task"));

    let mut presses: u32 = 0;
    loop {
        button.wait_for_low().await;
        // Take the readings from when the button went down, before pressing
        // it has had time to jog where the device points.
        let sighting = take_sighting();
        Timer::after(DEBOUNCE).await;
        if button.is_high() {
            // Too short to be a press: noise, or contacts bouncing.
            continue;
        }

        presses = presses.wrapping_add(1);
        TARGET.set((presses, sighting));
        let line = match sighting {
            Ok(sighting) => format!("target #{presses}: {sighting}"),
            Err(problem) => format!("target #{presses}: none, {problem}"),
        };
        rprintln!("{}", line);
        send_line(&mut serial, &line).await;

        // One coordinate per press, however long the button is held.
        wait_for_release(&mut button).await;
    }
}

/// Locates what the LiDAR is pointing at, from the latest sensor readings.
#[allow(
    clippy::large_stack_frames,
    reason = "clippy counts each copy the `?`s make of the 96 byte GPS solution separately; \
    compiled, they share a frame of a few hundred bytes"
)]
fn take_sighting() -> Result<Sighting, Problem> {
    let orientation = IMU.get(MAX_IMU_AGE)?;
    let measurement = LIDAR.get(MAX_LIDAR_AGE)?;
    let distance_cm = measurement.distance_cm.ok_or(Problem::NoReturn {
        strength: measurement.strength,
    })?;

    let (origin, source) = match (gps_position(), FALLBACK_POSITION) {
        (Ok((origin, source)), _) => (origin, source),
        (Err(problem), Some(fallback)) => (fallback, Source::Fallback(problem)),
        (Err(problem), None) => return Err(problem),
    };
    let aim = orientation.aim();
    let range_m = f32::from(distance_cm) / 100.0;

    Ok(Sighting {
        target: origin.project(aim, range_m),
        origin,
        aim,
        range_m,
        source,
        heading_accuracy_deg: orientation.heading_accuracy_rad.to_degrees(),
        compass: orientation.calibration,
    })
}

/// The device's own position from a recent GPS fix.
fn gps_position() -> Result<(Position, Source), Problem> {
    let solution = GPS.get(MAX_GPS_AGE)?;
    let (latitude_deg, longitude_deg) = solution.position().ok_or(Problem::NoFix {
        fix_type: solution.fix_type,
        satellites: solution.satellites,
    })?;
    let origin = Position {
        latitude_deg,
        longitude_deg,
        altitude_m: f64::from(solution.height_msl_m),
    };
    let source = Source::Gps {
        accuracy_m: solution.horizontal_accuracy_m,
        satellites: solution.satellites,
    };
    Ok((origin, source))
}

/// Where a sighting's origin came from.
#[derive(Clone, Copy)]
enum Source {
    Gps {
        /// The GPS's 1σ estimate of its horizontal error, metres.
        accuracy_m: f32,
        satellites: u8,
    },
    /// [`FALLBACK_POSITION`], because the GPS could not give a position.
    Fallback(Problem),
}

impl fmt::Display for Source {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Gps {
                accuracy_m,
                satellites,
            } => write!(f, "+/-{accuracy_m:.1} m, {satellites} satellites"),
            Self::Fallback(problem) => write!(f, "FALLBACK POSITION, {problem}"),
        }
    }
}

/// A located target, and what it was located from.
#[derive(Clone, Copy)]
struct Sighting {
    target: Position,
    origin: Position,
    aim: Aim,
    range_m: f32,
    source: Source,
    /// The IMU's estimate of its heading error, degrees.
    heading_accuracy_deg: f32,
    /// How far the IMU trusts its magnetometer calibration.
    compass: Accuracy,
}

impl fmt::Display for Sighting {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:.7}, {:.7}, alt {:.1} m \
             | range {:.2} m, bearing {:.1} deg, elevation {:.1} deg \
             | from {:.7}, {:.7}, alt {:.1} m, {} \
             | heading +/-{:.1} deg, compass calibration {:?}",
            self.target.latitude_deg,
            self.target.longitude_deg,
            self.target.altitude_m,
            self.range_m,
            self.aim.azimuth_deg,
            self.aim.elevation_deg,
            self.origin.latitude_deg,
            self.origin.longitude_deg,
            self.origin.altitude_m,
            self.source,
            self.heading_accuracy_deg,
            self.compass,
        )
    }
}

/// Why a press produced no coordinate, or why the GPS position was not used.
#[derive(Clone, Copy)]
enum Problem {
    /// A sensor has not reported recently, or at all.
    Stale {
        sensor: &'static str,
        age: Option<Duration>,
    },
    /// The GPS is running but has no position it vouches for: no fix yet, or
    /// one outside its accuracy limits.
    NoFix { fix_type: FixType, satellites: u8 },
    /// The LiDAR saw nothing in range, or nothing it trusts.
    NoReturn { strength: u16 },
}

impl fmt::Display for Problem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Stale { sensor, age: None } => write!(f, "no {sensor} reading yet"),
            Self::Stale {
                sensor,
                age: Some(age),
            } => write!(f, "last {sensor} reading is {} ms old", age.as_millis()),
            Self::NoFix {
                fix_type,
                satellites,
            } => write!(
                f,
                "GPS has no usable fix ({fix_type:?}, {satellites} satellites)"
            ),
            Self::NoReturn { strength } => {
                write!(f, "LiDAR has no return it trusts (strength {strength})")
            }
        }
    }
}

/// The latest reading from one sensor, and when it arrived. The sensor's task
/// overwrites it; a button press copies it out.
struct Latest<T> {
    sensor: &'static str,
    reading: Mutex<Cell<Option<(Instant, T)>>>,
}

impl<T: Copy> Latest<T> {
    const fn new(sensor: &'static str) -> Self {
        Self {
            sensor,
            reading: Mutex::new(Cell::new(None)),
        }
    }

    fn set(&self, value: T) {
        let reading = Some((Instant::now(), value));
        critical_section::with(|cs| self.reading.borrow(cs).set(reading));
    }

    /// The reading, as long as it is no older than `max_age`.
    fn get(&self, max_age: Duration) -> Result<T, Problem> {
        match critical_section::with(|cs| self.reading.borrow(cs).get()) {
            Some((at, value)) if at.elapsed() <= max_age => Ok(value),
            reading => Err(Problem::Stale {
                sensor: self.sensor,
                age: reading.map(|(at, _)| at.elapsed()),
            }),
        }
    }
}

/// What the IMU task keeps from each compass update.
#[derive(Clone, Copy)]
struct Orientation {
    quaternion: Quaternion,
    heading_accuracy_rad: f32,
    calibration: Accuracy,
}

impl Orientation {
    /// Where the LiDAR points, as a true bearing and an elevation.
    fn aim(&self) -> Aim {
        Aim::from_orientation(&self.quaternion, LIDAR_BORESIGHT, HEADING_CORRECTION_DEG)
    }
}

static GPS: Latest<Solution> = Latest::new("GPS");
static IMU: Latest<Orientation> = Latest::new("IMU");
/// The latest press: its number, and what it found.
static TARGET: Latest<(u32, Result<Sighting, Problem>)> = Latest::new("target");
static LIDAR: Latest<Measurement> = Latest::new("LiDAR");

/// Sends one line to the computer over the USB serial port, if something
/// there is reading it.
async fn send_line(serial: &mut UsbSerialJtag<'static, Async>, line: &str) {
    let sent = with_timeout(SERIAL_TIMEOUT, async {
        serial.write_all(line.as_bytes()).await?;
        serial.write_all(b"\r\n").await?;
        serial.flush().await
    })
    .await;
    if sent.is_err() {
        rprintln!("Nothing is reading the USB serial port; line not sent");
    }
}

/// Waits for the button to be let go and to stay up past any contact bounce.
async fn wait_for_release(button: &mut Input<'_>) {
    loop {
        button.wait_for_high().await;
        Timer::after(DEBOUNCE).await;
        if button.is_high() {
            return;
        }
    }
}

use display::display_task;
use sensors::{gps_task, imu_task, lidar_task};

mod sensors {
    //! One task per sensor, each keeping its latest reading where a button
    //! press can pick it up. They live in their own module so that the stack
    //! frame lint can be relaxed for them. An embassy task's future is
    //! allocated once in a static task pool rather than on the stack, and
    //! most of its size here is the drivers' buffers, the HAL's futures and
    //! the formatting machinery behind rprintln!, not anything this code puts
    //! on the stack.
    #![allow(clippy::large_stack_frames)]

    use super::{GPS, IMU, LIDAR, Orientation};
    use core::convert::Infallible;
    use embassy_time::{Delay, Instant, Timer, with_timeout};
    use esp_hal::Async;
    use esp_hal::gpio::{Input, Output};
    use esp_hal::spi;
    use esp_hal::spi::master::Spi;
    use esp_hal::uart::Uart;
    use ping_capstone::bno085::{
        self, Accuracy, Bno085, ProductId, SensorData, SensorId, SpiTransport,
    };
    use ping_capstone::constants::{
        COMPASS_LOG_INTERVAL, GPS_RATE_HZ, GPS_SILENCE, IMU_REPORT_INTERVAL_US, IMU_SILENCE,
        MAG_CALIBRATION, MAG_REPORT_INTERVAL_US, RETRY_DELAY,
    };
    use ping_capstone::mag_cal::{Compass, StartupCalibration};
    use ping_capstone::neo_m9n::{NeoM9n, Solution};
    use ping_capstone::tf03::Tf03;
    use rtt_target::rprintln;

    type ImuTransport = SpiTransport<
        Spi<'static, Async>,
        Output<'static>,
        Input<'static>,
        Output<'static>,
        Output<'static>,
    >;
    type Imu = Bno085<ImuTransport, Delay>;
    type ImuError = bno085::Error<bno085::SpiError<spi::Error, Infallible>>;

    /// Keeps the latest navigation solution from the NEO-M9N.
    #[embassy_executor::task]
    pub async fn gps_task(mut gps: NeoM9n<Uart<'static, Async>>) {
        loop {
            // The configuration survives a restart only while the
            // receiver's backup supply holds, so this runs again whenever
            // the receiver goes quiet, in case it is back to its factory
            // NMEA output.
            while let Err(error) = gps.configure(GPS_RATE_HZ).await {
                rprintln!("GPS configuration failed: {:?}, retrying", error);
                Timer::after(RETRY_DELAY).await;
            }
            rprintln!("GPS up: {} solutions a second", GPS_RATE_HZ);

            loop {
                match with_timeout(GPS_SILENCE, gps.read()).await {
                    Ok(Ok(solution)) => GPS.set(solution),
                    // Typically an RX FIFO overflow; the parser
                    // resynchronises on the next frame, so carry on reading.
                    Ok(Err(error)) => rprintln!("GPS read failed: {:?}", error),
                    Err(_) => {
                        rprintln!("GPS went quiet, configuring it again");
                        break;
                    }
                }
            }
        }
    }

    /// Logs the GPS's fix. The satellite count is those used in the fix, so
    /// it reads 0 until there is one, however many are being tracked;
    /// `gps_diag` shows those.
    fn log_gps(solution: &Solution) {
        match solution.position() {
            Some((latitude, longitude)) => rprintln!(
                "GPS: {:?}, {} satellites, {:.7}, {:.7} +/-{:.1} m",
                solution.fix_type,
                solution.satellites,
                latitude,
                longitude,
                solution.horizontal_accuracy_m
            ),
            None => rprintln!("GPS: no fix yet ({:?})", solution.fix_type),
        }
    }

    /// Keeps the latest orientation from the BNO085.
    ///
    /// The hub pulls INT low when it has reports waiting, and holds it low
    /// until everything waiting has been read, so each wake-up drains the hub
    /// before going back to waiting.
    #[embassy_executor::task]
    pub async fn imu_task(mut imu: Imu) {
        loop {
            match start(&mut imu).await {
                Ok(id) => rprintln!(
                    "BNO085 up: firmware {}.{}.{}, reset cause {:?}",
                    id.sw_version_major,
                    id.sw_version_minor,
                    id.sw_version_patch,
                    id.reset_cause
                ),
                Err(error) => {
                    rprintln!("BNO085 bring-up failed: {:?}, retrying", error);
                    Timer::after(RETRY_DELAY).await;
                    continue;
                }
            }

            // The game rotation vector's frame starts over with the hub.
            let mut compass = Compass::new(MAG_CALIBRATION);
            let mut startup = StartupCalibration::new(MAG_CALIBRATION.map(|c| c.matrix));
            let mut next_log = Instant::now();
            'reporting: loop {
                match with_timeout(IMU_SILENCE, imu.wait_for_data()).await {
                    Ok(Ok(())) => {}
                    Ok(Err(error)) => {
                        rprintln!("BNO085 wait failed: {:?}, starting it again", error);
                        break;
                    }
                    Err(_) => {
                        rprintln!("BNO085 went quiet, starting it again");
                        break;
                    }
                }

                loop {
                    match imu.read_event().await {
                        Ok(Some(event)) => match event.data {
                            SensorData::MagneticFieldUncalibrated { field, .. } => {
                                compass.update_field(field);
                                if let Some(found) = startup.add(field) {
                                    rprintln!(
                                        "Compass calibrated from handling: hard iron {:.1} {:.1} {:.1} uT, field {:.1} uT, residual {:.4}",
                                        found.offset[0],
                                        found.offset[1],
                                        found.offset[2],
                                        found.field_ut,
                                        found.residual
                                    );
                                    compass.set_calibration(found);
                                }
                            }
                            SensorData::GameRotationVector(quaternion) => {
                                compass.update_game(quaternion);
                                let Some(heading) = compass.heading() else {
                                    continue;
                                };
                                let orientation = Orientation {
                                    quaternion: heading.quaternion,
                                    heading_accuracy_rad: heading.accuracy_rad,
                                    calibration: heading.status,
                                };
                                IMU.set(orientation);

                                if Instant::now() >= next_log {
                                    next_log = Instant::now() + COMPASS_LOG_INTERVAL;
                                    log_compass(&orientation);
                                }
                            }
                            _ => {}
                        },
                        // Nothing left waiting; go back to the interrupt pin.
                        Ok(None) => break,
                        // A hub that resets on its own announces it with a
                        // packet too large for the driver, and a failing bus
                        // is worth a fresh start too. Either way the reports
                        // have stopped, so start the hub again.
                        Err(error) => {
                            rprintln!("BNO085 read failed: {:?}, starting it again", error);
                            break 'reporting;
                        }
                    }
                }
            }
        }
    }

    /// Resets the hub and starts the reports the compass is built from.
    async fn start(imu: &mut Imu) -> Result<ProductId, ImuError> {
        let id = imu.init().await?;
        // `imu_cal` calibrates the magnetometer; the hub keeps calibrating
        // only what the game rotation vector relies on.
        imu.configure_calibration(true, false, false).await?;
        imu.enable_report(SensorId::GameRotationVector, IMU_REPORT_INTERVAL_US)
            .await?;
        imu.enable_report(SensorId::MagneticFieldUncalibrated, MAG_REPORT_INTERVAL_US)
            .await?;
        Ok(id)
    }

    /// Logs where the LiDAR points by the compass, and how far the IMU
    /// trusts it.
    fn log_compass(orientation: &Orientation) {
        let aim = orientation.aim();
        rprintln!(
            "Compass: bearing {:.1} deg, elevation {:.1} deg, heading +/-{:.1} deg, calibration {:?}{}",
            aim.azimuth_deg,
            aim.elevation_deg,
            orientation.heading_accuracy_rad.to_degrees(),
            orientation.calibration,
            if orientation.calibration == Accuracy::High {
                ""
            } else {
                " (keep turning and tilting the device to calibrate; if it persists, the field here is disturbed)"
            }
        );
    }

    /// Keeps the latest distance from the TF03.
    #[embassy_executor::task]
    pub async fn lidar_task(mut lidar: Tf03<Uart<'static, Async>>) {
        // The TF03 streams measurements unasked, so distances need only its
        // TxD wire. Answering a command also needs its RxD wire, so the
        // firmware version is asked for once, for the log, and its answer
        // waited for no further.
        match lidar.firmware_version().await {
            Ok(version) => rprintln!(
                "TF03 firmware {}.{}.{}",
                version.major,
                version.minor,
                version.patch
            ),
            Err(error) => rprintln!(
                "TF03 did not answer the firmware version request: {:?}. Reading distances anyway.",
                error
            ),
        }

        loop {
            match lidar.read().await {
                Ok(measurement) => LIDAR.set(measurement),
                // Typically an RX FIFO overflow; the driver resynchronises
                // on the next frame, so carry on reading.
                Err(error) => rprintln!("TF03 read failed: {:?}", error),
            }
        }
    }
}

mod display {
    //! The display task, in its own module so the stack frame lint can be
    //! relaxed for it, as for the sensor tasks: its future lives in a static
    //! task pool, and is mostly the I2C driver's.
    #![allow(clippy::large_stack_frames)]

    use super::{GPS, IMU, LIDAR, Problem, Source, TARGET};
    use alloc::format;
    use alloc::string::String;
    use embassy_time::{Duration, Timer};
    use esp_hal::Async;
    use esp_hal::i2c::master::I2c;
    use ping_capstone::bno085::Accuracy;
    use ping_capstone::constants::{
        LCD_REFRESH, MAX_GPS_AGE, MAX_IMU_AGE, MAX_LIDAR_AGE, TARGET_HOLD,
    };
    use ping_capstone::ssd1306::{ROWS, Ssd1306};
    use rtt_target::rprintln;

    type Display = Ssd1306<I2c<'static, Async>>;

    /// The nearest of the sixteen compass points to `bearing_deg`.
    fn compass_point(bearing_deg: f32) -> &'static str {
        const POINTS: [&str; 16] = [
            "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW",
            "NW", "NNW",
        ];
        // Each point covers 22.5°, centred on its own bearing.
        POINTS[((bearing_deg / 22.5 + 0.5) as usize) % POINTS.len()]
    }

    /// Why there is no reading or no target, in two lines short enough for
    /// the display.
    fn explain(problem: &Problem) -> [String; 2] {
        match *problem {
            Problem::Stale { sensor, age: None } => {
                [format!("no {sensor}"), String::from("reading yet")]
            }
            Problem::Stale {
                sensor,
                age: Some(age),
            } => [
                format!("{sensor} silent"),
                format!("for {:.1} s", age.as_millis() as f32 / 1000.0),
            ],
            Problem::NoFix { satellites, .. } => [
                String::from("GPS no fix"),
                format!("{satellites} satellites"),
            ],
            Problem::NoReturn { strength } => [
                String::from("LiDAR: no return"),
                format!("strength {strength}"),
            ],
        }
    }

    /// What to show now: the latest press for [`TARGET_HOLD`] after it,
    /// otherwise the live readings.
    fn screen() -> [String; ROWS] {
        if let Ok((press, result)) = TARGET.get(TARGET_HOLD) {
            return match result {
                Ok(sighting) => {
                    let target = sighting.target;
                    [
                        match sighting.source {
                            Source::Gps { .. } => format!("#{press} target"),
                            Source::Fallback(_) => format!("#{press} test origin"),
                        },
                        format!(
                            "{} {:.6}",
                            if target.latitude_deg < 0.0 { 'S' } else { 'N' },
                            target.latitude_deg.abs()
                        ),
                        format!(
                            "{} {:.6}",
                            if target.longitude_deg < 0.0 { 'W' } else { 'E' },
                            target.longitude_deg.abs()
                        ),
                        format!(
                            "{:.1}m @ {:.0}°",
                            sighting.range_m, sighting.aim.azimuth_deg
                        ),
                    ]
                }
                Err(problem) => {
                    let [why, detail] = explain(&problem);
                    [format!("#{press} no target"), why, detail, String::new()]
                }
            };
        }

        let (heading, calibration) = match IMU.get(MAX_IMU_AGE) {
            Ok(orientation) => {
                let aim = orientation.aim();
                (
                    format!(
                        "hdg {:.1}° {}",
                        aim.azimuth_deg,
                        compass_point(aim.azimuth_deg)
                    ),
                    match orientation.calibration {
                        Accuracy::High => format!(
                            "cal high ±{:.0}°",
                            orientation.heading_accuracy_rad.to_degrees()
                        ),
                        Accuracy::Medium => String::from("cal med"),
                        Accuracy::Low => String::from("cal low"),
                        Accuracy::Unreliable => String::from("cal bad - move"),
                    },
                )
            }
            Err(problem) => explain(&problem).into(),
        };
        let range = match LIDAR.get(MAX_LIDAR_AGE) {
            Ok(measurement) => match measurement.distance_cm {
                Some(cm) => format!("rng {:.2} m", f32::from(cm) / 100.0),
                None => String::from("rng: no return"),
            },
            Err(_) => String::from("LiDAR silent"),
        };
        let gps = match GPS.get(MAX_GPS_AGE) {
            Ok(solution) => format!("GPS {:?} {}sat", solution.fix_type, solution.satellites),
            Err(_) => String::from("GPS silent"),
        };
        [heading, calibration, range, gps]
    }

    /// Keeps the display up to date, [`LCD_REFRESH`] at a time.
    #[embassy_executor::task]
    pub async fn display_task(mut lcd: Display) {
        // Whether the display answered last time, so a missing one is
        // reported once rather than five times a second.
        let mut working = false;
        let mut reported = false;
        loop {
            if !working {
                match lcd.init().await {
                    Ok(()) => {
                        rprintln!("Display up");
                        working = true;
                        reported = false;
                    }
                    Err(error) if !reported => {
                        rprintln!(
                            "Display not answering at {:#04x}: {:?}; retrying quietly",
                            super::LCD_ADDRESS,
                            error
                        );
                        reported = true;
                    }
                    Err(_) => {}
                }
            }

            if working {
                for (row, line) in screen().iter().enumerate() {
                    if let Err(error) = lcd.write_line(row, line).await {
                        rprintln!("Display write failed: {:?}; retrying quietly", error);
                        working = false;
                        reported = true;
                        break;
                    }
                }
            }

            // A missing display is retried less often.
            let wait = if working {
                LCD_REFRESH
            } else {
                Duration::from_secs(1)
            };
            Timer::after(wait).await;
        }
    }
}
