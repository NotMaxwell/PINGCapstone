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
//! direction (see [`ping_capstone::geo`]). The result goes to the USB serial
//! port, one line per press, where any serial monitor on the computer at the
//! other end of the cable shows it. Each line is also echoed over RTT
//! alongside the diagnostics, as in the demo binaries.
//!
//! Before trusting the bearings, set [`LIDAR_BORESIGHT`] to how the LiDAR is
//! mounted against the IMU and [`MAGNETIC_DECLINATION_DEG`] to where the
//! device is used. Both live in [`ping_capstone::constants`], with the pin map
//! and every other fixed setting.

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
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use ping_capstone::bno085::{Accuracy, Bno085, Quaternion, SpiTransport};
use ping_capstone::constants::{
    DEBOUNCE, GPS_BAUD_RATE, HEAP_SIZE, IMU_SPI_FREQUENCY_KHZ, LIDAR_BAUD_RATE, LIDAR_BORESIGHT,
    MAGNETIC_DECLINATION_DEG, MAX_GPS_AGE, MAX_IMU_AGE, MAX_LIDAR_AGE, SERIAL_TIMEOUT,
};
use ping_capstone::geo::{Aim, Position};
use ping_capstone::neo_m9n::{FixType, NeoM9n, Solution};
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

    spawner.spawn(gps_task(NeoM9n::new(gps_uart)).expect("Failed to spawn the GPS task"));
    spawner.spawn(imu_task(Bno085::new(imu, Delay)).expect("Failed to spawn the IMU task"));
    spawner.spawn(lidar_task(Tf03::new(lidar_uart)).expect("Failed to spawn the LiDAR task"));

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
    let solution = GPS.get(MAX_GPS_AGE)?;
    let orientation = IMU.get(MAX_IMU_AGE)?;
    let measurement = LIDAR.get(MAX_LIDAR_AGE)?;

    let (latitude_deg, longitude_deg) = solution.position().ok_or(Problem::NoFix {
        fix_type: solution.fix_type,
        satellites: solution.satellites,
    })?;
    let distance_cm = measurement.distance_cm.ok_or(Problem::NoReturn {
        strength: measurement.strength,
    })?;

    let origin = Position {
        latitude_deg,
        longitude_deg,
        altitude_m: f64::from(solution.height_msl_m),
    };
    let aim = Aim::from_orientation(
        &orientation.quaternion,
        LIDAR_BORESIGHT,
        MAGNETIC_DECLINATION_DEG,
    );
    let range_m = f32::from(distance_cm) / 100.0;

    Ok(Sighting {
        target: origin.project(aim, range_m),
        origin,
        aim,
        range_m,
        origin_accuracy_m: solution.horizontal_accuracy_m,
        satellites: solution.satellites,
        heading_accuracy_deg: orientation.heading_accuracy_rad.to_degrees(),
        compass: orientation.calibration,
    })
}

/// A located target, and what it was located from.
struct Sighting {
    target: Position,
    origin: Position,
    aim: Aim,
    range_m: f32,
    /// The GPS's 1σ estimate of its horizontal error, metres.
    origin_accuracy_m: f32,
    satellites: u8,
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
             | from {:.7}, {:.7}, alt {:.1} m, +/-{:.1} m, {} satellites \
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
            self.origin_accuracy_m,
            self.satellites,
            self.heading_accuracy_deg,
            self.compass,
        )
    }
}

/// Why a press produced no coordinate.
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

/// What the IMU task keeps from each rotation vector report.
#[derive(Clone, Copy)]
struct Orientation {
    quaternion: Quaternion,
    heading_accuracy_rad: f32,
    calibration: Accuracy,
}

static GPS: Latest<Solution> = Latest::new("GPS");
static IMU: Latest<Orientation> = Latest::new("IMU");
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
    use embassy_time::{Delay, Timer, with_timeout};
    use esp_hal::Async;
    use esp_hal::gpio::{Input, Output};
    use esp_hal::spi;
    use esp_hal::spi::master::Spi;
    use esp_hal::uart::Uart;
    use ping_capstone::bno085::{self, Bno085, ProductId, SensorData, SensorId, SpiTransport};
    use ping_capstone::constants::{
        GPS_RATE_HZ, GPS_SILENCE, IMU_REPORT_INTERVAL_US, IMU_SILENCE, RETRY_DELAY,
    };
    use ping_capstone::neo_m9n::NeoM9n;
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
            // The configuration lives in the receiver's RAM, so this runs
            // again whenever the receiver goes quiet, in case a power glitch
            // has put it back to its factory NMEA output.
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
                        Ok(Some(event)) => {
                            if let SensorData::RotationVector {
                                quaternion,
                                accuracy_rad,
                            } = event.data
                            {
                                IMU.set(Orientation {
                                    quaternion,
                                    heading_accuracy_rad: accuracy_rad,
                                    calibration: event.accuracy,
                                });
                            }
                        }
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

    /// Resets the hub and starts its rotation vector.
    async fn start(imu: &mut Imu) -> Result<ProductId, ImuError> {
        let id = imu.init().await?;
        imu.enable_report(SensorId::RotationVector, IMU_REPORT_INTERVAL_US)
            .await?;
        Ok(id)
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
