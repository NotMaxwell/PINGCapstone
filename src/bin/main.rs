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
//! device is used.

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
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use ping_capstone::bno085::{Accuracy, Address, Bno085, Quaternion, Vec3};
use ping_capstone::geo::{Aim, Position};
use ping_capstone::neo_m9n::{self, FixType, NeoM9n, Solution};
use ping_capstone::tf03::{Measurement, Tf03};
use rtt_target::rprintln;

/// How many positions a second the GPS computes.
const GPS_RATE_HZ: u8 = 10;

/// How often the IMU reports its orientation: every 10 ms, i.e. 100 Hz.
const IMU_REPORT_INTERVAL_US: u32 = 10_000;

/// The direction the LiDAR looks, in the IMU's frame: the axes printed on the
/// BNO085 breakout. Set this to match how the two are mounted. The default is
/// +Y because that is where the BNO085 itself points: lying level with +Y
/// towards magnetic north, it reports no rotation at all.
const LIDAR_BORESIGHT: Vec3 = Vec3 {
    x: 0.0,
    y: 1.0,
    z: 0.0,
};

/// How far magnetic north lies east of true north where the device is used,
/// in degrees, negative for west. The IMU measures bearings from magnetic
/// north, but coordinates are laid out from true north. Look the value up at
/// <https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml>.
const MAGNETIC_DECLINATION_DEG: f32 = 0.0;

// Readings older than these when the button is pressed are not used. The GPS
// allowance covers a few missed solutions; the IMU and LiDAR report 100 times
// a second, so a quarter of a second without a report means something is
// wrong.
const MAX_GPS_AGE: Duration = Duration::from_secs(2);
const MAX_IMU_AGE: Duration = Duration::from_millis(250);
const MAX_LIDAR_AGE: Duration = Duration::from_millis(250);

/// A GPS this long without a solution has probably lost power, and with it
/// its configuration, which lives in RAM. The GPS task then configures it
/// again.
const GPS_SILENCE: Duration = Duration::from_secs(3);

/// An IMU this long without a report has probably reset, which stops every
/// report. The IMU task then starts it again.
const IMU_SILENCE: Duration = Duration::from_secs(1);

/// How long the button has to stay down to count as pressed, or up to count
/// as released: longer than a switch's contact bounce, shorter than a tap.
const DEBOUNCE: Duration = Duration::from_millis(30);

/// How long a line may take to leave over the USB serial port. The port only
/// drains while a computer reads it, so with nothing at the other end a write
/// would otherwise wait forever.
const SERIAL_TIMEOUT: Duration = Duration::from_millis(100);

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

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    rprintln!("Embassy initialized!");

    // Unused for now; the web page that will serve coordinates needs it.
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    // The sensors are wired as in their demo binaries, which say more.
    //
    // GPS: GPIO15 (TX) to the NEO-M9N board's RX1, GPIO16 (RX) to its TX1.
    let gps_uart = Uart::new(
        peripherals.UART2,
        UartConfig::default().with_baudrate(neo_m9n::DEFAULT_BAUD_RATE),
    )
    .expect("Failed to configure the GPS UART")
    .with_tx(peripherals.GPIO15)
    .with_rx(peripherals.GPIO16)
    .into_async();

    // IMU: SDA on GPIO8, SCL on GPIO9, and the BNO085's INT line on GPIO10.
    // INT is open drain and active low, so it needs a pull-up.
    let imu_i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .expect("Failed to configure the IMU I2C bus")
    .with_sda(peripherals.GPIO8)
    .with_scl(peripherals.GPIO9)
    .into_async();
    let imu_int = Input::new(
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::Up),
    );

    // LiDAR: GPIO17 (TX) to the TF03's blue RxD wire, GPIO18 (RX) to its
    // brown TxD wire.
    let lidar_uart = Uart::new(
        peripherals.UART1,
        UartConfig::default().with_baudrate(115_200),
    )
    .expect("Failed to configure the LiDAR UART")
    .with_tx(peripherals.GPIO17)
    .with_rx(peripherals.GPIO18)
    .into_async();

    // GPIO0 is the BOOT button on most ESP32-S3 boards, so this works with
    // nothing wired up. For a button of your own, wire it from a free GPIO to
    // ground and change the pin here. Holding GPIO0 down while the board
    // resets starts the ROM bootloader instead of this program.
    let mut button = Input::new(
        peripherals.GPIO0,
        InputConfig::default().with_pull(Pull::Up),
    );

    // The USB port the board is flashed through. The computer sees it as a
    // serial port, /dev/cu.usbmodem* on macOS or a COM port on Windows.
    let mut serial = UsbSerialJtag::new(peripherals.USB_DEVICE).into_async();

    spawner.spawn(gps_task(NeoM9n::new(gps_uart)).expect("Failed to spawn the GPS task"));
    spawner.spawn(
        imu_task(Bno085::new(imu_i2c, Address::Default, Delay), imu_int)
            .expect("Failed to spawn the IMU task"),
    );
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

    use super::{
        GPS, GPS_RATE_HZ, GPS_SILENCE, IMU, IMU_REPORT_INTERVAL_US, IMU_SILENCE, LIDAR, Orientation,
    };
    use embassy_time::{Delay, Duration, Timer, with_timeout};
    use esp_hal::Async;
    use esp_hal::gpio::Input;
    use esp_hal::i2c;
    use esp_hal::i2c::master::I2c;
    use esp_hal::uart::Uart;
    use ping_capstone::bno085::{self, Bno085, ProductId, SensorData, SensorId};
    use ping_capstone::neo_m9n::NeoM9n;
    use ping_capstone::tf03::Tf03;
    use rtt_target::rprintln;

    type Imu = Bno085<I2c<'static, Async>, Delay>;

    /// Keeps the latest navigation solution from the NEO-M9N.
    #[embassy_executor::task]
    pub async fn gps_task(mut gps: NeoM9n<Uart<'static, Async>>) {
        loop {
            // The configuration lives in the receiver's RAM, so this runs
            // again whenever the receiver goes quiet, in case a power glitch
            // has put it back to its factory NMEA output.
            while let Err(error) = gps.configure(GPS_RATE_HZ).await {
                rprintln!("GPS configuration failed: {:?}, retrying", error);
                Timer::after(Duration::from_secs(1)).await;
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
    pub async fn imu_task(mut imu: Imu, mut int: Input<'static>) {
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
                    Timer::after(Duration::from_secs(1)).await;
                    continue;
                }
            }

            'reporting: loop {
                if with_timeout(IMU_SILENCE, int.wait_for_low()).await.is_err() {
                    rprintln!("BNO085 went quiet, starting it again");
                    break;
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
    async fn start(imu: &mut Imu) -> Result<ProductId, bno085::Error<i2c::master::Error>> {
        let id = imu.init().await?;
        imu.enable_report(SensorId::RotationVector, IMU_REPORT_INTERVAL_US)
            .await?;
        Ok(id)
    }

    /// Keeps the latest distance from the TF03.
    #[embassy_executor::task]
    pub async fn lidar_task(mut lidar: Tf03<Uart<'static, Async>>) {
        loop {
            match lidar.firmware_version().await {
                Ok(version) => {
                    rprintln!(
                        "TF03 up: firmware {}.{}.{}",
                        version.major,
                        version.minor,
                        version.patch
                    );
                    break;
                }
                Err(error) => {
                    rprintln!("TF03 not answering: {:?}, retrying", error);
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
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
