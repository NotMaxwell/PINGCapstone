//! Diagnostic binary for the BNO085's compass: finds out why the heading is
//! wrong.
//!
//! A heading is only as good as the magnetic field it is taken from. Twice a
//! second this prints, over RTT:
//!
//! * the corrected field's strength and dip, against Earth's field where the
//!   device is ([`EXPECTED_FIELD_UT`], [`EXPECTED_DIP_DEG`]). A field that is
//!   too strong or too weak, or dips at the wrong angle, means something near
//!   the IMU is bending it, or [`MAG_CALIBRATION`] no longer fits;
//! * the raw field's strength and the hard iron offset in use, which is how
//!   strongly magnetised the things that move with the IMU are;
//! * the hub's own hard iron estimate, which should stay near zero because
//!   its magnetometer calibration is off;
//! * the heading of the breakout's +Y axis three ways: the compass the
//!   application uses, one worked out here from the corrected field and
//!   gravity, and the game rotation vector (gyroscope only, so relative to
//!   where it started). The first two should agree.
//!
//! All headings are magnetic: no declination or trim is applied.
//!
//! [`EXPECTED_FIELD_UT`]: ping_capstone::constants::EXPECTED_FIELD_UT
//! [`EXPECTED_DIP_DEG`]: ping_capstone::constants::EXPECTED_DIP_DEG
//! [`MAG_CALIBRATION`]: ping_capstone::constants::MAG_CALIBRATION

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use ping_capstone::bno085::{Bno085, SpiTransport};
use ping_capstone::constants::{HEAP_SIZE, IMU_SPI_FREQUENCY_KHZ};
use ping_capstone::{board_bus, board_pin};
use rtt_target::rprintln;

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    rprintln!("{}", panic_info);
    loop {}
}

extern crate alloc;

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

    // Wi-Fi is deliberately not started, to keep its current draw and RF
    // away from the magnetometer.
    let spi = Spi::new(
        board_bus!(peripherals.IMU_SPI),
        SpiConfig::default()
            .with_frequency(Rate::from_khz(IMU_SPI_FREQUENCY_KHZ))
            .with_mode(SpiMode::_3),
    )
    .expect("Failed to configure SPI")
    .with_sck(board_pin!(peripherals.IMU_SCK))
    .with_mosi(board_pin!(peripherals.IMU_MOSI))
    .with_miso(board_pin!(peripherals.IMU_MISO))
    .into_async();
    let transport = SpiTransport::new(
        spi,
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

    spawner.spawn(
        diag::compass_task(Bno085::new(transport, Delay)).expect("Failed to spawn the IMU task"),
    );

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

mod diag {
    //! The diagnostic task, in its own module so the stack frame lint can be
    //! relaxed: the driver's buffers live in the task's static future.
    #![allow(clippy::large_stack_frames)]

    use embassy_time::{Delay, Duration, Instant, Timer};
    use esp_hal::Async;
    use esp_hal::gpio::{Input, Output};
    use esp_hal::spi::master::Spi;
    use libm::{asinf, atan2f, sqrtf};
    use ping_capstone::bno085::{Bno085, Quaternion, SensorData, SensorId, SpiTransport, Vec3};
    use ping_capstone::constants::{
        EXPECTED_DIP_DEG, EXPECTED_FIELD_UT, MAG_CALIBRATION, MAG_REPORT_INTERVAL_US, RETRY_DELAY,
    };
    use ping_capstone::mag_cal::{Compass, Heading};
    use rtt_target::rprintln;

    type Imu = Bno085<
        SpiTransport<
            Spi<'static, Async>,
            Output<'static>,
            Input<'static>,
            Output<'static>,
            Output<'static>,
        >,
        Delay,
    >;

    /// Every report this asks for.
    const SENSORS: [SensorId; 3] = [
        SensorId::GameRotationVector,
        SensorId::MagneticFieldUncalibrated,
        SensorId::Gravity,
    ];
    const PRINT_EVERY: Duration = Duration::from_millis(500);

    fn dot(a: Vec3, b: Vec3) -> f32 {
        a.x * b.x + a.y * b.y + a.z * b.z
    }
    fn cross(a: Vec3, b: Vec3) -> Vec3 {
        Vec3 {
            x: a.y * b.z - a.z * b.y,
            y: a.z * b.x - a.x * b.z,
            z: a.x * b.y - a.y * b.x,
        }
    }
    fn norm(a: Vec3) -> f32 {
        sqrtf(dot(a, a))
    }
    fn bearing(deg: f32) -> f32 {
        if deg < 0.0 { deg + 360.0 } else { deg }
    }

    /// The latest of each report, with the compass built from them.
    struct Latest {
        compass: Compass,
        game: Option<Quaternion>,
        raw: Option<Vec3>,
        bias: Option<Vec3>,
        gravity: Option<Vec3>,
    }

    fn print(latest: &Latest, heading: Option<Heading>, started: Instant) {
        rprintln!(
            "---- t={:.1}s",
            started.elapsed().as_millis() as f32 / 1000.0
        );

        if let Some(raw) = latest.raw {
            let field = MAG_CALIBRATION.map_or(raw, |c| c.apply(raw));
            match MAG_CALIBRATION {
                Some(c) => rprintln!(
                    "raw field {:5.1} uT, hard iron offset in use {:5.1} uT",
                    norm(raw),
                    norm(Vec3 {
                        x: c.offset[0],
                        y: c.offset[1],
                        z: c.offset[2],
                    })
                ),
                None => rprintln!(
                    "raw field {:5.1} uT, NOT CALIBRATED: run imu_cal",
                    norm(raw)
                ),
            }

            if let Some(gravity) = latest.gravity {
                // Gravity as the hub reports it points up, like an
                // accelerometer at rest; down is the opposite.
                let strength = norm(field);
                let up = Vec3 {
                    x: gravity.x / norm(gravity),
                    y: gravity.y / norm(gravity),
                    z: gravity.z / norm(gravity),
                };
                let dip = asinf(-dot(field, up) / strength).to_degrees();
                // Magnetic north is the field with its vertical part taken
                // out; east is north × up. The heading of +Y is then where
                // +Y's own horizontal part falls between them.
                let vertical = dot(field, up);
                let north = Vec3 {
                    x: field.x - vertical * up.x,
                    y: field.y - vertical * up.y,
                    z: field.z - vertical * up.z,
                };
                let east = cross(north, up);
                let tilt_heading = bearing(atan2f(east.y, north.y).to_degrees());
                rprintln!(
                    "field {:5.1} uT (expect {:.1})  horizontal {:5.1} uT  dip {:5.1} deg (expect {:.1})  -> heading {:5.1}",
                    strength,
                    EXPECTED_FIELD_UT,
                    norm(north),
                    dip,
                    EXPECTED_DIP_DEG,
                    tilt_heading
                );
            }
        }
        if let Some(bias) = latest.bias {
            rprintln!(
                "hub's hard iron estimate {:5.1} uT  ({:6.1} {:6.1} {:6.1})",
                norm(bias),
                bias.x,
                bias.y,
                bias.z
            );
        }
        if let Some(heading) = heading {
            rprintln!(
                "compass (app)           heading {:5.1}  +/-{:5.1}  {:?}",
                heading.quaternion.to_euler().yaw_deg,
                heading.accuracy_rad.to_degrees(),
                heading.status
            );
        }
        if let Some(q) = latest.game {
            rprintln!(
                "game (gyro only, rel.)  heading {:5.1}",
                q.to_euler().yaw_deg
            );
        }
    }

    #[embassy_executor::task]
    pub async fn compass_task(mut imu: Imu) {
        loop {
            match imu.init().await {
                Ok(id) => {
                    rprintln!(
                        "BNO085 up: firmware {}.{}.{}, reset cause {:?}",
                        id.sw_version_major,
                        id.sw_version_minor,
                        id.sw_version_patch,
                        id.reset_cause
                    );
                    break;
                }
                Err(error) => {
                    rprintln!("BNO085 bring-up failed: {:?}, retrying", error);
                    Timer::after(RETRY_DELAY).await;
                }
            }
        }
        // The same configuration the application uses.
        if let Err(error) = imu.configure_calibration(true, false, false).await {
            rprintln!("Configuring calibration failed: {:?}", error);
        }
        for sensor in SENSORS {
            if let Err(error) = imu.enable_report(sensor, MAG_REPORT_INTERVAL_US).await {
                rprintln!("Enabling {:?} failed: {:?}", sensor, error);
            }
        }

        let started = Instant::now();
        let mut latest = Latest {
            compass: Compass::new(MAG_CALIBRATION),
            game: None,
            raw: None,
            bias: None,
            gravity: None,
        };
        let mut next_print = started + PRINT_EVERY;
        loop {
            if let Err(error) = imu.wait_for_data().await {
                rprintln!("BNO085 wait failed: {:?}", error);
                Timer::after(RETRY_DELAY).await;
                continue;
            }
            loop {
                match imu.read_event().await {
                    Ok(Some(event)) => match event.data {
                        SensorData::GameRotationVector(quaternion) => {
                            latest.compass.update_game(quaternion);
                            latest.game = Some(quaternion);
                        }
                        SensorData::MagneticFieldUncalibrated { field, bias } => {
                            latest.compass.update_field(field);
                            latest.raw = Some(field);
                            latest.bias = Some(bias);
                        }
                        SensorData::Gravity(gravity) => latest.gravity = Some(gravity),
                        _ => {}
                    },
                    Ok(None) => break,
                    Err(error) => {
                        rprintln!("BNO085 read failed: {:?}", error);
                        break;
                    }
                }
            }
            if Instant::now() >= next_print {
                next_print += PRINT_EVERY;
                print(&latest, latest.compass.heading(), started);
            }
        }
    }
}
