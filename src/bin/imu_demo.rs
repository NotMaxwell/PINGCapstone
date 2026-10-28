//! Standalone demo binary for the BNO085 driver.
//!
//! Brings up the sensor hub over SPI, starts the rotation vector at 100 Hz and
//! shows the compass heading on the display and over RTT, five times a
//! second. Kept separate from `main.rs` so the application entry point stays
//! clear of sensor bring-up code.
//!
//! The heading is the direction [`LIDAR_BORESIGHT`] points, worked out the
//! same way the application aims, by [`Aim::from_orientation`]. It is shown
//! from true north, using [`MAGNETIC_DECLINATION_DEG`], with the nearest of
//! the 16 compass points, and again from magnetic north. The hub fuses the
//! accelerometer with the magnetometer, so the heading holds while the device
//! is tilted. Pitch and roll are those of the breakout itself; see
//! [`Quaternion::to_euler`].
//!
//! [`LIDAR_BORESIGHT`]: ping_capstone::constants::LIDAR_BORESIGHT
//! [`MAGNETIC_DECLINATION_DEG`]: ping_capstone::constants::MAGNETIC_DECLINATION_DEG
//! [`Aim::from_orientation`]: ping_capstone::geo::Aim::from_orientation
//! [`Quaternion::to_euler`]: ping_capstone::bno085::Quaternion::to_euler

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
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use ping_capstone::bno085::{Bno085, SpiTransport};
use ping_capstone::constants::{
    HEAP_SIZE, IMU_SPI_FREQUENCY_KHZ, LCD_ADDRESS, LCD_COLUMN_OFFSET, LCD_I2C_FREQUENCY_KHZ,
};
use ping_capstone::ssd1306::Ssd1306;
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

    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    // Pins come from ping_capstone::constants. The BNO085 runs in SPI mode
    // 3: PS1 is tied to 3V3 on the board, and WAKE (PS0) starts high so the
    // hub selects SPI when the driver releases its reset line. INT is open
    // drain and active low, so it needs a pull-up.
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

    // Display: the OLED on I2C0. A missing or miswired display only
    // costs the demo its screen; the readings still go over RTT.
    let lcd_i2c = I2c::new(
        board_bus!(peripherals.LCD_I2C),
        I2cConfig::default().with_frequency(Rate::from_khz(LCD_I2C_FREQUENCY_KHZ)),
    )
    .expect("Failed to configure the display I2C bus")
    .with_sda(board_pin!(peripherals.LCD_SDA))
    .with_scl(board_pin!(peripherals.LCD_SCL))
    .into_async();
    let mut lcd = Ssd1306::new(lcd_i2c, LCD_ADDRESS, LCD_COLUMN_OFFSET);
    if let Err(error) = lcd.init().await {
        rprintln!(
            "Display not answering at {:#04x}: {:?}; I2C devices found: {}",
            LCD_ADDRESS,
            error,
            lcd.scan().await
        );
    }

    spawner
        .spawn(imu_task(Bno085::new(transport, Delay), lcd).expect("Failed to spawn the IMU task"));

    // Everything happens in the task.
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

use imu::imu_task;

mod imu {
    //! The IMU task lives in its own module so that the stack frame lint can
    //! be relaxed for it. An embassy task's future is allocated once in a
    //! static task pool rather than on the stack, and most of its size here
    //! is the HAL's SPI and I2C futures and the formatting machinery behind
    //! rprintln!, not anything this code puts on the stack.
    #![allow(clippy::large_stack_frames)]

    use core::fmt;

    use embassy_time::{Delay, Instant, Timer};
    use esp_hal::Async;
    use esp_hal::gpio::{Input, Output};
    use esp_hal::i2c::master::I2c;
    use esp_hal::spi::master::Spi;
    use ping_capstone::bno085::{Accuracy, Bno085, SensorData, SensorId, SpiTransport};
    use ping_capstone::constants::{
        IMU_REPORT_INTERVAL_US, LCD_REFRESH, LIDAR_BORESIGHT, MAGNETIC_DECLINATION_DEG, RETRY_DELAY,
    };
    use ping_capstone::geo::Aim;
    use ping_capstone::ssd1306::{ROWS, Ssd1306};
    use rtt_target::{rprint, rprintln};

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

    type Display = Ssd1306<I2c<'static, Async>>;

    /// Shows `lines` on the display, one per row, blanking the rows below
    /// them, and prints them over RTT on one line. A failed display write is
    /// logged and otherwise ignored, so a loose display never stops the
    /// IMU being read.
    async fn show(lcd: &mut Display, lines: &[fmt::Arguments<'_>]) {
        let mut separator = "";
        for line in lines {
            rprint!("{}{}", separator, line);
            separator = " | ";
        }
        rprintln!();

        for row in 0..ROWS {
            let written = match lines.get(row) {
                Some(&line) => lcd.print_line(row, line).await,
                None => lcd.write_line(row, "").await,
            };
            if let Err(error) = written {
                rprintln!("Display write failed: {:?}", error);
                return;
            }
        }
    }

    /// The nearest of the 16 points of the compass to `bearing_deg`, degrees
    /// clockwise from north.
    fn compass_point(bearing_deg: f32) -> &'static str {
        const POINTS: [&str; 16] = [
            "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW",
            "NW", "NNW",
        ];
        // Each point covers 22.5°, centred on its own bearing. The cast
        // truncates towards zero and saturates, so a bearing a hair below
        // zero still lands on north.
        let sector = (bearing_deg / 22.5 + 0.5) as usize;
        POINTS[sector % POINTS.len()]
    }

    /// Whether it is time to show the next reading, at most once every
    /// [`LCD_REFRESH`]. The sensors report faster than anyone can read, and
    /// each rewrite of the display holds up the task for a few milliseconds.
    fn due(next_refresh: &mut Instant) -> bool {
        let now = Instant::now();
        if now < *next_refresh {
            return false;
        }
        *next_refresh = now + LCD_REFRESH;
        true
    }

    /// Reads orientation from the BNO085 and shows it.
    ///
    /// The sensor pulls INT low when it has something to say, and holds it low
    /// until everything waiting has been read, so each wake-up drains the hub
    /// before going back to sleep.
    #[embassy_executor::task]
    pub async fn imu_task(mut imu: Imu, mut lcd: Display) {
        show(
            &mut lcd,
            &[format_args!("BNO085"), format_args!("starting")],
        )
        .await;
        loop {
            match imu.init().await {
                Ok(id) => {
                    show(
                        &mut lcd,
                        &[
                            format_args!("BNO085 up"),
                            format_args!(
                                "fw {}.{}.{}",
                                id.sw_version_major, id.sw_version_minor, id.sw_version_patch
                            ),
                            format_args!("reset {:?}", id.reset_cause),
                        ],
                    )
                    .await;
                    break;
                }
                Err(error) => {
                    show(
                        &mut lcd,
                        &[
                            format_args!("BNO085 no reply"),
                            format_args!("{:?}", error),
                            format_args!("retrying"),
                        ],
                    )
                    .await;
                    Timer::after(RETRY_DELAY).await;
                }
            }
        }

        if let Err(error) = imu
            .enable_report(SensorId::RotationVector, IMU_REPORT_INTERVAL_US)
            .await
        {
            show(
                &mut lcd,
                &[
                    format_args!("BNO085 refused"),
                    format_args!("to report"),
                    format_args!("{:?}", error),
                ],
            )
            .await;
            return;
        }

        let mut next_refresh = Instant::now();
        loop {
            if let Err(error) = imu.wait_for_data().await {
                show(
                    &mut lcd,
                    &[format_args!("BNO085 wait"), format_args!("{:?}", error)],
                )
                .await;
                Timer::after(RETRY_DELAY).await;
                continue;
            }

            loop {
                match imu.read_event().await {
                    Ok(Some(event)) => {
                        let SensorData::RotationVector {
                            quaternion,
                            accuracy_rad,
                        } = event.data
                        else {
                            continue;
                        };
                        if !due(&mut next_refresh) {
                            continue;
                        }

                        let heading = Aim::from_orientation(
                            &quaternion,
                            LIDAR_BORESIGHT,
                            MAGNETIC_DECLINATION_DEG,
                        )
                        .azimuth_deg;
                        let magnetic =
                            Aim::from_orientation(&quaternion, LIDAR_BORESIGHT, 0.0).azimuth_deg;
                        let angles = quaternion.to_euler();
                        let accuracy = match event.accuracy {
                            Accuracy::High => "high",
                            Accuracy::Medium => "med",
                            Accuracy::Low => "low",
                            Accuracy::Unreliable => "bad",
                        };
                        show(
                            &mut lcd,
                            &[
                                format_args!("hdg {:>6.1}° {}", heading, compass_point(heading)),
                                format_args!("mag {:>6.1}°", magnetic),
                                format_args!(
                                    "p{:>+6.1} r{:>+7.1}",
                                    angles.pitch_deg, angles.roll_deg
                                ),
                                format_args!("acc {} ±{:.1}°", accuracy, accuracy_rad.to_degrees()),
                            ],
                        )
                        .await;
                    }
                    // Nothing left waiting; go back to the interrupt pin.
                    Ok(None) => break,
                    Err(error) => {
                        show(
                            &mut lcd,
                            &[format_args!("BNO085 read"), format_args!("{:?}", error)],
                        )
                        .await;
                        break;
                    }
                }
            }
        }
    }
}
