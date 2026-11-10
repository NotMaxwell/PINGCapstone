//! Calibrates the IMU from nothing, and prints the result to put in
//! `constants.rs`.
//!
//! This wipes what the hub has learnt, then walks through the six faces of
//! the device, one at a time:
//!
//! 1. Rest it with that face up. Holding still long enough records the
//!    accelerometer there and gives the hub the stationary spells its own
//!    accelerometer and gyroscope calibration learn from.
//! 2. Move it in a slow figure of eight for five seconds, which is what
//!    spreads the magnetometer readings around.
//!
//! The readings are then topped up until they cover nearly every direction,
//! and an ellipsoid is fitted to them, as [`ping_capstone::mag_cal`]
//! describes. The hub's accelerometer and gyroscope calibration is saved to
//! its flash; the magnetometer's lives in the firmware, so this prints it.
//!
//! `docs/heading-calibration.md` is the procedure.

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
        rprintln!("Display not answering: {:?}", error);
    }

    spawner.spawn(
        cal::cal_task(Bno085::new(transport, Delay), lcd).expect("Failed to spawn the task"),
    );

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

mod cal {
    //! The routine, in its own module so the stack frame lint can be relaxed:
    //! an embassy task's future lives in a static task pool rather than on
    //! the stack, and the readings buffer is in it.
    #![allow(clippy::large_stack_frames)]

    use alloc::format;
    use alloc::string::String;
    use core::fmt;

    use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
    use esp_hal::Async;
    use esp_hal::gpio::{Input, Output};
    use esp_hal::i2c::master::I2c;
    use esp_hal::spi::master::Spi;
    use libm::sqrtf;
    use ping_capstone::bno085::{Bno085, SensorData, SensorId, SpiTransport, Vec3};
    use ping_capstone::constants::{
        EXPECTED_FIELD_UT, IMU_SILENCE, LCD_REFRESH, MAG_REPORT_INTERVAL_US, RETRY_DELAY,
    };
    use ping_capstone::mag_cal::{
        CAPACITY, Collector, FIELD_TOLERANCE, MAX_RESIDUAL, MagCalibration, REQUIRED_SECTORS,
        SECTORS, coverage, fit,
    };
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

    /// The faces, in the order they are asked for, and which way an
    /// accelerometer at rest reads with that face up. It reads upwards.
    const FACES: [(&str, [f32; 3]); 6] = [
        ("+X up", [1.0, 0.0, 0.0]),
        ("-X up", [-1.0, 0.0, 0.0]),
        ("+Y up", [0.0, 1.0, 0.0]),
        ("-Y up", [0.0, -1.0, 0.0]),
        ("+Z up", [0.0, 0.0, 1.0]),
        ("-Z up", [0.0, 0.0, -1.0]),
    ];

    /// How many accelerometer readings must be held still: 1.5 s at
    /// [`MAG_REPORT_INTERVAL_US`].
    const HOLD_SAMPLES: u32 = 75;
    /// The most any axis may wander, m/s², and still count as held still.
    const STILL_LIMIT: f32 = 0.25;
    /// How closely the face must point up: cos 10°. A six point check reads
    /// the projection of gravity on each axis, so a tilt reads as a scale
    /// error.
    const ALIGNED: f32 = 0.985;
    /// How long to move the device about at each face.
    const SWEEP: Duration = Duration::from_secs(5);
    /// Standard gravity, m/s².
    const GRAVITY: f32 = 9.80665;

    /// Shows `lines` on the display and over RTT. A failed display write is
    /// logged and otherwise ignored.
    async fn show(lcd: &mut Display, lines: &[fmt::Arguments<'_>]) {
        let mut separator = "";
        for line in lines {
            rprint!("{}{}", separator, line);
            separator = " | ";
        }
        rprintln!();
        show_quietly(lcd, lines).await;
    }

    /// Writes `lines` to the display alone, for progress that changes
    /// several times a second.
    async fn show_quietly(lcd: &mut Display, lines: &[fmt::Arguments<'_>]) {
        for row in 0..ROWS {
            let written = match lines.get(row) {
                Some(&line) => lcd.print_line(row, line).await,
                None => lcd.write_line(row, "").await,
            };
            if written.is_err() {
                return;
            }
        }
    }

    /// Whether it is time to rewrite the display.
    fn due(next_refresh: &mut Instant) -> bool {
        let now = Instant::now();
        if now < *next_refresh {
            return false;
        }
        *next_refresh = now + LCD_REFRESH;
        true
    }

    /// Takes reports until the next accelerometer reading, keeping any field
    /// readings that arrive in the meantime.
    async fn next_accel(imu: &mut Imu, readings: &mut Collector) -> Vec3 {
        loop {
            match imu.read_event().await {
                Ok(Some(event)) => match event.data {
                    SensorData::Accelerometer(accel) => return accel,
                    SensorData::MagneticFieldUncalibrated { field, .. } => {
                        readings.add(field);
                    }
                    _ => {}
                },
                Ok(None) => match with_timeout(IMU_SILENCE, imu.wait_for_data()).await {
                    Ok(Ok(())) => {}
                    Ok(Err(error)) => {
                        rprintln!("BNO085 wait failed: {:?}", error);
                        Timer::after(RETRY_DELAY).await;
                    }
                    Err(_) => rprintln!("BNO085 went quiet"),
                },
                Err(error) => {
                    rprintln!("BNO085 read failed: {:?}", error);
                    Timer::after(RETRY_DELAY).await;
                }
            }
        }
    }

    /// A run of accelerometer readings that have stayed together.
    struct Hold {
        sum: [f32; 3],
        min: [f32; 3],
        max: [f32; 3],
        count: u32,
    }

    impl Hold {
        const fn new() -> Self {
            Self {
                sum: [0.0; 3],
                min: [f32::MAX; 3],
                max: [f32::MIN; 3],
                count: 0,
            }
        }

        /// Adds a reading, starting over if it has moved too far from the
        /// others.
        fn add(&mut self, accel: Vec3) {
            let values = [accel.x, accel.y, accel.z];
            for (axis, value) in values.into_iter().enumerate() {
                self.min[axis] = self.min[axis].min(value);
                self.max[axis] = self.max[axis].max(value);
                if self.max[axis] - self.min[axis] > STILL_LIMIT {
                    *self = Self::new();
                    return;
                }
            }
            for (sum, value) in self.sum.iter_mut().zip(values) {
                *sum += value;
            }
            self.count += 1;
        }

        fn mean(&self) -> [f32; 3] {
            self.sum.map(|s| s / self.count as f32)
        }
    }

    fn aligned(accel: Vec3, up: [f32; 3]) -> bool {
        let length = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
        length > 0.0 && (accel.x * up[0] + accel.y * up[1] + accel.z * up[2]) / length > ALIGNED
    }

    /// Waits for the device to be held still with this face up. Returns the
    /// accelerometer's average there.
    async fn hold_face(
        imu: &mut Imu,
        lcd: &mut Display,
        readings: &mut Collector,
        number: usize,
        name: &str,
        up: [f32; 3],
    ) -> [f32; 3] {
        rprintln!(
            "Face {}/{}: {}, rest it there and hold still",
            number,
            FACES.len(),
            name
        );
        let mut hold = Hold::new();
        let mut next_refresh = Instant::now();
        loop {
            let accel = next_accel(imu, readings).await;
            let on_face = aligned(accel, up);
            if on_face {
                hold.add(accel);
            } else {
                hold = Hold::new();
            }
            if hold.count >= HOLD_SAMPLES {
                return hold.mean();
            }
            if due(&mut next_refresh) {
                // Built first: a format_args! lives only to the end of the
                // statement that makes it, so one cannot come out of an `if`.
                let status = if on_face {
                    format!("hold still {}/{}", hold.count / 15, HOLD_SAMPLES / 15)
                } else {
                    String::from("turn to face")
                };
                show_quietly(
                    lcd,
                    &[
                        format_args!("face {}/{}", number, FACES.len()),
                        format_args!("{}", name),
                        format_args!("{status}"),
                    ],
                )
                .await;
            }
        }
    }

    /// Collects field readings while the device is moved about at one face.
    async fn sweep_face(
        imu: &mut Imu,
        lcd: &mut Display,
        readings: &mut Collector,
        number: usize,
        name: &str,
    ) {
        rprintln!("Face {}/{}: slow figure of eight", number, FACES.len());
        let end = Instant::now() + SWEEP;
        let mut next_refresh = Instant::now();
        while Instant::now() < end {
            next_accel(imu, readings).await;
            if due(&mut next_refresh) {
                let left = (end - Instant::now()).as_secs() + 1;
                show_quietly(
                    lcd,
                    &[
                        format_args!("face {}/{}", number, FACES.len()),
                        format_args!("{}", name),
                        format_args!("figure 8 {left} s"),
                        format_args!("cover {}/{}", readings.covered(), SECTORS),
                    ],
                )
                .await;
            }
        }
    }

    /// Prints the six point check: from each axis's reading up and down,
    /// `(up + down) / 2` is its bias and `g / ((up - down) / 2)` its scale.
    /// The hub keeps its own accelerometer calibration, so this is a check
    /// of it rather than something to apply.
    fn report_accel(means: &[[f32; 3]; 6]) {
        rprintln!("---- accelerometer, six point check (bias should be near 0, scale near 1)");
        for (axis, name) in ["X", "Y", "Z"].into_iter().enumerate() {
            let up = means[axis * 2][axis];
            let down = means[axis * 2 + 1][axis];
            rprintln!(
                "{}: up {:+7.3}  down {:+7.3}  bias {:+.3} m/s2  scale {:.4}",
                name,
                up,
                down,
                (up + down) / 2.0,
                GRAVITY / ((up - down) / 2.0)
            );
        }
    }

    /// Prints the fit, and returns why it should not be used, if it should
    /// not.
    fn report_fit(
        calibration: &MagCalibration,
        sectors: usize,
        samples: usize,
    ) -> Option<&'static str> {
        let [bx, by, bz] = calibration.offset;
        let [r0, r1, r2] = calibration.matrix;
        rprintln!("---- magnetometer fit, {} readings", samples);
        rprintln!(
            "sectors covered {}/{} (need {})",
            sectors,
            SECTORS,
            REQUIRED_SECTORS
        );
        rprintln!("hard iron offset {:.2} {:.2} {:.2} uT", bx, by, bz);
        rprintln!("soft iron matrix {:.4} {:.4} {:.4}", r0[0], r0[1], r0[2]);
        rprintln!("                 {:.4} {:.4} {:.4}", r1[0], r1[1], r1[2]);
        rprintln!("                 {:.4} {:.4} {:.4}", r2[0], r2[1], r2[2]);
        rprintln!(
            "field strength {:.1} uT (expect {:.1}), residual {:.4} (limit {:.4})",
            calibration.field_ut,
            EXPECTED_FIELD_UT,
            calibration.residual,
            MAX_RESIDUAL
        );

        if sectors < REQUIRED_SECTORS {
            Some("too few directions")
        } else if calibration.residual > MAX_RESIDUAL {
            Some("residual too big")
        } else if (calibration.field_ut - EXPECTED_FIELD_UT).abs()
            > FIELD_TOLERANCE * EXPECTED_FIELD_UT
        {
            Some("wrong field")
        } else {
            None
        }
    }

    /// Prints the calibration in the form it goes in `constants.rs`.
    fn print_constant(c: &MagCalibration) {
        let [r0, r1, r2] = c.matrix;
        rprintln!("Put this in src/constants.rs:");
        rprintln!("    pub const MAG_CALIBRATION: Option<MagCalibration> = Some(MagCalibration {{");
        rprintln!(
            "        offset: [{:.4}, {:.4}, {:.4}],",
            c.offset[0],
            c.offset[1],
            c.offset[2]
        );
        rprintln!("        matrix: [");
        rprintln!("            [{:.6}, {:.6}, {:.6}],", r0[0], r0[1], r0[2]);
        rprintln!("            [{:.6}, {:.6}, {:.6}],", r1[0], r1[1], r1[2]);
        rprintln!("            [{:.6}, {:.6}, {:.6}],", r2[0], r2[1], r2[2]);
        rprintln!("        ],");
        rprintln!("        field_ut: {:.3},", c.field_ut);
        rprintln!("        residual: {:.5},", c.residual);
        rprintln!("    }});");
    }

    /// Resets the hub, wipes its calibration and starts the reports the
    /// sequence reads.
    async fn start(imu: &mut Imu) -> Result<(), &'static str> {
        imu.init().await.map_err(|_| "no reply")?;
        imu.clear_calibration().await.map_err(|_| "clear refused")?;
        // Accelerometer and gyroscope only. The magnetometer is calibrated
        // here instead.
        imu.configure_calibration(true, true, false)
            .await
            .map_err(|_| "calibrate refused")?;
        imu.enable_report(SensorId::Accelerometer, MAG_REPORT_INTERVAL_US)
            .await
            .map_err(|_| "report refused")?;
        imu.enable_report(SensorId::MagneticFieldUncalibrated, MAG_REPORT_INTERVAL_US)
            .await
            .map_err(|_| "report refused")
    }

    /// Runs the sequence once. Reset the board to run it again.
    #[embassy_executor::task]
    pub async fn cal_task(mut imu: Imu, mut lcd: Display) {
        show(
            &mut lcd,
            &[
                format_args!("IMU calibration"),
                format_args!("clearing hub"),
            ],
        )
        .await;
        while let Err(why) = start(&mut imu).await {
            show(
                &mut lcd,
                &[format_args!("BNO085 {why}"), format_args!("retrying")],
            )
            .await;
            Timer::after(RETRY_DELAY).await;
        }
        rprintln!("Hub calibration cleared.");

        let mut readings = Collector::new();
        let mut means = [[0.0; 3]; 6];
        for (index, &(name, up)) in FACES.iter().enumerate() {
            let number = index + 1;
            means[index] = hold_face(&mut imu, &mut lcd, &mut readings, number, name, up).await;
            sweep_face(&mut imu, &mut lcd, &mut readings, number, name).await;
        }

        // The six faces reach most directions. The rest are the ones
        // between them, which only a tumble gets to.
        rprintln!("Keep turning the device through every direction.");
        let mut next_refresh = Instant::now();
        while readings.covered() < REQUIRED_SECTORS && !readings.is_full() {
            next_accel(&mut imu, &mut readings).await;
            if due(&mut next_refresh) {
                show_quietly(
                    &mut lcd,
                    &[
                        format_args!("tumble slowly"),
                        format_args!("all directions"),
                        format_args!("cover {}/{}", readings.covered(), SECTORS),
                        format_args!("{}/{} kept", readings.samples().len(), CAPACITY),
                    ],
                )
                .await;
            }
        }

        report_accel(&means);

        // Saved whatever the fit makes of the magnetometer: it is the hub's
        // accelerometer and gyroscope calibration, and it replaces whatever
        // was in flash.
        let saved = imu.save_calibration().await;
        match &saved {
            Ok(()) => {
                rprintln!("The IMU confirmed saving its accelerometer and gyroscope calibration.")
            }
            Err(error) => rprintln!("Saving the hub's calibration failed: {:?}", error),
        }

        show(&mut lcd, &[format_args!("fitting...")]).await;
        match fit(readings.samples()) {
            Ok(calibration) => {
                let sectors = coverage(readings.samples(), calibration.offset);
                let problem = report_fit(&calibration, sectors, readings.samples().len());
                match problem {
                    None => {
                        print_constant(&calibration);
                        show(
                            &mut lcd,
                            &[
                                format_args!("cal good"),
                                format_args!("field {:.1} uT", calibration.field_ut),
                                format_args!("resid {:.4}", calibration.residual),
                                format_args!(
                                    "{}",
                                    if saved.is_ok() {
                                        "copy from RTT"
                                    } else {
                                        "hub save failed"
                                    }
                                ),
                            ],
                        )
                        .await;
                    }
                    Some(why) => {
                        rprintln!(
                            "Not usable: {}. Move away from iron and magnets and run it again.",
                            why
                        );
                        show(
                            &mut lcd,
                            &[
                                format_args!("cal failed"),
                                format_args!("{}", why),
                                format_args!("field {:.1} uT", calibration.field_ut),
                                format_args!("resid {:.4}", calibration.residual),
                            ],
                        )
                        .await;
                    }
                }
            }
            Err(error) => {
                rprintln!("The readings could not be fitted: {:?}", error);
                show(
                    &mut lcd,
                    &[format_args!("fit failed"), format_args!("{:?}", error)],
                )
                .await;
            }
        }
    }
}
