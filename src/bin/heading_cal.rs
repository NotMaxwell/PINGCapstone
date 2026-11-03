//! Measures the heading correction: how far the bearing the device reports
//! sits from the bearing it is actually pointing along.
//!
//! The IMU's rotation vector takes its heading from the magnetometer, so the
//! bearings it gives are measured from magnetic north, and they also carry
//! however crookedly the breakout is mounted against the LiDAR and whatever
//! magnetised iron rides along with it. Those are three different causes with
//! one effect, and the device cannot tell them apart. This measures their sum
//! against bearings that are known, which is the only form of it that can be
//! checked.
//!
//! The device is aimed in turn at each bearing in [`STATIONS`] and the button
//! pressed. Each press averages the heading for [`SAMPLE_WINDOW`] and records
//! the error. After the last station this prints a table over RTT and the
//! value to put in [`HEADING_CORRECTION_DEG`], and another press starts a new
//! run.
//!
//! The spread of the errors matters as much as their average. An error that
//! is the same at every station is a constant, and one number removes it. An
//! error that changes with the heading is not: it is iron or a current near
//! the magnetometer bending the field, and no correction will straighten it
//! out. This says which of the two is happening; `compass_diag` then says
//! where the bent field comes from.
//!
//! `docs/heading-calibration.md` is the procedure, including how to come by
//! the known bearings in the first place.
//!
//! [`HEADING_CORRECTION_DEG`]: ping_capstone::constants::HEADING_CORRECTION_DEG

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

/// The bearings the device is aimed along, in the order it is aimed along
/// them, degrees clockwise from true north.
///
/// Four of them, a quarter turn apart, is enough to tell a constant error
/// from one that changes with the heading, which is the whole point of
/// measuring more than one. Replace them with the bearings actually
/// available: whatever is known about where the device stands, a line on a
/// map between two landmarks, a surveyed baseline. They need not be evenly
/// spaced, but they must between them cover a good part of the circle, and
/// every one of them must be a *true* bearing, not a magnetic one off a
/// handheld compass.
const STATIONS: [f32; 4] = [0.0, 90.0, 180.0, 270.0];

/// How long each station's heading is averaged. At 100 reports a second this
/// is over a hundred samples, which takes the noise down without asking
/// anyone to hold still for long.
const SAMPLE_WINDOW: Duration = Duration::from_millis(1_500);

/// A station whose heading wandered more than this while it was being
/// averaged was not held still, and is offered again.
const STEADY_LIMIT_DEG: f32 = 2.0;

/// Past this far from level, the magnetometer's share of the heading comes
/// increasingly from the vertical part of the field, which is steep here and
/// makes a tilt read as a turn. Noted on the station's line.
const LEVEL_LIMIT_DEG: f32 = 10.0;

/// Errors that differ by more than this between stations are not a constant,
/// so no single correction removes them.
const TRIM_LIMIT_DEG: f32 = 5.0;

/// How long a station's result stays on the display before the next station
/// is called for.
const RESULT_HOLD: Duration = Duration::from_millis(2_000);

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

    // Wi-Fi is deliberately not started, as in compass_diag: its current
    // draw and its RF are both things that can bend a heading, and a
    // correction measured with them off is the one the application wants
    // only if the application has them off too. If the finished device runs
    // Wi-Fi while aiming, measure again with it on.
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

    // The same trigger button the application uses, wired from its GPIO to
    // ground.
    let button = Input::new(
        board_pin!(peripherals.BUTTON),
        InputConfig::default().with_pull(Pull::Up),
    );

    // Display: a missing or miswired one only costs the routine its prompts;
    // everything it measures goes over RTT as well.
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

    spawner.spawn(
        cal::cal_task(Bno085::new(transport, Delay), lcd, button)
            .expect("Failed to spawn the calibration task"),
    );

    // Everything happens in the task.
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

mod cal {
    //! The calibration routine, in its own module so the stack frame lint
    //! can be relaxed: an embassy task's future lives in a static task pool
    //! rather than on the stack, and most of its size here is the HAL's SPI
    //! and I2C futures and the formatting machinery behind rprintln!.
    #![allow(clippy::large_stack_frames)]

    use super::{
        LEVEL_LIMIT_DEG, RESULT_HOLD, SAMPLE_WINDOW, STATIONS, STEADY_LIMIT_DEG, TRIM_LIMIT_DEG,
    };
    use alloc::format;
    use alloc::string::String;
    use core::fmt;

    use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
    use esp_hal::Async;
    use esp_hal::gpio::{Input, Output};
    use esp_hal::i2c::master::I2c;
    use esp_hal::spi::master::Spi;
    use libm::{atan2f, cosf, hypotf, logf, sinf, sqrtf};
    use ping_capstone::bno085::{Accuracy, Bno085, SensorData, SensorId, SpiTransport};
    use ping_capstone::constants::{
        DEBOUNCE, HEADING_CORRECTION_DEG, IMU_REPORT_INTERVAL_US, LCD_REFRESH, LIDAR_BORESIGHT,
        RETRY_DELAY,
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

    /// How often the button is looked at while the device is being aimed.
    const BUTTON_POLL: Duration = Duration::from_millis(20);

    /// How long to leave the hub alone when it has nothing waiting. It
    /// reports every [`IMU_REPORT_INTERVAL_US`], so this is a fraction of the
    /// gap between reports.
    const IDLE_POLL: Duration = Duration::from_millis(2);

    /// `deg` folded into (-180, 180], which is the range a heading error
    /// belongs in: 359° of error is 1° the other way.
    fn wrap180(mut deg: f32) -> f32 {
        while deg > 180.0 {
            deg -= 360.0;
        }
        while deg <= -180.0 {
            deg += 360.0;
        }
        deg
    }

    /// Ranks what the hub thinks of its compass, so two reports' opinions can
    /// be compared. [`Accuracy`] has no ordering of its own.
    fn rank(accuracy: Accuracy) -> u8 {
        match accuracy {
            Accuracy::Unreliable => 0,
            Accuracy::Low => 1,
            Accuracy::Medium => 2,
            Accuracy::High => 3,
        }
    }

    /// What the operator should do next, given what the hub makes of its
    /// compass: a heading measured on an uncalibrated magnetometer is not
    /// worth recording.
    fn prompt(accuracy: Accuracy) -> &'static str {
        if accuracy == Accuracy::High {
            "press button"
        } else {
            "fig 8 first"
        }
    }

    fn short(accuracy: Accuracy) -> &'static str {
        match accuracy {
            Accuracy::Unreliable => "bad",
            Accuracy::Low => "low",
            Accuracy::Medium => "med",
            Accuracy::High => "high",
        }
    }

    /// A running average of angles, taken through their sines and cosines so
    /// that 359° and 1° average to 0° rather than to 180°.
    #[derive(Clone, Copy, Default)]
    struct Angles {
        sin: f32,
        cos: f32,
        count: u32,
    }

    impl Angles {
        fn add(&mut self, deg: f32) {
            let rad = deg.to_radians();
            self.sin += sinf(rad);
            self.cos += cosf(rad);
            self.count += 1;
        }

        /// The average, in (-180, 180].
        fn mean_deg(&self) -> f32 {
            atan2f(self.sin, self.cos).to_degrees()
        }

        /// How widely the angles scattered about their average, degrees: the
        /// circular standard deviation. Zero when every one of them agreed.
        /// Only meaningful once something has been added.
        fn spread_deg(&self) -> f32 {
            let resultant = hypotf(self.sin, self.cos) / self.count as f32;
            // Rounding can put the resultant a hair above 1, where the
            // logarithm turns positive and the root imaginary.
            sqrtf(-2.0 * logf(resultant.clamp(f32::MIN_POSITIVE, 1.0))).to_degrees()
        }
    }

    /// What one station's averaging came to.
    #[derive(Clone, Copy)]
    struct Reading {
        /// The bearing the device reported, 0 up to 360, worked out exactly
        /// as the application works it out.
        heading_deg: f32,
        /// How far the heading wandered while it was being averaged.
        steady_deg: f32,
        elevation_deg: f32,
        roll_deg: f32,
        /// The worst the hub thought of its compass over the window.
        calibration: Accuracy,
        /// The hub's own estimate of its heading error, averaged.
        accuracy_deg: f32,
    }

    /// One station: where the device was pointed, and what it said.
    #[derive(Clone, Copy)]
    struct Station {
        /// The known bearing the device was aimed along.
        reference_deg: f32,
        reading: Reading,
        /// Reported minus known, in (-180, 180]. This is what a correction
        /// has to remove.
        error_deg: f32,
    }

    /// Shows `lines` on the display, one per row, blanking the rows below
    /// them, and prints them over RTT on one line. A failed display write is
    /// logged and otherwise ignored, so a loose display never stops the
    /// routine.
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

    /// Writes `lines` to the display alone, for the live readout while the
    /// device is being aimed: it is rewritten several times a second, and
    /// putting all of that over RTT would bury everything worth reading.
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

    /// Whether it is time to rewrite the display, at most once every
    /// [`LCD_REFRESH`].
    fn due(next_refresh: &mut Instant) -> bool {
        let now = Instant::now();
        if now < *next_refresh {
            return false;
        }
        *next_refresh = now + LCD_REFRESH;
        true
    }

    /// Everything the hub has waiting, decoded, with the last rotation vector
    /// of the lot handed back. Reading when nothing is pending is harmless,
    /// so this needs no interrupt and can be dropped into any wait.
    async fn drain(imu: &mut Imu, saved: &mut bool) -> Option<Reading> {
        let mut latest = None;
        loop {
            match imu.read_event().await {
                Ok(Some(event)) => {
                    if let SensorData::RotationVector {
                        quaternion,
                        accuracy_rad,
                    } = event.data
                    {
                        let aim = Aim::from_orientation(
                            &quaternion,
                            LIDAR_BORESIGHT,
                            HEADING_CORRECTION_DEG,
                        );
                        latest = Some(Reading {
                            heading_deg: aim.azimuth_deg,
                            steady_deg: 0.0,
                            elevation_deg: aim.elevation_deg,
                            roll_deg: quaternion.to_euler().roll_deg,
                            calibration: event.accuracy,
                            accuracy_deg: accuracy_rad.to_degrees(),
                        });
                        // Saved the first time the hub is confident, so the
                        // next boot — and the application — starts from the
                        // calibration this run was measured against.
                        if event.accuracy == Accuracy::High && !*saved {
                            *saved = true;
                            match imu.save_calibration().await {
                                Ok(()) => rprintln!("Compass calibrated; saved to the IMU"),
                                Err(error) => {
                                    rprintln!("Saving the compass calibration failed: {:?}", error)
                                }
                            }
                        }
                    }
                }
                // Nothing left waiting.
                Ok(None) => return latest,
                Err(error) => {
                    rprintln!("BNO085 read failed: {:?}", error);
                    return latest;
                }
            }
        }
    }

    /// Shows the live heading until the button is pressed, so the device can
    /// be aimed, keeping the hub drained while it waits.
    async fn aim_at(
        imu: &mut Imu,
        lcd: &mut Display,
        button: &mut Input<'static>,
        saved: &mut bool,
        station: usize,
        reference_deg: f32,
    ) {
        let mut next_refresh = Instant::now();
        let mut latest: Option<Reading> = None;
        rprintln!(
            "Station {}/{}: aim along {:.1} deg and press the button",
            station + 1,
            STATIONS.len(),
            reference_deg
        );
        loop {
            if with_timeout(BUTTON_POLL, button.wait_for_low())
                .await
                .is_ok()
            {
                Timer::after(DEBOUNCE).await;
                if button.is_low() {
                    return;
                }
                // Too short to be a press: noise, or contacts bouncing.
                continue;
            }

            if let Some(reading) = drain(imu, saved).await {
                latest = Some(reading);
            }
            if !due(&mut next_refresh) {
                continue;
            }
            match latest {
                Some(reading) => {
                    show_quietly(
                        lcd,
                        &[
                            format_args!(
                                "stn {}/{} {}",
                                station + 1,
                                STATIONS.len(),
                                short(reading.calibration)
                            ),
                            format_args!("aim {:.1}", reference_deg),
                            format_args!("now {:.1}", reading.heading_deg),
                            format_args!("{}", prompt(reading.calibration)),
                        ],
                    )
                    .await;
                }
                None => {
                    show_quietly(
                        lcd,
                        &[
                            format_args!("stn {}/{}", station + 1, STATIONS.len()),
                            format_args!("aim {:.1}", reference_deg),
                            format_args!("no IMU report"),
                        ],
                    )
                    .await;
                }
            }
        }
    }

    /// Averages the heading over [`SAMPLE_WINDOW`], along with the elevation,
    /// the roll and what the hub makes of its own compass. `None` if the hub
    /// sent no rotation vector in all that time.
    async fn measure(imu: &mut Imu, saved: &mut bool) -> Option<Reading> {
        let deadline = Instant::now() + SAMPLE_WINDOW;
        let mut heading = Angles::default();
        let mut elevation_deg = 0.0;
        let mut roll_deg = 0.0;
        let mut accuracy_deg = 0.0;
        let mut calibration = Accuracy::High;
        let mut count = 0u32;

        while Instant::now() < deadline {
            let Some(reading) = drain(imu, saved).await else {
                Timer::after(IDLE_POLL).await;
                continue;
            };
            heading.add(reading.heading_deg);
            elevation_deg += reading.elevation_deg;
            roll_deg += reading.roll_deg;
            accuracy_deg += reading.accuracy_deg;
            if rank(reading.calibration) < rank(calibration) {
                calibration = reading.calibration;
            }
            count += 1;
            Timer::after(IDLE_POLL).await;
        }
        if count == 0 {
            return None;
        }

        let samples = count as f32;
        let mut heading_deg = heading.mean_deg();
        if heading_deg < 0.0 {
            heading_deg += 360.0;
        }
        Some(Reading {
            heading_deg,
            steady_deg: heading.spread_deg(),
            elevation_deg: elevation_deg / samples,
            roll_deg: roll_deg / samples,
            calibration,
            accuracy_deg: accuracy_deg / samples,
        })
    }

    /// Waits for the button to be let go and to stay up past any contact
    /// bounce, keeping the hub drained while it waits.
    async fn wait_for_release(imu: &mut Imu, button: &mut Input<'static>, saved: &mut bool) {
        loop {
            while button.is_low() {
                let _ = with_timeout(BUTTON_POLL, button.wait_for_high()).await;
                let _ = drain(imu, saved).await;
            }
            Timer::after(DEBOUNCE).await;
            if button.is_high() {
                return;
            }
        }
    }

    /// Prints the run's table, works out the correction it implies and says
    /// whether a correction is the right fix at all.
    async fn report(stations: &[Option<Station>], lcd: &mut Display) {
        let mut errors = Angles::default();
        let mut worst = Accuracy::High;
        let mut off_level = false;
        for station in stations.iter().flatten() {
            errors.add(station.error_deg);
            if rank(station.reading.calibration) < rank(worst) {
                worst = station.reading.calibration;
            }
            if station.reading.elevation_deg.abs() > LEVEL_LIMIT_DEG
                || station.reading.roll_deg.abs() > LEVEL_LIMIT_DEG
            {
                off_level = true;
            }
        }
        if errors.count == 0 {
            return;
        }

        // The spread about the average, measured peak to peak: how much of
        // the error is not constant, and so how much of it no correction can
        // remove.
        let mean_deg = errors.mean_deg();
        let mut lowest = 0.0f32;
        let mut highest = 0.0f32;
        for station in stations.iter().flatten() {
            let about_mean = wrap180(station.error_deg - mean_deg);
            lowest = lowest.min(about_mean);
            highest = highest.max(about_mean);
        }
        let spread_deg = highest - lowest;
        // The headings already have the current correction in them, so the
        // new one is the old one less the error that is left.
        let correction_deg = wrap180(HEADING_CORRECTION_DEG - mean_deg);
        let constant = spread_deg <= TRIM_LIMIT_DEG;

        rprintln!("---- heading calibration, {} stations", errors.count);
        rprintln!("station  known  reported  error   elev   roll  steady  compass  hub +/-");
        for (index, station) in stations.iter().flatten().enumerate() {
            let reading = station.reading;
            rprintln!(
                "   {}    {:6.1}   {:6.1}  {:+6.1} {:+6.1} {:+6.1}  {:5.1}   {:>4}   {:5.1}",
                index + 1,
                station.reference_deg,
                reading.heading_deg,
                station.error_deg,
                reading.elevation_deg,
                reading.roll_deg,
                reading.steady_deg,
                short(reading.calibration),
                reading.accuracy_deg
            );
        }
        rprintln!(
            "average error {:+.1} deg, spread {:.1} deg peak to peak (limit {:.1})",
            mean_deg,
            spread_deg,
            TRIM_LIMIT_DEG
        );

        if constant {
            rprintln!(
                "The error is the same whichever way the device faces, so one number removes it:"
            );
            rprintln!(
                "    pub const HEADING_CORRECTION_DEG: f32 = {:.1};   // was {:.1}",
                correction_deg,
                HEADING_CORRECTION_DEG
            );
            rprintln!(
                "Put that in src/constants.rs, flash the application and run this again to check: \
                 the errors should then average to zero."
            );
        } else {
            rprintln!(
                "The error changes by {:.1} deg between stations, so no single number removes it. \
                 Something near the magnetometer is bending the field, or the compass is not \
                 calibrated. Run compass_diag and see docs/heading-calibration.md; measuring the \
                 average anyway would only spread the error around the circle.",
                spread_deg
            );
        }
        if worst != Accuracy::High {
            rprintln!(
                "The hub's confidence in its compass fell to {:?} during the run, so this result \
                 is provisional: move the device in a slow figure of eight until it reads high, \
                 then measure again.",
                worst
            );
        }
        if off_level {
            rprintln!(
                "At least one station was more than {:.0} deg off level. The field dips steeply \
                 here, so a tilt reads partly as a turn; hold the device level, or level it with \
                 the mount it will be used on.",
                LEVEL_LIMIT_DEG
            );
        }

        // The display's lines are built first: a format_args! lives only to
        // the end of the statement that makes it, so one cannot come out of
        // an `if`.
        let headline = if constant {
            format!("corr {correction_deg:+.1}")
        } else {
            String::from("not a constant")
        };
        let compass = if worst == Accuracy::High {
            String::from("cal high")
        } else {
            format!("cal {} - redo", short(worst))
        };
        show(
            lcd,
            &[
                format_args!("{headline}"),
                format_args!("spread {spread_deg:.1} pp"),
                format_args!("{compass}"),
                format_args!("press=again"),
            ],
        )
        .await;
    }

    /// Runs the routine: a round of stations, a report, and another round
    /// whenever the button is pressed.
    #[embassy_executor::task]
    pub async fn cal_task(mut imu: Imu, mut lcd: Display, mut button: Input<'static>) {
        show(
            &mut lcd,
            &[format_args!("heading cal"), format_args!("starting")],
        )
        .await;
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

        // The same configuration the application uses, so the headings
        // measured here are the headings it will report: the hub calibrates
        // the accelerometer and magnetometer, and the rotation vector is the
        // fused one.
        let started = match imu.configure_calibration(true, false, true).await {
            Ok(()) => {
                imu.enable_report(SensorId::RotationVector, IMU_REPORT_INTERVAL_US)
                    .await
            }
            Err(error) => Err(error),
        };
        if let Err(error) = started {
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

        rprintln!(
            "Heading calibration: {} stations, {:.0} ms each, current correction {:.1} deg.",
            STATIONS.len(),
            SAMPLE_WINDOW.as_millis(),
            HEADING_CORRECTION_DEG
        );
        rprintln!("Hold the device level, aim it along each bearing in turn and press the button.");

        let mut saved = false;
        loop {
            let mut stations = [None; STATIONS.len()];
            for (index, &reference_deg) in STATIONS.iter().enumerate() {
                // A station that was not held still, or that the hub said
                // nothing during, is offered again rather than silently
                // spoiling the average.
                loop {
                    aim_at(
                        &mut imu,
                        &mut lcd,
                        &mut button,
                        &mut saved,
                        index,
                        reference_deg,
                    )
                    .await;
                    show(
                        &mut lcd,
                        &[
                            format_args!("stn {}/{}", index + 1, STATIONS.len()),
                            format_args!("aim {:.1}", reference_deg),
                            format_args!("measuring..."),
                        ],
                    )
                    .await;
                    let reading = measure(&mut imu, &mut saved).await;
                    wait_for_release(&mut imu, &mut button, &mut saved).await;

                    let Some(reading) = reading else {
                        show(
                            &mut lcd,
                            &[
                                format_args!("no IMU reports"),
                                format_args!("check the hub"),
                                format_args!("press=again"),
                            ],
                        )
                        .await;
                        continue;
                    };
                    if reading.steady_deg > STEADY_LIMIT_DEG {
                        show(
                            &mut lcd,
                            &[
                                format_args!("moved {:.1} deg", reading.steady_deg),
                                format_args!("hold it still"),
                                format_args!("press=again"),
                            ],
                        )
                        .await;
                        continue;
                    }

                    let error_deg = wrap180(reading.heading_deg - reference_deg);
                    stations[index] = Some(Station {
                        reference_deg,
                        reading,
                        error_deg,
                    });
                    show(
                        &mut lcd,
                        &[
                            format_args!("stn {}/{} done", index + 1, STATIONS.len()),
                            format_args!("got {:.1}", reading.heading_deg),
                            format_args!("err {:+.1}", error_deg),
                            format_args!("cal {}", short(reading.calibration)),
                        ],
                    )
                    .await;
                    Timer::after(RESULT_HOLD).await;
                    break;
                }
            }

            report(&stations, &mut lcd).await;

            // A press starts a fresh round, so a correction can be checked
            // without reflashing.
            while button.is_high() {
                let _ = with_timeout(BUTTON_POLL, button.wait_for_low()).await;
                let _ = drain(&mut imu, &mut saved).await;
            }
            wait_for_release(&mut imu, &mut button, &mut saved).await;
        }
    }
}
