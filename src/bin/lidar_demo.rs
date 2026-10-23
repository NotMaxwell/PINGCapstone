//! Standalone demo binary for the TF03 LiDAR driver.
//!
//! Listens to the TF03 over UART and shows the measured distance and signal
//! strength on the display and over RTT, five times a second. It also asks
//! once for the TF03's firmware version, but reads distances whether or not
//! that is answered. Kept separate from `main.rs` so the
//! application entry point stays clear of sensor bring-up code.
//!
//! Nothing here changes the TF03's configuration, which it keeps in flash.

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use ping_capstone::constants::{
    HEAP_SIZE, LCD_ADDRESS, LCD_COLUMN_OFFSET, LCD_I2C_FREQUENCY_KHZ, LIDAR_BAUD_RATE,
};
use ping_capstone::ssd1306::Ssd1306;
use ping_capstone::tf03::Tf03;
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

    // Pins come from ping_capstone::constants. The TF03's UART is 3.3 V, so
    // it connects directly: board TX to the blue UART RxD wire, board RX to
    // the brown UART TxD wire, and the grounds together. The red supply wire
    // takes 5-24 V; on PING it runs from the fused 12 V line.
    let uart = Uart::new(
        board_bus!(peripherals.LIDAR_UART),
        UartConfig::default().with_baudrate(LIDAR_BAUD_RATE),
    )
    .expect("Failed to configure UART")
    .with_tx(board_pin!(peripherals.LIDAR_TX))
    .with_rx(board_pin!(peripherals.LIDAR_RX))
    .into_async();

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

    spawner.spawn(lidar_task(Tf03::new(uart), lcd).expect("Failed to spawn the LiDAR task"));

    // Everything happens in the task.
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

use lidar::lidar_task;

mod lidar {
    //! The LiDAR task lives in its own module so that the stack frame lint
    //! can be relaxed for it. An embassy task's future is allocated once in a
    //! static task pool rather than on the stack, and most of its size here
    //! is the HAL's UART and I2C futures and the formatting machinery behind
    //! rprintln!, not anything this code puts on the stack.
    #![allow(clippy::large_stack_frames)]

    use core::fmt;
    use core::sync::atomic::{AtomicBool, Ordering};

    use embassy_time::{Duration, Instant, with_timeout};
    use esp_hal::Async;
    use esp_hal::i2c::master::I2c;
    use esp_hal::uart::Uart;
    use ping_capstone::constants::LCD_REFRESH;
    use ping_capstone::ssd1306::{ROWS, Ssd1306};
    use ping_capstone::tf03::Tf03;
    use rtt_target::{rprint, rprintln};

    type Display = Ssd1306<I2c<'static, Async>>;

    /// Whether the last display write failed, so a display that stays
    /// unplugged is reported once rather than on every refresh.
    static DISPLAY_FAILING: AtomicBool = AtomicBool::new(false);

    /// Shows `lines` on the display, one per row, blanking the rows below
    /// them, and prints them over RTT on one line. A failed display write is
    /// logged and otherwise ignored, so a loose display never stops the
    /// LiDAR being read.
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
                if !DISPLAY_FAILING.swap(true, Ordering::Relaxed) {
                    rprintln!("Display write failed: {:?}", error);
                }
                return;
            }
        }
        if DISPLAY_FAILING.swap(false, Ordering::Relaxed) {
            rprintln!("Display back");
        }
    }

    /// How long the TF03 may go without a measurement before the demo says
    /// so. It sends 100 a second, so a whole second without one means none
    /// are arriving.
    const SILENCE: Duration = Duration::from_secs(1);

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

    /// Reads distance from the TF03 and shows it. The TF03 streams 100
    /// measurements a second; most go unshown.
    #[embassy_executor::task]
    pub async fn lidar_task(mut lidar: Tf03<Uart<'static, Async>>, mut lcd: Display) {
        show(&mut lcd, &[format_args!("TF03"), format_args!("starting")]).await;

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

        let mut next_refresh = Instant::now();
        loop {
            let reading = with_timeout(SILENCE, lidar.read()).await;
            if reading.is_ok() && !due(&mut next_refresh) {
                continue;
            }

            match reading {
                Err(_) => {
                    show(
                        &mut lcd,
                        &[
                            format_args!("TF03 no data"),
                            format_args!("check power,"),
                            format_args!("brown wire, GND"),
                        ],
                    )
                    .await
                }
                Ok(Ok(measurement)) => match measurement.distance_cm {
                    Some(distance_cm) => {
                        show(
                            &mut lcd,
                            &[
                                format_args!("dist {:>9.2} m", f32::from(distance_cm) / 100.0),
                                format_args!("strength {:>7}", measurement.strength),
                            ],
                        )
                        .await
                    }
                    None => {
                        show(
                            &mut lcd,
                            &[
                                format_args!("out of range"),
                                format_args!("raw {} cm", measurement.raw_distance_cm),
                                format_args!("strength {:>7}", measurement.strength),
                            ],
                        )
                        .await
                    }
                },
                // Typically an RX FIFO overflow; the driver resynchronises
                // on the next frame, so carry on reading.
                Ok(Err(error)) => {
                    show(
                        &mut lcd,
                        &[
                            format_args!("TF03 read failed"),
                            format_args!("{:?}", error),
                        ],
                    )
                    .await
                }
            }
        }
    }
}
