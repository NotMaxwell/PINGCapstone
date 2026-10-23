//! Standalone demo binary for the NEO-M9N GNSS driver.
//!
//! Brings up the receiver over UART, reads its version, switches it to UBX
//! output at 10 Hz and shows the position on the display and over RTT, five
//! times a second: latitude, longitude, horizontal accuracy in metres and
//! satellites in use. Kept separate from `main.rs` so the application entry
//! point stays clear of sensor bring-up code.
//!
//! The configuration is written to the receiver's RAM only, so a power cycle
//! puts it back to factory settings.

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
    GPS_BAUD_RATE, HEAP_SIZE, LCD_ADDRESS, LCD_COLUMN_OFFSET, LCD_I2C_FREQUENCY_KHZ,
};
use ping_capstone::neo_m9n::NeoM9n;
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

    // Pins come from ping_capstone::constants. The NEO-M9N runs at 3.3 V, so
    // it connects directly: board TX to the GPS's RXD, board RX to its TXD,
    // and the grounds together. UART2 leaves UART1 free for the TF03.
    let uart = Uart::new(
        board_bus!(peripherals.GPS_UART),
        UartConfig::default().with_baudrate(GPS_BAUD_RATE),
    )
    .expect("Failed to configure UART")
    .with_tx(board_pin!(peripherals.GPS_TX))
    .with_rx(board_pin!(peripherals.GPS_RX))
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

    spawner.spawn(gps_task(NeoM9n::new(uart), lcd).expect("Failed to spawn the GPS task"));

    // Everything happens in the task.
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}

use gps::gps_task;

mod gps {
    //! The GPS task lives in its own module so that the stack frame lint can
    //! be relaxed for it. An embassy task's future is allocated once in a
    //! static task pool rather than on the stack, and most of its size here
    //! is the driver's parser buffer, the HAL's UART and I2C futures and the
    //! formatting machinery behind rprintln!, not anything this code puts on
    //! the stack.
    #![allow(clippy::large_stack_frames)]

    use core::fmt;

    use embassy_time::{Instant, Timer};
    use esp_hal::Async;
    use esp_hal::i2c::master::I2c;
    use esp_hal::uart::Uart;
    use ping_capstone::constants::{GPS_RATE_HZ, LCD_REFRESH, RETRY_DELAY};
    use ping_capstone::neo_m9n::NeoM9n;
    use ping_capstone::ssd1306::{ROWS, Ssd1306};
    use rtt_target::{rprint, rprintln};

    type Display = Ssd1306<I2c<'static, Async>>;

    /// Shows `lines` on the display, one per row, blanking the rows below
    /// them, and prints them over RTT on one line. A failed display write is
    /// logged and otherwise ignored, so a loose display never stops the
    /// GPS being read.
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

    /// Reads position from the NEO-M9N and shows it.
    #[embassy_executor::task]
    pub async fn gps_task(mut gps: NeoM9n<Uart<'static, Async>>, mut lcd: Display) {
        show(&mut lcd, &[format_args!("GPS"), format_args!("starting")]).await;
        loop {
            match gps.version().await {
                Ok(version) => {
                    show(
                        &mut lcd,
                        &[
                            format_args!("GPS up"),
                            format_args!(
                                "{}",
                                version
                                    .module
                                    .as_ref()
                                    .map_or("unknown module", |m| m.as_str())
                            ),
                            format_args!("hw {}", version.hardware),
                            format_args!("protocol {:?}", version.protocol),
                        ],
                    )
                    .await;
                    break;
                }
                Err(error) => {
                    show(
                        &mut lcd,
                        &[
                            format_args!("GPS no reply"),
                            format_args!("{:?}", error),
                            format_args!("retrying"),
                        ],
                    )
                    .await;
                    Timer::after(RETRY_DELAY).await;
                }
            }
        }

        while let Err(error) = gps.configure(GPS_RATE_HZ).await {
            show(
                &mut lcd,
                &[
                    format_args!("GPS refused"),
                    format_args!("config"),
                    format_args!("{:?}", error),
                    format_args!("retrying"),
                ],
            )
            .await;
            Timer::after(RETRY_DELAY).await;
        }

        let mut next_refresh = Instant::now();
        loop {
            let reading = gps.read().await;
            if !due(&mut next_refresh) {
                continue;
            }

            match reading {
                Ok(solution) => match solution.position() {
                    Some((latitude, longitude)) => {
                        show(
                            &mut lcd,
                            &[
                                format_args!("lat {:+12.7}", latitude),
                                format_args!("lon {:+12.7}", longitude),
                                format_args!("acc ±{:.1} m", solution.horizontal_accuracy_m),
                                format_args!("{} satellites", solution.satellites),
                            ],
                        )
                        .await
                    }
                    None => {
                        show(
                            &mut lcd,
                            &[
                                format_args!("no fix"),
                                format_args!("{:?}", solution.fix_type),
                                format_args!("{} satellites", solution.satellites),
                            ],
                        )
                        .await
                    }
                },
                // Typically an RX FIFO overflow; the parser resynchronises on
                // the next frame, so carry on reading.
                Err(error) => {
                    show(
                        &mut lcd,
                        &[format_args!("GPS read failed"), format_args!("{:?}", error)],
                    )
                    .await
                }
            }
        }
    }
}
