//! Standalone demo and wiring check for the SSD1306 OLED display.
//!
//! Works through the ways the display can fail to show anything, from the
//! wires up, and reports each over RTT:
//!
//! 1. With the I2C lines still plain inputs, it checks that both idle high.
//!    The module's pull-up resistors hold them there, so a line that reads
//!    low means the module is unpowered, unplugged from that pin, or has no
//!    pull-ups.
//! 2. Once a second it scans the bus and lists every address that answers,
//!    so wires can be wiggled or reseated while it runs.
//! 3. When the display answers at `LCD_ADDRESS`, it fills every row with
//!    text, updates a counter and flips the screen between normal and
//!    inverted each second. Text shifted sideways with a stripe of noise at
//!    one edge means the module is an SH1106: set `LCD_COLUMN_OFFSET` to 2.

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
use esp_hal::gpio::{Input, InputConfig, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use ping_capstone::constants::{HEAP_SIZE, LCD_ADDRESS, LCD_COLUMN_OFFSET, LCD_I2C_FREQUENCY_KHZ};
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
async fn main(_spawner: Spawner) -> ! {
    rtt_target::rtt_init_print!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let mut peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: HEAP_SIZE);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    rprintln!("Embassy initialized!");

    // Step 1: idle levels. The internal pull-downs (about 45 kΩ) lose to the
    // module's pull-ups (a few kΩ), so a line reads high only if a powered
    // module is on it. Pins come from ping_capstone::constants.
    {
        let pulled_down = InputConfig::default().with_pull(Pull::Down);
        let sda = Input::new(board_pin!(peripherals.LCD_SDA).reborrow(), pulled_down);
        let scl = Input::new(board_pin!(peripherals.LCD_SCL).reborrow(), pulled_down);
        Timer::after(Duration::from_millis(10)).await;
        for (name, line) in [("SDA (GPIO1)", &sda), ("SCL (GPIO2)", &scl)] {
            if line.is_high() {
                rprintln!("{} idles high: pulled up, good", name);
            } else {
                rprintln!(
                    "{} idles LOW: check the display's VCC and GND, this wire, and that \
                     the module has pull-up resistors",
                    name
                );
            }
        }
    }

    let i2c = I2c::new(
        board_bus!(peripherals.LCD_I2C),
        I2cConfig::default().with_frequency(Rate::from_khz(LCD_I2C_FREQUENCY_KHZ)),
    )
    .expect("Failed to configure the display I2C bus")
    .with_sda(board_pin!(peripherals.LCD_SDA))
    .with_scl(board_pin!(peripherals.LCD_SCL))
    .into_async();
    let mut lcd = Ssd1306::new(i2c, LCD_ADDRESS, LCD_COLUMN_OFFSET);

    // Steps 2 and 3, repeated so changes to the wiring show up live.
    let mut ready = false;
    let mut count: u32 = 0;
    loop {
        let found = lcd.scan().await;
        if !found.iter().any(|address| address == LCD_ADDRESS) {
            ready = false;
            match found.iter().next() {
                None => rprintln!(
                    "I2C scan: nothing answers. Check VCC to 3V3, GND, SDA to GPIO1 (J3-4) \
                     and SCL to GPIO2 (J3-5); swapped SDA and SCL is the usual mistake"
                ),
                Some(_) => rprintln!(
                    "I2C scan: nothing at {:#04x}, but found {}. Set LCD_ADDRESS in \
                     constants.rs to the display's address (0x3C or 0x3D for an SSD1306)",
                    LCD_ADDRESS,
                    found
                ),
            }
        } else {
            if !ready {
                rprintln!("I2C scan: display answers at {:#04x}", LCD_ADDRESS);
                match lcd.init().await {
                    Ok(()) => ready = true,
                    Err(error) => rprintln!("Display init failed: {:?}", error),
                }
            }

            if ready {
                count = count.wrapping_add(1);
                let inverted = count.is_multiple_of(2);
                let written = async {
                    lcd.set_inverted(inverted).await?;
                    lcd.write_line(0, "PING display").await?;
                    lcd.print_line(1, format_args!("count {}", count)).await?;
                    lcd.write_line(2, "0123456789ABCDEF").await?;
                    lcd.write_line(3, "12.5° ±0.1 m").await
                }
                .await;
                match written {
                    Ok(()) => rprintln!(
                        "wrote count {}, {}",
                        count,
                        if inverted { "inverted" } else { "normal" }
                    ),
                    Err(error) => {
                        rprintln!("Display write failed: {:?}", error);
                        ready = false;
                    }
                }
            }
        }

        Timer::after(Duration::from_secs(1)).await;
    }
}
