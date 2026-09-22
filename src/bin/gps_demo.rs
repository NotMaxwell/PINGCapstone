//! Standalone demo binary for the NEO-M9N GNSS driver.
//!
//! Brings up the receiver over UART, reads its version, switches it to UBX
//! output at 10 Hz and prints each solution over RTT. Kept separate from
//! `main.rs` so the application entry point stays clear of sensor bring-up
//! code.
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
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use ping_capstone::neo_m9n::{DEFAULT_BAUD_RATE, NeoM9n};
use rtt_target::rprintln;

/// Ask for a navigation solution ten times a second.
const NAV_RATE_HZ: u8 = 10;

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

    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    // Adjust these pins to match how the NEO-M9N is wired. The SparkFun board
    // is 3.3 V throughout, so it connects directly: GPIO15 (TX) to the
    // board's RX1 pin, GPIO16 (RX) to its TX1 pin, and the grounds together.
    // UART2 leaves UART1 free for the TF03.
    let uart = Uart::new(
        peripherals.UART2,
        UartConfig::default().with_baudrate(DEFAULT_BAUD_RATE),
    )
    .expect("Failed to configure UART")
    .with_tx(peripherals.GPIO15)
    .with_rx(peripherals.GPIO16)
    .into_async();

    spawner.spawn(gps_task(NeoM9n::new(uart)).expect("Failed to spawn the GPS task"));

    loop {
        rprintln!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }
}

use gps::gps_task;

mod gps {
    //! The GPS task lives in its own module so that the stack frame lint can
    //! be relaxed for it. An embassy task's future is allocated once in a
    //! static task pool rather than on the stack, and most of its size here
    //! is the driver's parser buffer, the HAL's UART futures and the
    //! formatting machinery behind rprintln!, not anything this code puts on
    //! the stack.
    #![allow(clippy::large_stack_frames)]

    use super::NAV_RATE_HZ;
    use embassy_time::{Duration, Timer};
    use esp_hal::Async;
    use esp_hal::uart::Uart;
    use ping_capstone::neo_m9n::NeoM9n;
    use rtt_target::rprintln;

    /// Reads position from the NEO-M9N.
    #[embassy_executor::task]
    pub async fn gps_task(mut gps: NeoM9n<Uart<'static, Async>>) {
        loop {
            match gps.version().await {
                Ok(version) => {
                    rprintln!(
                        "GPS up: {} (hardware {}, protocol {:?})",
                        version
                            .module
                            .as_ref()
                            .map_or("unknown module", |m| m.as_str()),
                        version.hardware,
                        version.protocol,
                    );
                    break;
                }
                Err(error) => {
                    rprintln!("GPS not answering: {:?}, retrying", error);
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
        }

        while let Err(error) = gps.configure(NAV_RATE_HZ).await {
            rprintln!("GPS refused configuration: {:?}, retrying", error);
            Timer::after(Duration::from_secs(1)).await;
        }

        loop {
            match gps.read().await {
                Ok(solution) => match solution.position() {
                    Some((latitude, longitude)) => rprintln!(
                        "{:.7}, {:.7} ±{:.1} m, {} satellites",
                        latitude,
                        longitude,
                        solution.horizontal_accuracy_m,
                        solution.satellites
                    ),
                    None => rprintln!(
                        "no fix ({:?}, {} satellites)",
                        solution.fix_type,
                        solution.satellites
                    ),
                },
                // Typically an RX FIFO overflow; the parser resynchronises on
                // the next frame, so carry on reading.
                Err(error) => rprintln!("GPS read failed: {:?}", error),
            }
        }
    }
}
