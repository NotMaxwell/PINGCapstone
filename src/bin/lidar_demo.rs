//! Standalone demo binary for the TF03 LiDAR driver.
//!
//! Brings up the TF03 over UART, reads its firmware version and prints the
//! measured distance over RTT. Kept separate from `main.rs` so the
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
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart};
use ping_capstone::tf03::Tf03;
use rtt_target::rprintln;

/// The TF03 streams 100 measurements a second by default; print one in ten.
const PRINT_EVERY: u32 = 10;

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

    // Adjust these pins to match how the TF03 is wired. Its UART is 3.3 V, so
    // it connects directly: GPIO17 (TX) to the TF03's blue RxD wire, GPIO18
    // (RX) to its brown TxD wire, and the grounds together. The red supply
    // wire needs 5-24 V, not the 3.3 V rail.
    let uart = Uart::new(peripherals.UART1, UartConfig::default().with_baudrate(115_200))
        .expect("Failed to configure UART")
        .with_tx(peripherals.GPIO17)
        .with_rx(peripherals.GPIO18)
        .into_async();

    spawner.spawn(lidar_task(Tf03::new(uart)).expect("Failed to spawn the LiDAR task"));

    loop {
        rprintln!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }
}

use lidar::lidar_task;

mod lidar {
    //! The LiDAR task lives in its own module so that the stack frame lint
    //! can be relaxed for it. An embassy task's future is allocated once in a
    //! static task pool rather than on the stack, and most of its size here
    //! is the HAL's UART futures and the formatting machinery behind
    //! rprintln!, not anything this code puts on the stack.
    #![allow(clippy::large_stack_frames)]

    use super::PRINT_EVERY;
    use embassy_time::{Duration, Timer};
    use esp_hal::Async;
    use esp_hal::uart::Uart;
    use ping_capstone::tf03::Tf03;
    use rtt_target::rprintln;

    /// Reads distance from the TF03.
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

        let mut count: u32 = 0;
        loop {
            match lidar.read().await {
                Ok(measurement) => {
                    count = count.wrapping_add(1);
                    if !count.is_multiple_of(PRINT_EVERY) {
                        continue;
                    }

                    match measurement.distance_cm {
                        Some(distance_cm) => rprintln!(
                            "distance {} cm (strength {})",
                            distance_cm,
                            measurement.strength
                        ),
                        None => rprintln!(
                            "out of range (raw {} cm, strength {})",
                            measurement.raw_distance_cm,
                            measurement.strength
                        ),
                    }
                }
                // Typically an RX FIFO overflow; the driver resynchronises
                // on the next frame, so carry on reading.
                Err(error) => rprintln!("TF03 read failed: {:?}", error),
            }
        }
    }
}
