//! Standalone demo binary for the BNO085 driver.
//!
//! Brings up the sensor hub over SPI, starts the rotation vector at 100 Hz and
//! prints each quaternion over RTT. Kept separate from `main.rs` so the
//! application entry point stays clear of sensor bring-up code.

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

    spawner.spawn(imu_task(Bno085::new(transport, Delay)).expect("Failed to spawn the IMU task"));

    loop {
        rprintln!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }
}

use imu::imu_task;

mod imu {
    //! The IMU task lives in its own module so that the stack frame lint can
    //! be relaxed for it. An embassy task's future is allocated once in a
    //! static task pool rather than on the stack, and most of its size here
    //! is the HAL's SPI futures and the formatting machinery behind
    //! rprintln!, not anything this code puts on the stack.
    #![allow(clippy::large_stack_frames)]

    use embassy_time::{Delay, Timer};
    use esp_hal::Async;
    use esp_hal::gpio::{Input, Output};
    use esp_hal::spi::master::Spi;
    use ping_capstone::bno085::{Bno085, SensorData, SensorId, SpiTransport};
    use ping_capstone::constants::{IMU_REPORT_INTERVAL_US, RETRY_DELAY};
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

    /// Reads orientation from the BNO085.
    ///
    /// The sensor pulls INT low when it has something to say, and holds it low
    /// until everything waiting has been read, so each wake-up drains the hub
    /// before going back to sleep.
    #[embassy_executor::task]
    pub async fn imu_task(mut imu: Imu) {
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
                    rprintln!("BNO085 init failed: {:?}, retrying", error);
                    Timer::after(RETRY_DELAY).await;
                }
            }
        }

        if let Err(error) = imu
            .enable_report(SensorId::RotationVector, IMU_REPORT_INTERVAL_US)
            .await
        {
            rprintln!("BNO085 could not start reporting: {:?}", error);
            return;
        }

        loop {
            if let Err(error) = imu.wait_for_data().await {
                rprintln!("BNO085 wait failed: {:?}", error);
                Timer::after(RETRY_DELAY).await;
                continue;
            }

            loop {
                match imu.read_event().await {
                    Ok(Some(event)) => {
                        if let SensorData::RotationVector {
                            quaternion,
                            accuracy_rad,
                        } = event.data
                        {
                            rprintln!(
                                "quat i={} j={} k={} r={} (+/- {} rad, {:?})",
                                quaternion.i,
                                quaternion.j,
                                quaternion.k,
                                quaternion.real,
                                accuracy_rad,
                                event.accuracy
                            );
                        }
                    }
                    // Nothing left waiting; go back to the interrupt pin.
                    Ok(None) => break,
                    Err(error) => {
                        rprintln!("BNO085 read failed: {:?}", error);
                        break;
                    }
                }
            }
        }
    }
}
