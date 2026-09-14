//! Async UART driver for the Benewake TF03 long-range LiDAR (TF03-100 and
//! TF03-180, which differ only in maximum range).
//!
//! The TF03 measures distance by pulsed time of flight and, unless told
//! otherwise, streams a nine byte data frame per measurement at 100 Hz
//! without being asked. Configuration commands share the same line under a
//! separate framing; see [`protocol`].
//!
//! # Interface
//!
//! The standard TF03 talks UART or CAN, one at a time, and leaves the
//! factory on UART. It has no I²C. This driver uses UART: the TF03's UART is
//! 3.3 V LVTTL, so it wires straight to ESP32-S3 GPIOs, whereas CAN needs an
//! external transceiver and buys noise immunity over long cable runs. The
//! driver works over any [`embedded_io_async`] stream; on esp-hal that is a
//! `Uart<'_, Async>` at the TF03's default of 115200 8N1.
//!
//! [`Tf03::read`] needs only the TF03's output line, so a `UartRx` on its own
//! is enough for a receive-only hookup. Everything else sends a command and
//! also needs `Write`.
//!
//! # Keeping up
//!
//! A data frame is 90 bits on the wire, so at 115200 baud the line tops out
//! near 1.2 kHz of frames. The ESP32-S3's UART FIFO holds 128 bytes, about
//! fourteen frames; a task that stops reading for longer than that loses
//! bytes, and the next read reports the overflow. Reading can carry on
//! afterwards, since the parser resynchronises on the next sound frame.
//!
//! # Example
//!
//! ```ignore
//! let mut lidar = Tf03::new(uart);
//! let version = lidar.firmware_version().await?;
//!
//! loop {
//!     let measurement = lidar.read().await?;
//!     if let Some(distance_cm) = measurement.distance_cm {
//!         // ...
//!     }
//! }
//! ```

pub mod protocol;

use embassy_time::{Duration, with_timeout};
use embedded_io_async::{Read, Write};

use protocol::{Command, DataFrame, Packet, Parser, Response};

// Command IDs.
const COMMAND_FIRMWARE_VERSION: u8 = 0x01;
const COMMAND_SYSTEM_RESET: u8 = 0x02;
const COMMAND_FRAME_RATE: u8 = 0x03;
const COMMAND_TRIGGER: u8 = 0x04;
const COMMAND_BAUD_RATE: u8 = 0x06;
const COMMAND_OUTPUT_ENABLE: u8 = 0x07;
const COMMAND_RESTORE_DEFAULTS: u8 = 0x10;
const COMMAND_SAVE_SETTINGS: u8 = 0x11;
const COMMAND_OUT_OF_RANGE: u8 = 0x4F;

/// The distance the TF03 reports when nothing is in range, until configured
/// otherwise with [`Tf03::set_out_of_range_threshold`].
pub const DEFAULT_OUT_OF_RANGE_CM: u16 = 18_000;

/// Below this signal strength the TF03 does not trust its return, and sends
/// the out-of-range distance in place of a measurement.
pub const MIN_RELIABLE_STRENGTH: u16 = 40;

/// UART baud rates the TF03 accepts.
///
/// The two editions of the manual disagree at the fast end (V1.2.2 adds
/// 500000 and 600000, the 2024 edition 1500000 and 2000000), so only the
/// rates both list are allowed. An unsupported rate is not refused by the
/// TF03; it silently falls back to 115200 and the link is lost.
pub const SUPPORTED_BAUD_RATES: [u32; 15] = [
    9_600, 14_400, 19_200, 38_400, 56_000, 57_600, 115_200, 128_000, 230_400, 256_000, 460_800,
    512_000, 750_000, 921_600, 1_000_000,
];

/// How long the TF03 may take to answer a command. The manual gives a
/// command no response within one second as failed.
const RESPONSE_TIMEOUT: Duration = Duration::from_secs(1);

/// Largest chunk taken from the UART in one read.
const RX_CHUNK: usize = 64;

/// What the driver can fail at.
#[derive(Debug)]
pub enum Error<E> {
    /// The underlying UART returned an error. On esp-hal this includes RX
    /// FIFO overflows, after which reading can simply continue.
    Uart(E),
    /// The UART stream reported end of file.
    EndOfStream,
    /// The TF03 did not answer a command in time.
    Timeout,
    /// The TF03 answered a command with this nonzero status code.
    Rejected(u8),
    /// The TF03 answered a command with something other than the manual
    /// describes.
    Protocol,
    /// The TF03 does not support the value asked for.
    Unsupported,
}

/// One distance reading.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Measurement {
    /// Distance to the target in centimetres, or `None` when nothing was in
    /// range or the return was too weak to trust.
    pub distance_cm: Option<u16>,
    /// The distance field as sent, which reads as the out-of-range threshold
    /// when there was no usable return.
    pub raw_distance_cm: u16,
    /// Strength of the return, from 0 to 3500. Readings between 40 and 1200
    /// are the most reliable; highly reflective targets read above 1500.
    pub strength: u16,
}

/// The firmware version, which the TF03 reports as `major.minor.patch`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct FirmwareVersion {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
}

/// A TF03 on a UART.
pub struct Tf03<U> {
    uart: U,
    parser: Parser,
    /// The last chunk read from the UART. Kept here rather than on the stack
    /// so it does not inflate the future of every call that reads.
    rx_buf: [u8; RX_CHUNK],
    /// How far through `rx_buf` the parser has been fed.
    rx_start: usize,
    /// How much of `rx_buf` holds received bytes.
    rx_end: usize,
    /// The distance that means nothing was measured.
    out_of_range_cm: u16,
}

impl<U> Tf03<U> {
    /// Wraps a UART already configured to match the TF03, 115200 8N1 from
    /// the factory. Nothing is sent.
    pub const fn new(uart: U) -> Self {
        Self {
            uart,
            parser: Parser::new(),
            rx_buf: [0; RX_CHUNK],
            rx_start: 0,
            rx_end: 0,
            out_of_range_cm: DEFAULT_OUT_OF_RANGE_CM,
        }
    }

    /// Tells the driver about an out-of-range threshold configured on the
    /// TF03 earlier, so readings at that distance are reported as `None`.
    /// The TF03 keeps the threshold in flash and has no command to read it
    /// back.
    pub const fn with_out_of_range_threshold(mut self, threshold_cm: u16) -> Self {
        self.out_of_range_cm = threshold_cm;
        self
    }

    /// Hands the UART back.
    pub fn release(self) -> U {
        self.uart
    }

    fn measurement(&self, frame: DataFrame) -> Measurement {
        let usable =
            frame.distance_cm < self.out_of_range_cm && frame.strength >= MIN_RELIABLE_STRENGTH;

        Measurement {
            distance_cm: usable.then_some(frame.distance_cm),
            raw_distance_cm: frame.distance_cm,
            strength: frame.strength,
        }
    }
}

impl<U: Read> Tf03<U> {
    /// Waits for the next measurement.
    ///
    /// Any command responses that arrive in the meantime are dropped.
    pub async fn read(&mut self) -> Result<Measurement, Error<U::Error>> {
        loop {
            if let Packet::Data(frame) = self.read_packet().await? {
                return Ok(self.measurement(frame));
            }
        }
    }

    /// Waits for the next packet of either kind.
    ///
    /// Cancelling this is safe as long as the UART's own read is: received
    /// bytes are only recorded once a read has completed.
    async fn read_packet(&mut self) -> Result<Packet, Error<U::Error>> {
        loop {
            while self.rx_start < self.rx_end {
                let byte = self.rx_buf[self.rx_start];
                self.rx_start += 1;
                if let Some(packet) = self.parser.push(byte) {
                    return Ok(packet);
                }
            }

            match self.uart.read(&mut self.rx_buf).await {
                Ok(0) => return Err(Error::EndOfStream),
                Ok(count) => {
                    self.rx_start = 0;
                    self.rx_end = count;
                }
                Err(error) => {
                    // Bytes were probably lost, so whatever the parser holds
                    // would be spliced onto an unrelated later packet.
                    self.parser.reset();
                    return Err(Error::Uart(error));
                }
            }
        }
    }

    /// Waits for the TF03's answer to the command `id`, skipping data frames
    /// and answers to anything else.
    async fn await_response(&mut self, id: u8) -> Result<Response, Error<U::Error>> {
        loop {
            if let Packet::Response(response) = self.read_packet().await?
                && response.id == id
            {
                return Ok(response);
            }
        }
    }
}

impl<U: Read + Write> Tf03<U> {
    /// Reads the firmware version. Also a cheap way to check the TF03 is
    /// there and the baud rate matches.
    pub async fn firmware_version(&mut self) -> Result<FirmwareVersion, Error<U::Error>> {
        let response = self
            .transact(Command::new(COMMAND_FIRMWARE_VERSION, &[]))
            .await?;

        match *response.payload() {
            [patch, minor, major] => Ok(FirmwareVersion {
                major,
                minor,
                patch,
            }),
            _ => Err(Error::Protocol),
        }
    }

    /// Reboots the TF03. Settings in flash are kept.
    pub async fn system_reset(&mut self) -> Result<(), Error<U::Error>> {
        let response = self
            .transact(Command::new(COMMAND_SYSTEM_RESET, &[]))
            .await?;
        check_status(&response)
    }

    /// Sets how many measurements per second the TF03 streams.
    ///
    /// The TF03 supports rates of the form `n × 10^k` for `n` from 1 to 9 and
    /// `k` from 0 to 3, i.e. 1–9, 10–90, 100–900 and 1000–9000 Hz; Benewake
    /// rates it for up to 1000 Hz. Anything else is refused here, since the
    /// TF03 itself would quietly fall back to 100 Hz. Mind the baud rate: see
    /// the module documentation.
    pub async fn set_frame_rate(&mut self, hz: u16) -> Result<(), Error<U::Error>> {
        if !is_supported_frame_rate(hz) {
            return Err(Error::Unsupported);
        }

        let command = Command::new(COMMAND_FRAME_RATE, &hz.to_le_bytes());
        let response = self.transact(command).await?;
        check_echo(&command, &response)
    }

    /// Starts or stops the automatic stream of measurements. With output
    /// stopped, the TF03 measures only when [`Tf03::trigger`] asks.
    pub async fn set_output_enabled(&mut self, enabled: bool) -> Result<(), Error<U::Error>> {
        let command = Command::new(COMMAND_OUTPUT_ENABLE, &[enabled as u8]);
        let response = self.transact(command).await?;
        check_echo(&command, &response)
    }

    /// Takes a single measurement. Meant for when output has been stopped
    /// with [`Tf03::set_output_enabled`]; otherwise it returns whichever
    /// streamed measurement arrives next.
    pub async fn trigger(&mut self) -> Result<Measurement, Error<U::Error>> {
        self.send(Command::new(COMMAND_TRIGGER, &[])).await?;

        with_timeout(RESPONSE_TIMEOUT, self.read())
            .await
            .map_err(|_| Error::Timeout)?
    }

    /// Changes the TF03's UART baud rate.
    ///
    /// The host UART has to be reconfigured to match afterwards, and the
    /// manual does not say which of the two rates the TF03 answers this
    /// command at. A [`Error::Timeout`] here therefore does not mean the
    /// change failed: switch the host UART over and confirm with
    /// [`Tf03::firmware_version`].
    pub async fn set_baud_rate(&mut self, baud: u32) -> Result<(), Error<U::Error>> {
        if !SUPPORTED_BAUD_RATES.contains(&baud) {
            return Err(Error::Unsupported);
        }

        let command = Command::new(COMMAND_BAUD_RATE, &baud.to_le_bytes());
        let response = self.transact(command).await?;
        check_echo(&command, &response)
    }

    /// Sets the distance the TF03 reports when nothing is in range, and
    /// which the driver will then report as `None`.
    pub async fn set_out_of_range_threshold(
        &mut self,
        threshold_cm: u16,
    ) -> Result<(), Error<U::Error>> {
        let response = self
            .transact(Command::new(
                COMMAND_OUT_OF_RANGE,
                &threshold_cm.to_le_bytes(),
            ))
            .await?;
        check_status(&response)?;

        self.out_of_range_cm = threshold_cm;
        Ok(())
    }

    /// Writes the current settings to flash.
    ///
    /// The manual says settings are stored once acknowledged, but also lists
    /// this command; calling it after configuring costs nothing.
    pub async fn save_settings(&mut self) -> Result<(), Error<U::Error>> {
        let response = self
            .transact(Command::new(COMMAND_SAVE_SETTINGS, &[]))
            .await?;
        check_status(&response)
    }

    /// Restores factory settings, including a baud rate of 115200 and a
    /// frame rate of 100 Hz.
    pub async fn restore_defaults(&mut self) -> Result<(), Error<U::Error>> {
        let response = self
            .transact(Command::new(COMMAND_RESTORE_DEFAULTS, &[]))
            .await?;
        check_status(&response)?;

        self.out_of_range_cm = DEFAULT_OUT_OF_RANGE_CM;
        Ok(())
    }

    /// Sends a command and waits for the TF03's answer to it.
    async fn transact(&mut self, command: Command) -> Result<Response, Error<U::Error>> {
        self.send(command).await?;

        with_timeout(RESPONSE_TIMEOUT, self.await_response(command.id()))
            .await
            .map_err(|_| Error::Timeout)?
    }

    async fn send(&mut self, command: Command) -> Result<(), Error<U::Error>> {
        self.uart
            .write_all(command.as_bytes())
            .await
            .map_err(Error::Uart)?;
        self.uart.flush().await.map_err(Error::Uart)
    }
}

/// Whether `hz` is of the form `n × 10^k` with `n` in 1..=9 and `k` in 0..=3.
fn is_supported_frame_rate(hz: u16) -> bool {
    [1, 10, 100, 1000]
        .into_iter()
        .any(|scale| hz.is_multiple_of(scale) && (1..=9).contains(&(hz / scale)))
}

/// For commands the TF03 acknowledges with a single status byte.
fn check_status<E>(response: &Response) -> Result<(), Error<E>> {
    match *response.payload() {
        [0] => Ok(()),
        [code] => Err(Error::Rejected(code)),
        _ => Err(Error::Protocol),
    }
}

/// For commands the TF03 acknowledges by sending the command back.
fn check_echo<E>(command: &Command, response: &Response) -> Result<(), Error<E>> {
    if response.payload() == command.payload() {
        Ok(())
    } else {
        Err(Error::Protocol)
    }
}
