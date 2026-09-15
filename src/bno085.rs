//! Async I²C driver for the CEVA/Hillcrest BNO085 nine axis sensor hub.
//!
//! The BNO085 is not a register mapped IMU. It runs SH-2 sensor fusion
//! firmware on its own microcontroller and speaks a packet protocol: you ask
//! it to start producing a *report* at some rate, and it pushes reports at you
//! until told otherwise. This module implements that protocol on top of any
//! [`embedded_hal_async::i2c::I2c`] bus.
//!
//! # Interrupt pin
//!
//! The hub drives its `INT` line low when it has a packet waiting. Reading
//! when nothing is pending is harmless — [`Bno085::read_event`] returns
//! `Ok(None)` — but polling wastes bus bandwidth and, on some boards, reads
//! while the hub is asleep are unreliable. Wire `INT` up and await a falling
//! edge before each read.
//!
//! # Example
//!
//! ```ignore
//! let mut imu = Bno085::new(i2c, Address::Default, Delay);
//! let id = imu.init().await?;
//! imu.enable_report(SensorId::RotationVector, 10_000).await?; // 100 Hz
//!
//! loop {
//!     int_pin.wait_for_falling_edge().await;
//!     while let Some(event) = imu.read_event().await? {
//!         if let SensorData::RotationVector { quaternion, .. } = event.data {
//!             // ...
//!         }
//!     }
//! }
//! ```

pub mod reports;
pub mod shtp;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;

pub use reports::{Accuracy, Event, Quaternion, RawVec3, SensorData, SensorId, Vec3};
use shtp::{Channel, HEADER_LEN, Header};

/// Largest cargo the driver will buffer.
///
/// A packet holds a timestamp report and however many sensor reports were
/// due at once, so tens of bytes in normal use and a few hundred only under
/// aggressive batching. The reset advertisement is the one genuinely large
/// packet the hub sends, and that is discarded rather than parsed.
const MAX_CARGO: usize = 256;

/// Largest single I²C transfer the driver will attempt.
///
/// Cargos longer than this are read in several transactions, each of which
/// the hub prefixes with a fresh four byte header.
const MAX_TRANSFER: usize = 64;

/// Enough for the longest command this driver sends (set feature, 17 bytes)
/// plus its SHTP header.
const MAX_WRITE: usize = 32;

// Control channel report IDs.
const REPORT_COMMAND_REQUEST: u8 = 0xF2;
const REPORT_PRODUCT_ID_RESPONSE: u8 = 0xF8;
const REPORT_PRODUCT_ID_REQUEST: u8 = 0xF9;
const REPORT_TIMESTAMP_REBASE: u8 = 0xFA;
const REPORT_BASE_TIMESTAMP: u8 = 0xFB;
const REPORT_SET_FEATURE_COMMAND: u8 = 0xFD;

/// Both the request we send on the executable channel and the notification the
/// hub sends back once it has rebooted.
const EXECUTABLE_RESET: u8 = 0x01;

// Commands carried by a command request report.
const COMMAND_TARE: u8 = 0x03;
const COMMAND_SAVE_DCD: u8 = 0x06;
const COMMAND_ME_CALIBRATE: u8 = 0x07;

const TARE_NOW: u8 = 0x00;
const TARE_PERSIST: u8 = 0x01;

/// Tare about the X axis.
pub const TARE_AXIS_X: u8 = 1 << 0;
/// Tare about the Y axis.
pub const TARE_AXIS_Y: u8 = 1 << 1;
/// Tare about the Z axis.
pub const TARE_AXIS_Z: u8 = 1 << 2;
/// Tare about all three axes.
pub const TARE_AXIS_ALL: u8 = TARE_AXIS_X | TARE_AXIS_Y | TARE_AXIS_Z;

/// How long the hub takes to come up after a reset.
const BOOT_DELAY_MS: u32 = 100;
/// How long to wait between polls when the hub has nothing to say.
const POLL_INTERVAL_MS: u32 = 2;
/// How long to keep waiting for an expected packet before giving up.
const RESPONSE_TIMEOUT_MS: u32 = 1_000;

/// I²C address, selected on the board by the ADR/SA0 pin.
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum Address {
    /// ADR pulled low, the usual case.
    Default = 0x4A,
    /// ADR pulled high.
    Alternate = 0x4B,
}

/// What the driver can fail at.
#[derive(Debug)]
pub enum Error<E> {
    /// The underlying I²C bus returned an error.
    I2c(E),
    /// A cargo arrived that is larger than [`MAX_CARGO`]. It has been read and
    /// discarded so the bus stays in sync; nothing else is wrong.
    CargoTooLarge(u16),
    /// The hub did not send an expected packet in time.
    Timeout,
    /// A packet arrived that does not fit the protocol.
    Protocol,
}

/// Which reset the hub is reporting in its product ID response.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ResetCause {
    NotApplicable,
    PowerOn,
    InternalSystemReset,
    WatchdogTimeout,
    ExternalReset,
    Other(u8),
}

impl ResetCause {
    const fn from_u8(value: u8) -> Self {
        match value {
            0 => Self::NotApplicable,
            1 => Self::PowerOn,
            2 => Self::InternalSystemReset,
            3 => Self::WatchdogTimeout,
            4 => Self::ExternalReset,
            other => Self::Other(other),
        }
    }
}

/// The hub's identification, returned by [`Bno085::product_id`].
#[derive(Clone, Copy, Debug)]
pub struct ProductId {
    pub reset_cause: ResetCause,
    pub sw_version_major: u8,
    pub sw_version_minor: u8,
    pub sw_version_patch: u16,
    pub sw_part_number: u32,
    pub sw_build_number: u32,
}

/// Which orientation output a tare should be applied to.
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum TareBasis {
    RotationVector = 0,
    GameRotationVector = 1,
    GeomagneticRotationVector = 2,
    GyroIntegratedRotationVector = 3,
    ArvrStabilizedRotationVector = 4,
    ArvrStabilizedGameRotationVector = 5,
}

/// Everything the hub needs to know to start producing a report.
///
/// [`ReportConfig::new`] covers the common case; the remaining fields matter
/// only for change-sensitivity filtering, batching and wake behaviour.
#[derive(Clone, Copy, Debug)]
pub struct ReportConfig {
    pub sensor: SensorId,
    /// How often to report, in microseconds. Zero disables the sensor.
    pub interval_us: u32,
    /// How long the hub may batch reports before delivering them, in
    /// microseconds. Zero delivers each report as it is produced.
    pub batch_interval_us: u32,
    /// Threshold below which a change is not worth reporting, in the
    /// sensor's own units.
    pub change_sensitivity: u16,
    /// Treat `change_sensitivity` as a fraction of the last value rather than
    /// an absolute amount.
    pub change_sensitivity_relative: bool,
    /// Apply `change_sensitivity` at all.
    pub change_sensitivity_enabled: bool,
    /// Let this sensor wake the host.
    pub wake_enabled: bool,
    /// Keep this sensor running even when the hub would otherwise sleep.
    pub always_on_enabled: bool,
    /// Sensor specific configuration word; zero for all the motion sensors.
    pub sensor_specific: u32,
}

impl ReportConfig {
    /// A plain periodic report at `interval_us`, with no filtering or batching.
    pub const fn new(sensor: SensorId, interval_us: u32) -> Self {
        Self {
            sensor,
            interval_us,
            batch_interval_us: 0,
            change_sensitivity: 0,
            change_sensitivity_relative: false,
            change_sensitivity_enabled: false,
            wake_enabled: false,
            always_on_enabled: false,
            sensor_specific: 0,
        }
    }

    const fn flags(&self) -> u8 {
        let mut flags = 0;
        if self.change_sensitivity_relative {
            flags |= 1 << 0;
        }
        if self.change_sensitivity_enabled {
            flags |= 1 << 1;
        }
        if self.wake_enabled {
            flags |= 1 << 2;
        }
        if self.always_on_enabled {
            flags |= 1 << 3;
        }
        flags
    }
}

/// A BNO085 on an async I²C bus.
pub struct Bno085<I2C, D> {
    i2c: I2C,
    address: u8,
    delay: D,
    /// SHTP sequence numbers, one per channel.
    sequence: [u8; 6],
    /// Sequence number for command requests, which count separately.
    command_sequence: u8,
    /// The cargo most recently read, header included.
    buf: [u8; MAX_CARGO],
    /// Landing area for one I²C transfer. Kept here rather than on the stack
    /// so it does not inflate the future of every call that reads.
    scratch: [u8; MAX_TRANSFER],
    /// Length of that cargo, or zero when the buffer holds nothing.
    cargo_len: usize,
    /// How far through the buffered cargo [`Bno085::read_event`] has walked.
    cursor: usize,
    /// The channel the buffered cargo arrived on.
    channel: u8,
    /// Timestamp correction from the last base timestamp report, in the hub's
    /// 100 µs ticks. Added to a report's delay to place the sample in time.
    reference_delta: i32,
}

impl<I2C, D> Bno085<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    /// Wraps an I²C bus. Nothing is sent until [`Bno085::init`] is called.
    pub fn new(i2c: I2C, address: Address, delay: D) -> Self {
        Self {
            i2c,
            address: address as u8,
            delay,
            sequence: [0; 6],
            command_sequence: 0,
            buf: [0; MAX_CARGO],
            scratch: [0; MAX_TRANSFER],
            cargo_len: 0,
            cursor: 0,
            channel: 0,
            reference_delta: 0,
        }
    }

    /// Resets the hub and reads back its identification.
    ///
    /// No reports are enabled by a reset, so follow this with
    /// [`Bno085::enable_report`] for each sensor you want.
    pub async fn init(&mut self) -> Result<ProductId, Error<I2C::Error>> {
        self.soft_reset().await?;
        self.product_id().await
    }

    /// Asks the hub to reboot, then waits for it to announce that it has.
    ///
    /// The hub sends an advertisement packet on its way up. That packet is
    /// larger than the driver's buffer and describes only things the driver
    /// already knows, so it is read and discarded.
    pub async fn soft_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_cargo(Channel::Executable, &[EXECUTABLE_RESET])
            .await?;
        self.delay.delay_ms(BOOT_DELAY_MS).await;

        // A reset resets the hub's sequence numbers too.
        self.sequence = [0; 6];
        self.discard_buffered();

        let mut waited = 0;
        let mut last_error = None;
        loop {
            match self.read_cargo().await {
                Ok(Some(header)) => {
                    if header.channel == Channel::Executable as u8
                        && self.buf[HEADER_LEN] == EXECUTABLE_RESET
                    {
                        return Ok(());
                    }
                }
                Ok(None) => {
                    self.delay.delay_ms(POLL_INTERVAL_MS).await;
                    waited += POLL_INTERVAL_MS;
                }
                // The advertisement is expected to be too large. It carries
                // nothing the driver needs, and reading it kept the bus in
                // sync, so this is not a failure.
                Err(Error::CargoTooLarge(_)) => {}
                // The hub may not answer at all while it is rebooting.
                Err(error) => {
                    last_error = Some(error);
                    self.delay.delay_ms(POLL_INTERVAL_MS).await;
                    waited += POLL_INTERVAL_MS;
                }
            }

            if waited >= RESPONSE_TIMEOUT_MS {
                return Err(last_error.unwrap_or(Error::Timeout));
            }
        }
    }

    /// Asks the hub what it is and which reset it last went through.
    pub async fn product_id(&mut self) -> Result<ProductId, Error<I2C::Error>> {
        self.write_cargo(Channel::Control, &[REPORT_PRODUCT_ID_REQUEST, 0])
            .await?;

        let len = self
            .await_control_report(REPORT_PRODUCT_ID_RESPONSE, 16)
            .await?;
        let payload = &self.buf[HEADER_LEN..HEADER_LEN + len];

        Ok(ProductId {
            reset_cause: ResetCause::from_u8(payload[1]),
            sw_version_major: payload[2],
            sw_version_minor: payload[3],
            sw_part_number: u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
            sw_build_number: u32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
            sw_version_patch: u16::from_le_bytes([payload[12], payload[13]]),
        })
    }

    /// Starts a sensor reporting every `interval_us` microseconds.
    ///
    /// The hub rounds the interval to what the sensor can actually do; ask for
    /// 10_000 µs to get roughly 100 Hz. Enabling a sensor that is already
    /// enabled just changes its rate.
    pub async fn enable_report(
        &mut self,
        sensor: SensorId,
        interval_us: u32,
    ) -> Result<(), Error<I2C::Error>> {
        self.set_feature(&ReportConfig::new(sensor, interval_us))
            .await
    }

    /// Stops a sensor reporting.
    pub async fn disable_report(&mut self, sensor: SensorId) -> Result<(), Error<I2C::Error>> {
        self.set_feature(&ReportConfig::new(sensor, 0)).await
    }

    /// Configures a report in full, for the cases [`Bno085::enable_report`]
    /// does not cover.
    pub async fn set_feature(&mut self, config: &ReportConfig) -> Result<(), Error<I2C::Error>> {
        let mut payload = [0u8; 17];
        payload[0] = REPORT_SET_FEATURE_COMMAND;
        payload[1] = config.sensor as u8;
        payload[2] = config.flags();
        payload[3..5].copy_from_slice(&config.change_sensitivity.to_le_bytes());
        payload[5..9].copy_from_slice(&config.interval_us.to_le_bytes());
        payload[9..13].copy_from_slice(&config.batch_interval_us.to_le_bytes());
        payload[13..17].copy_from_slice(&config.sensor_specific.to_le_bytes());

        self.write_cargo(Channel::Control, &payload).await
    }

    /// Reads the next sensor report, if one is waiting.
    ///
    /// One packet often carries several reports, so call this in a loop until
    /// it returns `Ok(None)` before going back to waiting on the `INT` pin.
    /// Packets that are not sensor reports are consumed and skipped.
    pub async fn read_event(&mut self) -> Result<Option<Event>, Error<I2C::Error>> {
        loop {
            if let Some(event) = self.next_buffered_event() {
                return Ok(Some(event));
            }

            match self.read_cargo().await? {
                Some(header) => {
                    self.cargo_len = header.cargo_len as usize;
                    self.cursor = HEADER_LEN;
                    self.channel = header.channel;
                }
                None => return Ok(None),
            }
        }
    }

    /// Zeroes the current orientation, so that where the sensor points now
    /// becomes the reference the chosen output reports against.
    ///
    /// The tare is lost on reset unless [`Bno085::persist_tare`] follows.
    pub async fn tare(&mut self, axes: u8, basis: TareBasis) -> Result<(), Error<I2C::Error>> {
        self.send_command(COMMAND_TARE, &[TARE_NOW, axes, basis as u8, 0, 0, 0, 0, 0, 0])
            .await
    }

    /// Writes the current tare to flash so it survives a reset.
    pub async fn persist_tare(&mut self) -> Result<(), Error<I2C::Error>> {
        self.send_command(COMMAND_TARE, &[TARE_PERSIST, 0, 0, 0, 0, 0, 0, 0, 0])
            .await
    }

    /// Chooses which sensors the hub keeps calibrating as it runs.
    ///
    /// The hub calibrates the accelerometer and magnetometer by default. Gyro
    /// calibration is worth enabling if the device spends time still.
    pub async fn configure_calibration(
        &mut self,
        accelerometer: bool,
        gyroscope: bool,
        magnetometer: bool,
    ) -> Result<(), Error<I2C::Error>> {
        self.send_command(
            COMMAND_ME_CALIBRATE,
            &[
                u8::from(accelerometer),
                u8::from(gyroscope),
                u8::from(magnetometer),
                0, // subcommand 0: configure
                0, // planar accelerometer calibration
                0,
                0,
                0,
                0,
            ],
        )
        .await
    }

    /// Saves the running calibration to flash.
    ///
    /// Worth doing once the hub reports [`Accuracy::High`], so the next boot
    /// starts from a good calibration instead of relearning it.
    pub async fn save_calibration(&mut self) -> Result<(), Error<I2C::Error>> {
        self.send_command(COMMAND_SAVE_DCD, &[0; 9]).await
    }

    /// Sends a raw command request. See the SH-2 Reference Manual for the
    /// command numbers and their parameters.
    pub async fn send_command(
        &mut self,
        command: u8,
        params: &[u8; 9],
    ) -> Result<(), Error<I2C::Error>> {
        let mut payload = [0u8; 12];
        payload[0] = REPORT_COMMAND_REQUEST;
        payload[1] = self.command_sequence;
        payload[2] = command;
        payload[3..12].copy_from_slice(params);
        self.command_sequence = self.command_sequence.wrapping_add(1);

        self.write_cargo(Channel::Control, &payload).await
    }

    /// Gives the I²C bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Pulls the next report out of the buffered cargo, if there is one left.
    fn next_buffered_event(&mut self) -> Option<Event> {
        match Channel::from_u8(self.channel) {
            // The gyro-integrated rotation vector gets a channel to itself and
            // fills the whole payload: no report ID, no timestamp, no status.
            Some(Channel::InputGyroRv) => {
                if self.cursor >= self.cargo_len {
                    return None;
                }
                let payload = &self.buf[HEADER_LEN..self.cargo_len];
                self.cursor = self.cargo_len;

                let expected = SensorId::GyroIntegratedRotationVector.report_len();
                (payload.len() >= expected).then(|| reports::decode_gyro_integrated_rv(payload))
            }

            Some(Channel::InputNormal | Channel::InputWake) => {
                while self.cursor < self.cargo_len {
                    let id = self.buf[self.cursor];
                    let remaining = self.cargo_len - self.cursor;

                    // Timestamp reports are not sensor data; they set the
                    // reference the reports after them are measured against.
                    if matches!(id, REPORT_BASE_TIMESTAMP | REPORT_TIMESTAMP_REBASE) {
                        if remaining < 5 {
                            break;
                        }
                        let ticks = i32::from_le_bytes([
                            self.buf[self.cursor + 1],
                            self.buf[self.cursor + 2],
                            self.buf[self.cursor + 3],
                            self.buf[self.cursor + 4],
                        ]);
                        self.reference_delta = if id == REPORT_BASE_TIMESTAMP {
                            -ticks
                        } else {
                            self.reference_delta.wrapping_add(ticks)
                        };
                        self.cursor += 5;
                        continue;
                    }

                    // Without a length for this report there is no way to find
                    // where the next one starts, so the rest of the packet has
                    // to go.
                    let Some(sensor) = SensorId::from_u8(id) else {
                        break;
                    };
                    let len = sensor.report_len();
                    if len > remaining {
                        break;
                    }

                    let start = self.cursor;
                    let event = reports::decode(
                        sensor,
                        &self.buf[start..start + len],
                        self.reference_delta,
                    );
                    self.cursor = start + len;
                    return Some(event);
                }

                self.discard_buffered();
                None
            }

            // Advertisements, command responses and the like.
            _ => {
                self.discard_buffered();
                None
            }
        }
    }

    fn discard_buffered(&mut self) {
        self.cargo_len = 0;
        self.cursor = 0;
    }

    /// Reads whatever cargo the hub has waiting into `self.buf`.
    ///
    /// Returns `Ok(None)` when the hub has nothing to send. Every read
    /// transaction comes back with a four byte header in front of it, so a
    /// four byte read peeks the length without consuming any payload, and
    /// each chunk after the first has a header to skip.
    async fn read_cargo(&mut self) -> Result<Option<Header>, Error<I2C::Error>> {
        let mut head = [0u8; HEADER_LEN];
        self.i2c
            .read(self.address, &mut head)
            .await
            .map_err(Error::I2c)?;

        let Some(header) = Header::parse(head) else {
            return Ok(None);
        };

        let total = header.cargo_len as usize;
        if total > MAX_CARGO {
            self.discard_cargo(total).await?;
            return Err(Error::CargoTooLarge(header.cargo_len));
        }

        let mut remaining = total;
        let mut written = 0;
        let mut first = true;

        while remaining > 0 {
            let want = if first {
                remaining.min(MAX_TRANSFER)
            } else {
                (remaining + HEADER_LEN).min(MAX_TRANSFER)
            };

            self.i2c
                .read(self.address, &mut self.scratch[..want])
                .await
                .map_err(Error::I2c)?;

            // The first chunk's header is the cargo's own; later chunks
            // carry a repeat of it that is not part of the payload.
            let chunk = if first {
                &self.scratch[..want]
            } else {
                &self.scratch[HEADER_LEN..want]
            };

            self.buf[written..written + chunk.len()].copy_from_slice(chunk);
            written += chunk.len();
            remaining -= chunk.len();
            first = false;
        }

        Ok(Some(header))
    }

    /// Reads a cargo the driver has no room for and throws it away, so the
    /// hub moves on to the next one.
    async fn discard_cargo(&mut self, total: usize) -> Result<(), Error<I2C::Error>> {
        let mut remaining = total;
        let mut first = true;

        while remaining > 0 {
            let want = if first {
                remaining.min(MAX_TRANSFER)
            } else {
                (remaining + HEADER_LEN).min(MAX_TRANSFER)
            };

            self.i2c
                .read(self.address, &mut self.scratch[..want])
                .await
                .map_err(Error::I2c)?;

            remaining -= if first { want } else { want - HEADER_LEN };
            first = false;
        }

        Ok(())
    }

    /// Waits for a particular report to arrive on the control channel,
    /// discarding anything else that turns up first. Returns the length of
    /// the payload, which is left at `self.buf[HEADER_LEN..]`.
    async fn await_control_report(
        &mut self,
        report_id: u8,
        min_len: usize,
    ) -> Result<usize, Error<I2C::Error>> {
        let mut waited = 0;

        loop {
            match self.read_cargo().await {
                Ok(Some(header)) => {
                    if header.channel == Channel::Control as u8
                        && header.payload_len() >= min_len
                        && self.buf[HEADER_LEN] == report_id
                    {
                        return Ok(header.payload_len());
                    }
                }
                Ok(None) => {
                    self.delay.delay_ms(POLL_INTERVAL_MS).await;
                    waited += POLL_INTERVAL_MS;
                }
                Err(Error::CargoTooLarge(_)) => {}
                Err(error) => return Err(error),
            }

            if waited >= RESPONSE_TIMEOUT_MS {
                return Err(Error::Timeout);
            }
        }
    }

    /// Frames a payload and sends it on `channel`.
    async fn write_cargo(
        &mut self,
        channel: Channel,
        payload: &[u8],
    ) -> Result<(), Error<I2C::Error>> {
        debug_assert!(payload.len() + HEADER_LEN <= MAX_WRITE);

        let sequence = &mut self.sequence[channel as usize];
        let header = Header::encode(payload.len(), channel, *sequence);
        *sequence = sequence.wrapping_add(1);

        let mut frame = [0u8; MAX_WRITE];
        frame[..HEADER_LEN].copy_from_slice(&header);
        frame[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);

        self.i2c
            .write(self.address, &frame[..HEADER_LEN + payload.len()])
            .await
            .map_err(Error::I2c)
    }
}
