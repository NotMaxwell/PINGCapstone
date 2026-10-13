//! The bus underneath the BNO085 driver.
//!
//! SHTP cargos look the same on every bus; how they get on and off the wire
//! does not. A [`Transport`] moves whole cargos, and the driver above it
//! never touches the bus directly.
//!
//! * [`SpiTransport`] is what PING uses. SPI needs no clock stretching, which
//!   the ESP32's I²C peripheral handles poorly, and is the faster link.
//! * [`I2cTransport`] is the original I²C path, kept for boards wired that
//!   way.
//!
//! Both own the hub's `INT` line, so the driver can wait for data the same
//! way on either.

use embassy_time::{Duration, Timer, with_timeout};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::SpiBus;

use super::shtp::{HEADER_LEN, Header};
use super::{Address, Error, MAX_CARGO, MAX_TRANSFER, MAX_WRITE};

/// How long the hub may take to answer a wake request on SPI. The datasheet
/// expects the host to act within about 10 ms of `INT`; the hub itself is
/// quicker than that.
const WAKE_TIMEOUT: Duration = Duration::from_millis(50);

/// How long `NRST` is held low for a hardware reset.
const RESET_PULSE: Duration = Duration::from_millis(10);

/// How long the hub may take to boot after a hardware reset before it raises
/// `INT` to send its advertisement.
const BOOT_TIMEOUT: Duration = Duration::from_millis(500);

/// Moves SHTP cargos between the driver and the hub.
#[allow(
    async_fn_in_trait,
    reason = "used only with concrete types in this crate's binaries"
)]
pub trait Transport {
    /// What the underlying bus or pins can fail with.
    type Error;

    /// Puts the hub through a hardware reset, where the wiring allows one.
    async fn reset(&mut self) -> Result<(), Error<Self::Error>>;

    /// Waits until the hub signals that it has a cargo waiting.
    async fn wait_for_data(&mut self) -> Result<(), Error<Self::Error>>;

    /// Reads the next cargo into `buf`, header included.
    ///
    /// Returns `Ok(None)` when the hub has nothing to send. A cargo larger
    /// than `buf` is read and thrown away, and reported as
    /// [`Error::CargoTooLarge`], so the hub moves on to the next one.
    async fn read_cargo(
        &mut self,
        buf: &mut [u8; MAX_CARGO],
    ) -> Result<Option<Header>, Error<Self::Error>>;

    /// Sends one complete cargo, header included.
    async fn write_cargo(&mut self, frame: &[u8]) -> Result<(), Error<Self::Error>>;
}

// ---------------------------------------------------------------------------
// SPI
// ---------------------------------------------------------------------------

/// What the SPI transport can fail with.
#[derive(Debug)]
pub enum SpiError<S, P> {
    /// The SPI bus returned an error.
    Spi(S),
    /// One of the control pins returned an error.
    Pin(P),
}

/// The BNO085 on a four-wire SPI bus (mode 3), with its `INT`, `WAKE` and
/// `NRST` lines.
///
/// The hub selects SPI when `PS0` and `PS1` are both high at reset. `PS1` is
/// tied high on the board; `PS0` doubles as `WAKE`, so it must already be
/// driven high when [`Transport::reset`] releases `NRST`. Create the `WAKE`
/// output high.
///
/// Chip select is driven by hand rather than through an `SpiDevice`, because
/// a read has to look at the header before it knows how many more bytes to
/// clock in while chip select stays low.
pub struct SpiTransport<SPI, CS, INT, WAKE, RST> {
    spi: SPI,
    cs: CS,
    int: INT,
    wake: WAKE,
    reset: RST,
    /// SPI is full duplex, so a write also clocks in whatever cargo the hub
    /// had waiting. That cargo is kept here and handed out by the next read.
    stash: [u8; MAX_CARGO],
    /// Length of the stashed cargo, or zero when there is none.
    stash_len: usize,
    /// Landing area for bytes read only to be thrown away.
    scratch: [u8; MAX_TRANSFER],
}

impl<SPI, CS, INT, WAKE, RST, P> SpiTransport<SPI, CS, INT, WAKE, RST>
where
    SPI: SpiBus,
    CS: OutputPin<Error = P>,
    INT: InputPin<Error = P> + Wait,
    WAKE: OutputPin<Error = P>,
    RST: OutputPin<Error = P>,
{
    /// Takes the bus and pins. `cs` and `wake` should start high.
    pub fn new(spi: SPI, cs: CS, int: INT, wake: WAKE, reset: RST) -> Self {
        Self {
            spi,
            cs,
            int,
            wake,
            reset,
            stash: [0; MAX_CARGO],
            stash_len: 0,
            scratch: [0; MAX_TRANSFER],
        }
    }

    /// Asks the hub to wake up and waits for it to say it is listening.
    async fn wake(&mut self) -> Result<(), Error<SpiError<SPI::Error, P>>> {
        self.wake.set_low().map_err(pin)?;
        let ready = with_timeout(WAKE_TIMEOUT, self.int.wait_for_low()).await;
        self.wake.set_high().map_err(pin)?;
        match ready {
            Ok(result) => result.map_err(pin),
            Err(_) => Err(Error::Timeout),
        }
    }

    /// Clocks in `count` bytes and throws them away. Chip select must be low.
    async fn discard(&mut self, mut count: usize) -> Result<(), Error<SpiError<SPI::Error, P>>> {
        while count > 0 {
            let want = count.min(MAX_TRANSFER);
            self.spi
                .read(&mut self.scratch[..want])
                .await
                .map_err(spi)?;
            count -= want;
        }
        Ok(())
    }

    /// Ends a transaction: waits for the bus to finish, then raises chip
    /// select.
    async fn deselect(&mut self) -> Result<(), Error<SpiError<SPI::Error, P>>> {
        self.spi.flush().await.map_err(spi)?;
        self.cs.set_high().map_err(pin)
    }

    /// Gives the bus and pins back.
    pub fn release(self) -> (SPI, CS, INT, WAKE, RST) {
        (self.spi, self.cs, self.int, self.wake, self.reset)
    }
}

fn spi<S, P>(error: S) -> Error<SpiError<S, P>> {
    Error::Bus(SpiError::Spi(error))
}

fn pin<S, P>(error: P) -> Error<SpiError<S, P>> {
    Error::Bus(SpiError::Pin(error))
}

impl<SPI, CS, INT, WAKE, RST, P> Transport for SpiTransport<SPI, CS, INT, WAKE, RST>
where
    SPI: SpiBus,
    CS: OutputPin<Error = P>,
    INT: InputPin<Error = P> + Wait,
    WAKE: OutputPin<Error = P>,
    RST: OutputPin<Error = P>,
{
    type Error = SpiError<SPI::Error, P>;

    async fn reset(&mut self) -> Result<(), Error<Self::Error>> {
        // PS0 must be high as NRST rises for the hub to come up in SPI mode.
        self.wake.set_high().map_err(pin)?;
        self.cs.set_high().map_err(pin)?;
        self.stash_len = 0;

        self.reset.set_low().map_err(pin)?;
        Timer::after(RESET_PULSE).await;
        self.reset.set_high().map_err(pin)?;

        // The hub raises INT once it has booted, to send its advertisement.
        match with_timeout(BOOT_TIMEOUT, self.int.wait_for_low()).await {
            Ok(result) => result.map_err(pin),
            Err(_) => Err(Error::Timeout),
        }
    }

    async fn wait_for_data(&mut self) -> Result<(), Error<Self::Error>> {
        if self.stash_len > 0 {
            return Ok(());
        }
        self.int.wait_for_low().await.map_err(pin)
    }

    async fn read_cargo(
        &mut self,
        buf: &mut [u8; MAX_CARGO],
    ) -> Result<Option<Header>, Error<Self::Error>> {
        if self.stash_len > 0 {
            let len = self.stash_len;
            self.stash_len = 0;
            buf[..len].copy_from_slice(&self.stash[..len]);
            let head = [buf[0], buf[1], buf[2], buf[3]];
            return Ok(Header::parse(head));
        }

        // On SPI the hub only has something to send while INT is low.
        if self.int.is_high().map_err(pin)? {
            return Ok(None);
        }

        self.cs.set_low().map_err(pin)?;

        let mut head = [0u8; HEADER_LEN];
        if let Err(error) = self.spi.read(&mut head).await {
            self.cs.set_high().map_err(pin)?;
            return Err(spi(error));
        }

        let Some(header) = Header::parse(head) else {
            self.deselect().await?;
            return Ok(None);
        };

        let total = header.cargo_len as usize;
        if total > MAX_CARGO {
            self.discard(total - HEADER_LEN).await?;
            self.deselect().await?;
            return Err(Error::CargoTooLarge(header.cargo_len));
        }

        buf[..HEADER_LEN].copy_from_slice(&head);
        let body = self.spi.read(&mut buf[HEADER_LEN..total]).await;
        self.deselect().await?;
        body.map_err(spi)?;

        Ok(Some(header))
    }

    async fn write_cargo(&mut self, frame: &[u8]) -> Result<(), Error<Self::Error>> {
        debug_assert!(frame.len() <= MAX_WRITE && frame.len() >= HEADER_LEN);

        self.wake().await?;
        self.cs.set_low().map_err(pin)?;

        let sent = frame.len();
        let mut incoming = [0u8; MAX_WRITE];
        if let Err(error) = self.spi.transfer(&mut incoming[..sent], frame).await {
            self.cs.set_high().map_err(pin)?;
            return Err(spi(error));
        }

        // Whatever the hub clocked out while we wrote is the start of a cargo
        // it had waiting. Read the rest of it now, while chip select is still
        // low, and keep it for the next read.
        let head = [incoming[0], incoming[1], incoming[2], incoming[3]];
        if let Some(header) = Header::parse(head) {
            let total = header.cargo_len as usize;
            if total <= MAX_CARGO {
                let have = sent.min(total);
                self.stash[..have].copy_from_slice(&incoming[..have]);
                if total > sent {
                    let rest = self.spi.read(&mut self.stash[sent..total]).await;
                    if let Err(error) = rest {
                        self.cs.set_high().map_err(pin)?;
                        return Err(spi(error));
                    }
                }
                self.stash_len = total;
            } else if total > sent {
                // Too large to keep: the reset advertisement, which the
                // driver would discard anyway.
                self.discard(total - sent).await?;
            }
        }

        self.deselect().await
    }
}

// ---------------------------------------------------------------------------
// I2C
// ---------------------------------------------------------------------------

/// What the I²C transport can fail with.
#[derive(Debug)]
pub enum I2cError<B, P> {
    /// The I²C bus returned an error.
    I2c(B),
    /// The `INT` pin returned an error.
    Pin(P),
}

/// The BNO085 on an I²C bus, with its `INT` line.
pub struct I2cTransport<I2C, INT> {
    i2c: I2C,
    address: u8,
    int: INT,
    /// Landing area for one I²C transfer.
    scratch: [u8; MAX_TRANSFER],
}

impl<I2C, INT> I2cTransport<I2C, INT>
where
    I2C: I2c,
    INT: Wait,
{
    pub fn new(i2c: I2C, address: Address, int: INT) -> Self {
        Self {
            i2c,
            address: address as u8,
            int,
            scratch: [0; MAX_TRANSFER],
        }
    }

    /// Gives the bus and pin back.
    pub fn release(self) -> (I2C, INT) {
        (self.i2c, self.int)
    }

    /// Reads a cargo the driver has no room for and throws it away, so the
    /// hub moves on to the next one.
    async fn discard_cargo(
        &mut self,
        total: usize,
    ) -> Result<(), Error<I2cError<I2C::Error, INT::Error>>> {
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
                .map_err(|e| Error::Bus(I2cError::I2c(e)))?;

            remaining -= if first { want } else { want - HEADER_LEN };
            first = false;
        }

        Ok(())
    }
}

impl<I2C, INT> Transport for I2cTransport<I2C, INT>
where
    I2C: I2c,
    INT: Wait,
{
    type Error = I2cError<I2C::Error, INT::Error>;

    /// No reset line on this wiring; the driver's soft reset does the job.
    async fn reset(&mut self) -> Result<(), Error<Self::Error>> {
        Ok(())
    }

    async fn wait_for_data(&mut self) -> Result<(), Error<Self::Error>> {
        self.int
            .wait_for_low()
            .await
            .map_err(|e| Error::Bus(I2cError::Pin(e)))
    }

    /// Every read transaction comes back with a four byte header in front of
    /// it, so a four byte read peeks the length without consuming any
    /// payload, and each chunk after the first has a header to skip.
    async fn read_cargo(
        &mut self,
        buf: &mut [u8; MAX_CARGO],
    ) -> Result<Option<Header>, Error<Self::Error>> {
        let mut head = [0u8; HEADER_LEN];
        self.i2c
            .read(self.address, &mut head)
            .await
            .map_err(|e| Error::Bus(I2cError::I2c(e)))?;

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
                .map_err(|e| Error::Bus(I2cError::I2c(e)))?;

            // The first chunk's header is the cargo's own; later chunks
            // carry a repeat of it that is not part of the payload.
            let chunk = if first {
                &self.scratch[..want]
            } else {
                &self.scratch[HEADER_LEN..want]
            };

            buf[written..written + chunk.len()].copy_from_slice(chunk);
            written += chunk.len();
            remaining -= chunk.len();
            first = false;
        }

        Ok(Some(header))
    }

    async fn write_cargo(&mut self, frame: &[u8]) -> Result<(), Error<Self::Error>> {
        self.i2c
            .write(self.address, frame)
            .await
            .map_err(|e| Error::Bus(I2cError::I2c(e)))
    }
}
