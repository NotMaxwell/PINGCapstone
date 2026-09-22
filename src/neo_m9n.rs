//! Async UART driver for the u-blox NEO-M9N GNSS receiver (as on the
//! SparkFun GPS-15733 breakout).
//!
//! The receiver speaks two protocols on the same line: NMEA text, which it
//! sends from the factory, and u-blox's binary UBX. This driver uses UBX
//! only. It asks for one `UBX-NAV-PVT` message per navigation solution, which
//! carries position, velocity, time and their accuracy estimates in a single
//! 100 byte frame, and turns NMEA output off so the line carries nothing
//! else.
//!
//! Framing, checksums and field decoding come from the [`ublox`] crate; this
//! module adds the UART plumbing, configuration and a plain [`Solution`] type
//! that does not borrow from the parser.
//!
//! # Interface
//!
//! The NEO-M9N has UART, I²C and SPI. This driver uses UART1, which the
//! SparkFun board breaks out as TX/RX at 3.3 V, so it wires straight to
//! ESP32-S3 GPIOs. The factory baud rate is 38400 8N1. A NAV-PVT frame is 100
//! bytes, so 38400 baud carries it at the receiver's full 25 Hz with room to
//! spare, and there is no reason to change the rate.
//!
//! # Configuration
//!
//! The M9 generation is configured with key/value pairs (`UBX-CFG-VALSET`);
//! the older `CFG-PRT`/`CFG-MSG` messages are gone. [`NeoM9n::configure`]
//! writes to the RAM layer only, so nothing is left behind in the receiver's
//! flash and a fresh board and a reused one behave the same after a power
//! cycle. Call it once at start-up.
//!
//! # Example
//!
//! ```ignore
//! let mut gps = NeoM9n::new(uart);
//! gps.configure(10).await?; // 10 Hz, UBX only
//!
//! loop {
//!     let solution = gps.read().await?;
//!     if let Some((latitude, longitude)) = solution.position() {
//!         // ...
//!     }
//! }
//! ```

use embassy_time::{Duration, with_timeout};
use embedded_io_async::{Read, Write};
use ublox::cfg_val::CfgVal;
use ublox::packets::cfg_val::{CfgLayerSet, CfgValSet, CfgValSetBuilder};
use ublox::packets::mon_ver::{MonVer, MonVerRef};
use ublox::packets::nav_pvt::common::{NavPvtFlags, NavPvtValidFlags};
use ublox::packets::nav_pvt::proto31::NavPvtRef;
use ublox::proto31::{PacketRef, Proto31};
use ublox::{FixedBuffer, GnssFixType, Parser, UbxPacket, UbxPacketMeta, UbxPacketRequest};

/// The baud rate the NEO-M9N's UART1 uses from the factory.
pub const DEFAULT_BAUD_RATE: u32 = 38_400;

/// The fastest navigation rate the NEO-M9N supports.
pub const MAX_NAV_RATE_HZ: u8 = 25;

/// How long the receiver may take to answer a poll or acknowledge a
/// configuration. u-blox answers within a navigation epoch; one second covers
/// the slowest rate this driver configures.
const RESPONSE_TIMEOUT: Duration = Duration::from_secs(1);

/// Largest chunk taken from the UART in one read.
const RX_CHUNK: usize = 64;

/// How much the UBX parser holds between reads. The largest message this
/// driver reads is `UBX-MON-VER`: 40 bytes plus 30 per extension string, a
/// few hundred bytes in all.
const PARSER_BUF: usize = 1024;

/// Room for the configuration [`NeoM9n::configure`] sends: a 4 byte header,
/// six keys of at most 6 bytes each, and 8 bytes of framing.
const CONFIG_BUF: usize = 64;

/// What the driver can fail at.
#[derive(Debug)]
pub enum Error<E> {
    /// The underlying UART returned an error. On esp-hal this includes RX
    /// FIFO overflows, after which reading can simply continue.
    Uart(E),
    /// The UART stream reported end of file.
    EndOfStream,
    /// The receiver did not answer in time.
    Timeout,
    /// The receiver refused a configuration message (`UBX-ACK-NAK`).
    Rejected,
    /// The receiver does not support the value asked for.
    Unsupported,
}

/// What kind of solution the receiver has.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FixType {
    NoFix,
    /// Position from dead reckoning alone. The NEO-M9N has no sensor fusion,
    /// so this should not appear.
    DeadReckoningOnly,
    Fix2D,
    Fix3D,
    GnssPlusDeadReckoning,
    /// Time is known but position is not trusted, e.g. in a fixed-position
    /// timing mode.
    TimeOnly,
}

impl From<GnssFixType> for FixType {
    fn from(fix: GnssFixType) -> Self {
        match fix {
            GnssFixType::NoFix => Self::NoFix,
            GnssFixType::DeadReckoningOnly => Self::DeadReckoningOnly,
            GnssFixType::Fix2D => Self::Fix2D,
            GnssFixType::Fix3D => Self::Fix3D,
            GnssFixType::GPSPlusDeadReckoning => Self::GnssPlusDeadReckoning,
            GnssFixType::TimeOnlyFix => Self::TimeOnly,
            // Values 6-255 are reserved; no solution worth using carries one.
            _ => Self::NoFix,
        }
    }
}

/// UTC date and time of a solution.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct UtcTime {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    /// Fraction of the second, which can be negative: the receiver rounds
    /// the other fields to the nearest second and puts the remainder here.
    pub nanosecond: i32,
}

/// One navigation solution, decoded from `UBX-NAV-PVT`.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Solution {
    /// GPS time of week of the navigation epoch, milliseconds. Solutions
    /// from the same epoch share it, which makes it the key for matching
    /// this against other u-blox messages.
    pub itow_ms: u32,
    pub fix_type: FixType,
    /// The fix is within the receiver's DOP and accuracy masks. Treat the
    /// position as usable only when this is set; [`Solution::position`] does.
    pub fix_ok: bool,
    /// Differential corrections (e.g. SBAS) were applied.
    pub differential: bool,
    /// Satellites used in the solution.
    pub satellites: u8,
    /// Degrees, positive north.
    pub latitude_deg: f64,
    /// Degrees, positive east.
    pub longitude_deg: f64,
    /// Height above the WGS-84 ellipsoid, metres.
    pub height_ellipsoid_m: f32,
    /// Height above mean sea level, metres.
    pub height_msl_m: f32,
    /// 1σ horizontal position accuracy estimate, metres.
    pub horizontal_accuracy_m: f32,
    /// 1σ vertical position accuracy estimate, metres.
    pub vertical_accuracy_m: f32,
    /// Velocity north, east and down, m/s.
    pub velocity_ned_mps: [f32; 3],
    /// Horizontal speed, m/s.
    pub ground_speed_mps: f32,
    /// 1σ speed accuracy estimate, m/s.
    pub speed_accuracy_mps: f32,
    /// Direction of travel, degrees clockwise from north. Meaningless when
    /// standing still.
    pub heading_of_motion_deg: f32,
    /// 1σ heading accuracy estimate, degrees.
    pub heading_accuracy_deg: f32,
    /// Position dilution of precision.
    pub pdop: f32,
    /// UTC time of the solution, once the receiver has resolved it fully.
    pub utc: Option<UtcTime>,
    /// The receiver flagged latitude, longitude and heights as invalid
    /// despite a fix.
    invalid_position: bool,
}

impl Solution {
    /// Latitude and longitude in degrees, when the receiver vouches for them.
    pub fn position(&self) -> Option<(f64, f64)> {
        let has_position = matches!(
            self.fix_type,
            FixType::Fix2D | FixType::Fix3D | FixType::GnssPlusDeadReckoning
        );
        (has_position && self.fix_ok && !self.invalid_position)
            .then_some((self.latitude_deg, self.longitude_deg))
    }

    fn from_nav_pvt(pvt: &NavPvtRef<'_>) -> Self {
        let valid = pvt.valid();
        let utc_resolved = valid.contains(
            NavPvtValidFlags::VALID_DATE
                | NavPvtValidFlags::VALID_TIME
                | NavPvtValidFlags::FULLY_RESOLVED,
        );
        let flags = pvt.flags();

        Self {
            itow_ms: pvt.itow(),
            fix_type: pvt.fix_type().into(),
            fix_ok: flags.contains(NavPvtFlags::GPS_FIX_OK),
            differential: flags.contains(NavPvtFlags::DIFF_SOLN),
            satellites: pvt.num_satellites(),
            latitude_deg: pvt.latitude(),
            longitude_deg: pvt.longitude(),
            height_ellipsoid_m: pvt.height_above_ellipsoid() as f32,
            height_msl_m: pvt.height_msl() as f32,
            horizontal_accuracy_m: pvt.horizontal_accuracy() as f32,
            vertical_accuracy_m: pvt.vertical_accuracy() as f32,
            velocity_ned_mps: [
                pvt.vel_north() as f32,
                pvt.vel_east() as f32,
                pvt.vel_down() as f32,
            ],
            ground_speed_mps: pvt.ground_speed_2d() as f32,
            speed_accuracy_mps: pvt.speed_accuracy() as f32,
            heading_of_motion_deg: pvt.heading_motion() as f32,
            heading_accuracy_deg: pvt.heading_accuracy() as f32,
            pdop: pvt.pdop() as f32,
            utc: utc_resolved.then(|| UtcTime {
                year: pvt.year(),
                month: pvt.month(),
                day: pvt.day(),
                hour: pvt.hour(),
                minute: pvt.min(),
                second: pvt.sec(),
                nanosecond: pvt.nanosec(),
            }),
            invalid_position: pvt.flags3().invalid_llh(),
        }
    }
}

/// A short string copied out of a UBX message, so it outlives the parser's
/// buffer.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct FixedStr<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

impl<const N: usize> FixedStr<N> {
    /// Copies as much of `s` as fits, cut back to a character boundary.
    fn new(s: &str) -> Self {
        let mut len = s.len().min(N);
        while !s.is_char_boundary(len) {
            len -= 1;
        }
        let mut bytes = [0; N];
        bytes[..len].copy_from_slice(&s.as_bytes()[..len]);
        Self { bytes, len }
    }

    pub fn as_str(&self) -> &str {
        // Only ever filled from a `&str` cut at a character boundary.
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or_default()
    }
}

impl<const N: usize> core::fmt::Debug for FixedStr<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self.as_str(), f)
    }
}

impl<const N: usize> core::fmt::Display for FixedStr<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// What the receiver says it is, from `UBX-MON-VER`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Version {
    /// Receiver software version, e.g. `ROM SPG 4.04 (...)`.
    pub software: FixedStr<30>,
    /// Hardware version, e.g. `000A0000`.
    pub hardware: FixedStr<10>,
    /// Module name from the `MOD=` extension, e.g. `NEO-M9N`.
    pub module: Option<FixedStr<30>>,
    /// UBX protocol version from the `PROTVER=` extension, e.g. `32.01`.
    pub protocol: Option<FixedStr<30>>,
}

impl Version {
    fn from_mon_ver(ver: &MonVerRef<'_>) -> Self {
        let mut version = Self {
            software: FixedStr::new(ver.software_version()),
            hardware: FixedStr::new(ver.hardware_version()),
            module: None,
            protocol: None,
        };
        for extension in ver.extension() {
            if let Some(module) = extension.strip_prefix("MOD=") {
                version.module = Some(FixedStr::new(module));
            } else if let Some(protocol) = extension.strip_prefix("PROTVER=") {
                version.protocol = Some(FixedStr::new(protocol));
            }
        }
        version
    }
}

/// The UBX messages this driver acts on, copied out of the parser.
enum Message {
    Solution(Solution),
    Version(Version),
    Ack { class: u8, id: u8 },
    Nak { class: u8, id: u8 },
}

impl Message {
    fn from_packet(packet: UbxPacket<'_>) -> Option<Self> {
        let UbxPacket::Proto31(packet) = packet;
        match packet {
            PacketRef::NavPvt(pvt) => Some(Self::Solution(Solution::from_nav_pvt(&pvt))),
            PacketRef::MonVer(ver) => Some(Self::Version(Version::from_mon_ver(&ver))),
            PacketRef::AckAck(ack) => Some(Self::Ack {
                class: ack.class(),
                id: ack.msg_id(),
            }),
            PacketRef::AckNak(nak) => Some(Self::Nak {
                class: nak.class(),
                id: nak.msg_id(),
            }),
            _ => None,
        }
    }
}

/// A NEO-M9N on a UART.
pub struct NeoM9n<U> {
    uart: U,
    /// UBX frames are up to a few hundred bytes and arrive split across UART
    /// reads, so the parser keeps a partial frame between them.
    parser: Parser<FixedBuffer<PARSER_BUF>, Proto31>,
    /// The last chunk read from the UART. Kept here rather than on the stack
    /// so it does not inflate the future of every call that reads.
    rx_buf: [u8; RX_CHUNK],
    /// How much of `rx_buf` holds bytes not yet handed to the parser.
    rx_len: usize,
}

impl<U> NeoM9n<U> {
    /// Wraps a UART already configured to match the receiver,
    /// [`DEFAULT_BAUD_RATE`] 8N1 from the factory. Nothing is sent.
    pub const fn new(uart: U) -> Self {
        Self {
            uart,
            parser: Parser::with_fixed_buffer(),
            rx_buf: [0; RX_CHUNK],
            rx_len: 0,
        }
    }

    /// Hands the UART back.
    pub fn release(self) -> U {
        self.uart
    }
}

impl<U: Read> NeoM9n<U> {
    /// Waits for the next navigation solution.
    ///
    /// Solutions arrive once per navigation epoch whether or not there is a
    /// fix; check [`Solution::position`] before using the position.
    pub async fn read(&mut self) -> Result<Solution, Error<U::Error>> {
        loop {
            if let Message::Solution(solution) = self.read_message().await? {
                return Ok(solution);
            }
        }
    }

    /// Waits for the next UBX message this driver understands.
    ///
    /// Cancelling this is safe as long as the UART's own read is: received
    /// bytes are only handed to the parser once a read has completed.
    async fn read_message(&mut self) -> Result<Message, Error<U::Error>> {
        loop {
            // Hand over whatever the last read brought, then walk the parser
            // for a complete frame. Stopping part way leaves the rest in the
            // parser's buffer for next time. Corrupted frames and NMEA text
            // are skipped by the parser.
            let fresh = core::mem::take(&mut self.rx_len);
            let mut packets = self.parser.consume_ubx(&self.rx_buf[..fresh]);
            while let Some(result) = packets.next() {
                if let Ok(packet) = result
                    && let Some(message) = Message::from_packet(packet)
                {
                    return Ok(message);
                }
            }
            drop(packets);

            match self.uart.read(&mut self.rx_buf).await {
                Ok(0) => return Err(Error::EndOfStream),
                Ok(count) => self.rx_len = count,
                Err(error) => {
                    // Bytes were probably lost, so a partial frame in the
                    // parser would be spliced onto an unrelated later one.
                    self.parser = Parser::with_fixed_buffer();
                    return Err(Error::Uart(error));
                }
            }
        }
    }
}

impl<U: Read + Write> NeoM9n<U> {
    /// Switches UART1 to UBX-only output and sets the receiver to send one
    /// `UBX-NAV-PVT` per solution, `rate_hz` solutions a second.
    ///
    /// Written to RAM only; see the module documentation. `rate_hz` must be
    /// between 1 and [`MAX_NAV_RATE_HZ`].
    pub async fn configure(&mut self, rate_hz: u8) -> Result<(), Error<U::Error>> {
        if !(1..=MAX_NAV_RATE_HZ).contains(&rate_hz) {
            return Err(Error::Unsupported);
        }

        let settings = [
            CfgVal::Uart1InProtUbx(true),
            CfgVal::Uart1OutProtUbx(true),
            CfgVal::Uart1OutProtNmea(false),
            // One NAV-PVT per navigation solution.
            CfgVal::MsgOutUbxNavPvtUart1(1),
            // Measure every 1000 / rate_hz ms, and compute a solution from
            // every measurement.
            CfgVal::RateMeas(1000 / u16::from(rate_hz)),
            CfgVal::RateNav(1),
        ];

        let mut writer = FrameWriter::<CONFIG_BUF>::new();
        CfgValSetBuilder {
            version: 0,
            layers: CfgLayerSet::RAM,
            reserved1: 0,
            cfg_data: &settings,
        }
        .extend_to(&mut writer);
        // CONFIG_BUF is sized for exactly these settings, so this only fails
        // if someone adds one without growing it.
        let frame = writer.frame().ok_or(Error::Unsupported)?;

        self.send(frame).await?;
        with_timeout(RESPONSE_TIMEOUT, self.await_ack::<CfgValSet>())
            .await
            .map_err(|_| Error::Timeout)?
    }

    /// Asks the receiver what it is. Also a cheap way to check it is there
    /// and the baud rate matches.
    pub async fn version(&mut self) -> Result<Version, Error<U::Error>> {
        self.send(&UbxPacketRequest::request_for::<MonVer>().into_packet_bytes())
            .await?;

        with_timeout(RESPONSE_TIMEOUT, async {
            loop {
                if let Message::Version(version) = self.read_message().await? {
                    return Ok(version);
                }
            }
        })
        .await
        .map_err(|_| Error::Timeout)?
    }

    /// Waits for the receiver to acknowledge or refuse the message `T`,
    /// skipping everything else.
    async fn await_ack<T: UbxPacketMeta>(&mut self) -> Result<(), Error<U::Error>> {
        loop {
            match self.read_message().await? {
                Message::Ack { class, id } if class == T::CLASS && id == T::ID => return Ok(()),
                Message::Nak { class, id } if class == T::CLASS && id == T::ID => {
                    return Err(Error::Rejected);
                }
                _ => {}
            }
        }
    }

    async fn send(&mut self, bytes: &[u8]) -> Result<(), Error<U::Error>> {
        self.uart.write_all(bytes).await.map_err(Error::Uart)?;
        self.uart.flush().await.map_err(Error::Uart)
    }
}

/// Somewhere for the `ublox` crate to build an outgoing frame without a heap.
/// The crate's builders write through `Extend<u8>` and only ship an
/// implementation for `Vec`.
struct FrameWriter<const N: usize> {
    bytes: [u8; N],
    len: usize,
    /// `Extend` has no way to report running out of room, so it is recorded
    /// here and checked once the frame is built.
    overflowed: bool,
}

impl<const N: usize> FrameWriter<N> {
    const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            overflowed: false,
        }
    }

    /// The finished frame, or `None` if it did not fit.
    fn frame(&self) -> Option<&[u8]> {
        (!self.overflowed).then_some(&self.bytes[..self.len])
    }
}

impl<const N: usize> Extend<u8> for FrameWriter<N> {
    fn extend<I: IntoIterator<Item = u8>>(&mut self, bytes: I) {
        for byte in bytes {
            match self.bytes.get_mut(self.len) {
                Some(slot) => {
                    *slot = byte;
                    self.len += 1;
                }
                None => self.overflowed = true,
            }
        }
    }
}

// The builders patch the length field and checksum over what they have
// written so far, so they need to see it as a slice.
impl<const N: usize> core::ops::Deref for FrameWriter<N> {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl<const N: usize> core::ops::DerefMut for FrameWriter<N> {
    fn deref_mut(&mut self) -> &mut [u8] {
        &mut self.bytes[..self.len]
    }
}
