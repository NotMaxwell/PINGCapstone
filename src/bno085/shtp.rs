//! SHTP — the Sensor Hub Transport Protocol that frames every BNO085 transfer.
//!
//! All traffic, in both directions, is a *cargo*: a four byte header followed
//! by a payload. The header carries the total cargo length (header included),
//! the channel the payload belongs to, and a per-channel sequence number.
//!
//! Reference: CEVA SH-2 / SHTP Reference Manual, and the reference driver at
//! <https://github.com/ceva-dsp/sh2>.

/// Length of the SHTP header prefixing every cargo.
pub const HEADER_LEN: usize = 4;

/// Set in the length field when a transfer continues the preceding cargo.
const CONTINUATION_BIT: u16 = 0x8000;

/// The SHTP channels the sensor hub advertises.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Channel {
    /// SHTP-level commands and the advertisement sent after reset.
    Command = 0,
    /// Reset requests, and the reset-complete notification.
    Executable = 1,
    /// Sensor hub control: feature commands, product ID, FRS records.
    Control = 2,
    /// Sensor reports from non-wake sensors.
    InputNormal = 3,
    /// Sensor reports from sensors configured as wake sources.
    InputWake = 4,
    /// The gyro-integrated rotation vector, which has its own channel.
    InputGyroRv = 5,
}

impl Channel {
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Command),
            1 => Some(Self::Executable),
            2 => Some(Self::Control),
            3 => Some(Self::InputNormal),
            4 => Some(Self::InputWake),
            5 => Some(Self::InputGyroRv),
            _ => None,
        }
    }
}

/// A decoded SHTP header.
#[derive(Clone, Copy, Debug)]
pub struct Header {
    /// Total cargo length in bytes, *including* these four header bytes.
    pub cargo_len: u16,
    /// Whether this transfer continues the previous one.
    pub continuation: bool,
    pub channel: u8,
    pub sequence: u8,
}

impl Header {
    /// Decodes a header, returning `None` when the bytes do not describe a
    /// cargo with a payload.
    ///
    /// A hub with nothing to say clocks out zeros, and a hub that is asleep or
    /// absent leaves the bus pulled high; a cargo of exactly `HEADER_LEN` has
    /// no payload to deliver. None of the three is worth reading further.
    pub const fn parse(bytes: [u8; HEADER_LEN]) -> Option<Self> {
        let raw = (bytes[0] as u16) | ((bytes[1] as u16) << 8);
        if raw == 0x0000 || raw == 0xFFFF {
            return None;
        }

        let cargo_len = raw & !CONTINUATION_BIT;
        if (cargo_len as usize) <= HEADER_LEN {
            return None;
        }

        Some(Self {
            cargo_len,
            continuation: (raw & CONTINUATION_BIT) != 0,
            channel: bytes[2],
            sequence: bytes[3],
        })
    }

    /// Payload length, i.e. the cargo without its header.
    pub const fn payload_len(&self) -> usize {
        self.cargo_len as usize - HEADER_LEN
    }

    /// Builds the header for an outgoing payload.
    pub const fn encode(payload_len: usize, channel: Channel, sequence: u8) -> [u8; HEADER_LEN] {
        let cargo_len = (payload_len + HEADER_LEN) as u16;
        [
            cargo_len as u8,
            (cargo_len >> 8) as u8,
            channel as u8,
            sequence,
        ]
    }
}
