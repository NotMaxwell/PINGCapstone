//! Wire format of the TF03's UART protocol.
//!
//! Two kinds of packet share the line, told apart by their first byte:
//!
//! * **Data frames**, nine bytes, sent continuously at the configured frame
//!   rate (or once per trigger command):
//!   `59 59 DIST_L DIST_H STRENGTH_L STRENGTH_H reserved reserved SUM`
//! * **Command frames**, `5A LEN ID payload… SUM`, where `LEN` counts every
//!   byte including the header and the checksum. The host sends these, and
//!   the TF03 answers in the same shape.
//!
//! Every packet ends in the low eight bits of the sum of the bytes before it,
//! and multi-byte fields are little endian.
//!
//! References: Benewake's *TF03 UART/CAN User Manual* (the 2024 edition, and
//! V1.2.2 as hosted by Seeed, which prints the data frame as a table rather
//! than a figure), cross-checked against ArduPilot's
//! `libraries/AP_RangeFinder/AP_RangeFinder_Benewake.cpp`.

/// Both of the first two bytes of every data frame.
pub const DATA_HEADER: u8 = 0x59;

/// Length of a data frame.
pub const DATA_FRAME_LEN: usize = 9;

/// First byte of every command and every response.
pub const COMMAND_HEADER: u8 = 0x5A;

/// Shortest command frame: header, length, ID and checksum, with no payload.
pub const MIN_COMMAND_LEN: usize = 4;

/// Longest command frame in the protocol, which belongs to the commands
/// taking a four byte parameter (UART baud rate, CAN IDs).
pub const MAX_COMMAND_LEN: usize = 8;

/// Longest command payload.
pub const MAX_PAYLOAD: usize = MAX_COMMAND_LEN - MIN_COMMAND_LEN;

/// Longest packet of either kind, and so how much the parser has to hold.
const MAX_PACKET_LEN: usize = if DATA_FRAME_LEN > MAX_COMMAND_LEN {
    DATA_FRAME_LEN
} else {
    MAX_COMMAND_LEN
};

/// The low eight bits of the sum of `bytes`.
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |sum, byte| sum.wrapping_add(*byte))
}

/// An outgoing command frame.
#[derive(Clone, Copy, Debug)]
pub struct Command {
    bytes: [u8; MAX_COMMAND_LEN],
    len: usize,
}

impl Command {
    /// Frames `payload` as the command `id`.
    ///
    /// # Panics
    ///
    /// If `payload` is longer than [`MAX_PAYLOAD`]. No command in the
    /// protocol has a longer one.
    pub fn new(id: u8, payload: &[u8]) -> Self {
        assert!(payload.len() <= MAX_PAYLOAD, "TF03 command payload too long");

        let len = MIN_COMMAND_LEN + payload.len();
        let mut bytes = [0; MAX_COMMAND_LEN];
        bytes[0] = COMMAND_HEADER;
        bytes[1] = len as u8;
        bytes[2] = id;
        bytes[3..len - 1].copy_from_slice(payload);
        bytes[len - 1] = checksum(&bytes[..len - 1]);

        Self { bytes, len }
    }

    pub const fn id(&self) -> u8 {
        self.bytes[2]
    }

    pub fn payload(&self) -> &[u8] {
        &self.bytes[3..self.len - 1]
    }

    /// The complete frame, ready to write to the UART.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

/// A packet received from the TF03.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Packet {
    Data(DataFrame),
    Response(Response),
}

/// The fields of a data frame, exactly as sent.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DataFrame {
    pub distance_cm: u16,
    pub strength: u16,
}

/// A command frame sent back by the TF03 in answer to a command.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Response {
    /// The ID of the command being answered.
    pub id: u8,
    payload: [u8; MAX_PAYLOAD],
    payload_len: usize,
}

impl Response {
    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.payload_len]
    }
}

/// Pulls packets out of a byte stream.
///
/// Bytes are fed in one at a time as they arrive. A header is not trusted
/// until the checksum at the end of its packet agrees, and on a mismatch the
/// parser steps forward a single byte rather than throwing the whole
/// candidate away. Discarding whole candidates can lock a parser out of
/// step: a checksum byte of `0x59` directly before a real frame reads as a
/// header, and for as long as the measurement does not change, every retry
/// after a nine byte discard lands on the same false header.
#[derive(Clone, Debug)]
pub struct Parser {
    buf: [u8; MAX_PACKET_LEN],
    len: usize,
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_PACKET_LEN],
            len: 0,
        }
    }

    /// Forgets any partly received packet, e.g. after bytes were lost.
    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// Adds one received byte, returning a packet if it completed one.
    pub fn push(&mut self, byte: u8) -> Option<Packet> {
        // There is always room: `settle` only returns once the buffer is
        // shorter than the packet it starts with, and no packet is longer
        // than the buffer.
        self.buf[self.len] = byte;
        self.len += 1;
        self.settle()
    }

    /// Decodes the packet at the front of the buffer if it is complete and
    /// sound, stepping past bytes that cannot begin one.
    fn settle(&mut self) -> Option<Packet> {
        loop {
            while self.len > 0 && !self.plausible_start() {
                self.consume(1);
            }

            let needed = self.expected_len()?;
            if self.len < needed {
                return None;
            }

            let candidate = &self.buf[..needed];
            if checksum(&candidate[..needed - 1]) == candidate[needed - 1] {
                let packet = decode(candidate);
                self.consume(needed);
                return Some(packet);
            }

            // A false header, or a corrupted packet. A real one may begin
            // anywhere after its first byte.
            self.consume(1);
        }
    }

    /// Whether the buffered bytes could be the beginning of a packet.
    fn plausible_start(&self) -> bool {
        match (self.buf[0], self.len) {
            (DATA_HEADER | COMMAND_HEADER, 1) => true,
            (DATA_HEADER, _) => self.buf[1] == DATA_HEADER,
            (COMMAND_HEADER, _) => {
                (MIN_COMMAND_LEN..=MAX_COMMAND_LEN).contains(&(self.buf[1] as usize))
            }
            _ => false,
        }
    }

    /// Length of the packet the buffer begins with, once enough of it has
    /// arrived to tell.
    fn expected_len(&self) -> Option<usize> {
        if self.len < 2 {
            return None;
        }

        Some(if self.buf[0] == DATA_HEADER {
            DATA_FRAME_LEN
        } else {
            self.buf[1] as usize
        })
    }

    fn consume(&mut self, count: usize) {
        self.buf.copy_within(count..self.len, 0);
        self.len -= count;
    }
}

/// Decodes a packet whose header, length and checksum have been checked.
fn decode(packet: &[u8]) -> Packet {
    if packet[0] == DATA_HEADER {
        Packet::Data(DataFrame {
            distance_cm: u16::from_le_bytes([packet[2], packet[3]]),
            strength: u16::from_le_bytes([packet[4], packet[5]]),
        })
    } else {
        let received = &packet[3..packet.len() - 1];
        let mut payload = [0; MAX_PAYLOAD];
        payload[..received.len()].copy_from_slice(received);

        Packet::Response(Response {
            id: packet[2],
            payload,
            payload_len: received.len(),
        })
    }
}
