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
//! No library covers the TF03, so this is written here, following Benewake's
//! *TF03 UART/CAN User Manual* (the 2024 edition, and V1.2.2 as hosted by
//! Seeed) cross-checked against ArduPilot's
//! `libraries/AP_RangeFinder/AP_RangeFinder_Benewake.cpp`.
//!
//! Nothing here touches hardware, so `zig build test` exercises it directly.

const std = @import("std");

/// Both of the first two bytes of every data frame.
pub const data_header: u8 = 0x59;
/// Length of a data frame.
pub const data_frame_len = 9;
/// First byte of every command and every response.
pub const command_header: u8 = 0x5A;
/// Shortest command frame: header, length, ID and checksum, with no payload.
pub const min_command_len = 4;
/// Longest command frame in the protocol, which belongs to the commands taking
/// a four byte parameter (UART baud rate, CAN IDs).
pub const max_command_len = 8;
/// Longest command payload.
pub const max_payload = max_command_len - min_command_len;

/// The low eight bits of the sum of `bytes`.
pub fn checksum(bytes: []const u8) u8 {
    var sum: u8 = 0;
    for (bytes) |byte| sum +%= byte;
    return sum;
}

/// A run of at most `max_payload` bytes, held by value.
pub const Payload = struct {
    bytes: [max_payload]u8 = @splat(0),
    len: usize = 0,

    pub fn slice(payload: *const Payload) []const u8 {
        return payload.bytes[0..payload.len];
    }

    pub fn eql(a: Payload, b: Payload) bool {
        return std.mem.eql(u8, a.slice(), b.slice());
    }
};

/// An outgoing command frame.
pub const Command = struct {
    frame: [max_command_len]u8 = @splat(0),
    len: usize,

    /// Frames `payload` as the command `id`. A payload longer than
    /// `max_payload` is cut short; no command in the protocol has one.
    pub fn init(command_id: u8, data: []const u8) Command {
        const payload_len = @min(data.len, max_payload);
        var command: Command = .{ .len = min_command_len + payload_len };
        command.frame[0] = command_header;
        command.frame[1] = @intCast(command.len);
        command.frame[2] = command_id;
        @memcpy(command.frame[3..][0..payload_len], data[0..payload_len]);
        command.frame[command.len - 1] = checksum(command.frame[0 .. command.len - 1]);
        return command;
    }

    pub fn id(command: Command) u8 {
        return command.frame[2];
    }

    pub fn payload(command: Command) Payload {
        var result: Payload = .{ .len = command.len - min_command_len };
        @memcpy(result.bytes[0..result.len], command.frame[3..][0..result.len]);
        return result;
    }

    /// The complete frame, ready to write to the UART.
    pub fn bytes(command: *const Command) []const u8 {
        return command.frame[0..command.len];
    }
};

/// The fields of a data frame, exactly as sent.
pub const DataFrame = struct {
    distance_cm: u16,
    strength: u16,
};

/// A command frame sent back by the TF03 in answer to a command.
pub const Response = struct {
    /// The ID of the command being answered.
    id: u8,
    payload: Payload,
};

/// A packet received from the TF03.
pub const Packet = union(enum) {
    data: DataFrame,
    response: Response,
};

/// Decodes a packet whose header, length and checksum have been checked.
fn decode(packet: []const u8) Packet {
    if (packet[0] == data_header) {
        return .{ .data = .{
            .distance_cm = std.mem.readInt(u16, packet[2..4], .little),
            .strength = std.mem.readInt(u16, packet[4..6], .little),
        } };
    }
    const received = packet[3 .. packet.len - 1];
    var response: Response = .{ .id = packet[2], .payload = .{ .len = received.len } };
    @memcpy(response.payload.bytes[0..received.len], received);
    return .{ .response = response };
}

/// Pulls packets out of a byte stream.
///
/// Bytes are fed in one at a time as they arrive. A header is not trusted
/// until the checksum at the end of its packet agrees, and on a mismatch the
/// parser steps forward a single byte rather than throwing the whole candidate
/// away. Discarding whole candidates can lock a parser out of step: a checksum
/// byte of `0x59` directly before a real frame reads as a header, and for as
/// long as the measurement does not change, every retry after a nine byte
/// discard lands on the same false header.
pub const Parser = struct {
    const max_packet_len = @max(data_frame_len, max_command_len);

    buf: [max_packet_len]u8 = @splat(0),
    len: usize = 0,

    /// Forgets any partly received packet, e.g. after bytes were lost.
    pub fn reset(parser: *Parser) void {
        parser.len = 0;
    }

    /// Adds one received byte, returning a packet if it completed one.
    pub fn push(parser: *Parser, byte: u8) ?Packet {
        // There is always room: this only returns once the buffer is shorter
        // than the packet it starts with, and no packet is longer than the
        // buffer.
        parser.buf[parser.len] = byte;
        parser.len += 1;

        while (true) {
            while (parser.len > 0 and !parser.plausibleStart()) parser.consume(1);

            const needed = parser.expectedLen() orelse return null;
            if (parser.len < needed) return null;

            const candidate = parser.buf[0..needed];
            if (checksum(candidate[0 .. needed - 1]) == candidate[needed - 1]) {
                const packet = decode(candidate);
                parser.consume(needed);
                return packet;
            }

            // A false header, or a corrupted packet. A real one may begin
            // anywhere after its first byte.
            parser.consume(1);
        }
    }

    fn plausibleStart(parser: *const Parser) bool {
        const first = parser.buf[0];
        if (first != data_header and first != command_header) return false;
        if (parser.len == 1) return true;
        if (first == data_header) return parser.buf[1] == data_header;
        return parser.buf[1] >= min_command_len and parser.buf[1] <= max_command_len;
    }

    fn expectedLen(parser: *const Parser) ?usize {
        if (parser.len < 2) return null;
        return if (parser.buf[0] == data_header) data_frame_len else parser.buf[1];
    }

    fn consume(parser: *Parser, count: usize) void {
        std.mem.copyForwards(u8, parser.buf[0 .. parser.len - count], parser.buf[count..parser.len]);
        parser.len -= count;
    }
};

// ---------------------------------------------------------------------------
// Tests: every case here is something the parser has to survive on a real,
// noisy, already-streaming line.
// ---------------------------------------------------------------------------

const testing = std.testing;

/// A data frame as the TF03 sends it, checksum included.
fn dataFrame(distance_cm: u16, strength: u16) [data_frame_len]u8 {
    var frame: [data_frame_len]u8 = .{ data_header, data_header, 0, 0, 0, 0, 0, 0, 0 };
    std.mem.writeInt(u16, frame[2..4], distance_cm, .little);
    std.mem.writeInt(u16, frame[4..6], strength, .little);
    frame[8] = checksum(frame[0..8]);
    return frame;
}

/// Feeds `bytes` to `parser`, collecting the packets that come out into
/// `out`, and returns how many there were.
fn feed(parser: *Parser, bytes: []const u8, out: []Packet) usize {
    var count: usize = 0;
    for (bytes) |byte| {
        if (parser.push(byte)) |packet| {
            if (count < out.len) out[count] = packet;
            count += 1;
        }
    }
    return count;
}

test "checksum is the low byte of the sum" {
    try testing.expectEqual(@as(u8, 0x11), checksum(&.{ 0xFF, 0x02, 0x10 }));
}

test "frames a command" {
    // The firmware version request from the manual: 5A 04 01 5F.
    const command: Command = .init(0x01, &.{});
    try testing.expectEqualSlices(u8, &.{ 0x5A, 0x04, 0x01, 0x5F }, command.bytes());
    try testing.expectEqual(@as(u8, 0x01), command.id());
    try testing.expectEqual(@as(usize, 0), command.payload().len);
}

test "frames a command with a payload" {
    // Frame rate 100 Hz: 5A 06 03 64 00 C7.
    const command: Command = .init(0x03, &.{ 0x64, 0x00 });
    try testing.expectEqualSlices(u8, &.{ 0x5A, 0x06, 0x03, 0x64, 0x00, 0xC7 }, command.bytes());
}

test "decodes a data frame" {
    var parser: Parser = .{};
    var packets: [4]Packet = undefined;
    try testing.expectEqual(@as(usize, 1), feed(&parser, &dataFrame(1234, 567), &packets));
    try testing.expectEqual(DataFrame{ .distance_cm = 1234, .strength = 567 }, packets[0].data);
}

test "decodes a response" {
    // Firmware version 1.2.3, sent patch first.
    var frame: [7]u8 = .{ 0x5A, 0x07, 0x01, 3, 2, 1, 0 };
    frame[6] = checksum(frame[0..6]);

    var parser: Parser = .{};
    var packets: [4]Packet = undefined;
    try testing.expectEqual(@as(usize, 1), feed(&parser, &frame, &packets));
    const response = packets[0].response;
    try testing.expectEqual(@as(u8, 0x01), response.id);
    try testing.expectEqualSlices(u8, &.{ 3, 2, 1 }, response.payload.slice());
}

test "skips garbage before a frame" {
    const bytes = [_]u8{ 0x00, 0x13, 0x59, 0x42 } ++ dataFrame(100, 200);
    var parser: Parser = .{};
    var packets: [4]Packet = undefined;
    try testing.expectEqual(@as(usize, 1), feed(&parser, &bytes, &packets));
    try testing.expectEqual(@as(u16, 100), packets[0].data.distance_cm);
}

test "rejects a corrupted frame and recovers on the next" {
    var corrupted = dataFrame(100, 200);
    corrupted[3] ^= 0x01;
    const bytes = corrupted ++ dataFrame(300, 400);

    var parser: Parser = .{};
    var packets: [4]Packet = undefined;
    try testing.expectEqual(@as(usize, 1), feed(&parser, &bytes, &packets));
    try testing.expectEqual(@as(u16, 300), packets[0].data.distance_cm);
}

test "does not lock onto a false header" {
    // A frame whose checksum is 0x59, followed by more of the same: a parser
    // that throws away nine bytes after a bad candidate keeps landing on the
    // false header formed by that checksum and the next frame's first byte.
    // Find a distance that makes the checksum 0x59.
    var frame = dataFrame(0, 0);
    for (0..0x100) |distance| {
        frame = dataFrame(@intCast(distance), 0);
        if (frame[8] == data_header) break;
    }
    try testing.expectEqual(data_header, frame[8]);

    // Start mid-frame, so the parser has to find its footing.
    const bytes = frame[4..].* ++ frame ++ frame ++ frame ++ frame ++ frame;

    var parser: Parser = .{};
    var packets: [8]Packet = undefined;
    try testing.expectEqual(@as(usize, 5), feed(&parser, &bytes, &packets));
}

test "reset forgets a partial packet" {
    const frame = dataFrame(100, 200);
    var parser: Parser = .{};
    var packets: [4]Packet = undefined;
    _ = feed(&parser, frame[0..5], &packets);
    parser.reset();
    // The tail of the old frame no longer completes anything.
    try testing.expectEqual(@as(usize, 0), feed(&parser, frame[5..], &packets));
    try testing.expectEqual(@as(usize, 1), feed(&parser, &frame, &packets));
}
