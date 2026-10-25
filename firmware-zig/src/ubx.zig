//! The UBX messages the NEO-M9N driver sends and reads.
//!
//! Framing and checksums are u-blox's own code: `uUbxProtocolEncode` and
//! `uUbxProtocolDecode` from ubxlib, vendored in `components/ubxlib_ubx`. So
//! are the configuration key IDs (`u_gnss_cfg_val_key.h`) and the NAV-PVT fix
//! types and flag bits (`u_gnss_dec_ubx_nav_pvt.h`). This file lays out the
//! message bodies on top: CFG-VALSET, NAV-PVT and MON-VER, following the
//! u-blox M9 SPG 4.04 interface description.
//!
//! Nothing here touches hardware: the host tests build the same ubxlib C
//! sources for the host and run through `zig build test`.

const std = @import("std");
const c = @import("ubx");

pub const MessageId = struct {
    class: u8,
    id: u8,

    pub fn eql(a: MessageId, b: MessageId) bool {
        return a.class == b.class and a.id == b.id;
    }
};

pub const nav_pvt: MessageId = .{
    .class = c.U_GNSS_DEC_UBX_NAV_PVT_MESSAGE_CLASS,
    .id = c.U_GNSS_DEC_UBX_NAV_PVT_MESSAGE_ID,
};
pub const mon_ver: MessageId = .{ .class = 0x0A, .id = 0x04 };
pub const cfg_valset: MessageId = .{ .class = 0x06, .id = 0x8A };
pub const ack_ack: MessageId = .{ .class = 0x05, .id = 0x01 };
pub const ack_nak: MessageId = .{ .class = 0x05, .id = 0x00 };

/// Header, length and checksum around every body.
pub const overhead = c.U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES;

/// Frames `body` as the message `message` into `out`, which needs `overhead`
/// bytes more than the body.
pub fn encode(message: MessageId, body: []const u8, out: []u8) []const u8 {
    std.debug.assert(out.len >= body.len + overhead);
    const len = c.uUbxProtocolEncode(message.class, message.id, body.ptr, body.len, out.ptr);
    std.debug.assert(len >= 0);
    return out[0..@intCast(len)];
}

/// A received message. `body` is valid until the next call to
/// `Decoder.next`.
pub const Message = struct {
    id: MessageId,
    body: []const u8,
    /// The message was longer than the decoder keeps; `body` is its start.
    truncated: bool,
};

/// Pulls whole messages out of a byte stream, skipping anything between them
/// (NMEA sentences included).
///
/// Received bytes go into `unused()` and are then `commit`ted. ubxlib's
/// decoder looks at a whole buffer at a time, so a message cut off at the end
/// stays in the buffer, from its header on, until the rest of it arrives.
pub const Decoder = struct {
    /// Room for several NAV-PVTs (100 bytes each) and a MON-VER.
    buf: [1024]u8 = undefined,
    len: usize = 0,
    body: [256]u8 = undefined,

    /// Where the next received bytes go.
    pub fn unused(decoder: *Decoder) []u8 {
        return decoder.buf[decoder.len..];
    }

    /// Adds the `count` bytes just written to `unused()`.
    pub fn commit(decoder: *Decoder, count: usize) void {
        decoder.len += count;
    }

    /// Throws away everything received so far.
    pub fn reset(decoder: *Decoder) void {
        decoder.len = 0;
    }

    /// The next complete message received, if there is one.
    pub fn next(decoder: *Decoder) ?Message {
        if (decoder.len == 0) return null;

        var class: i32 = 0;
        var id: i32 = 0;
        var end: [*c]const u8 = undefined;
        const result = c.uUbxProtocolDecode(&decoder.buf, decoder.len, &class, &id, &decoder.body, decoder.body.len, &end);

        if (result >= 0) {
            decoder.drop(@intFromPtr(end) - @intFromPtr(&decoder.buf));
            const body_len: usize = @intCast(result);
            return .{
                .id = .{ .class = @intCast(class), .id = @intCast(id) },
                .body = decoder.body[0..@min(body_len, decoder.body.len)],
                .truncated = body_len > decoder.body.len,
            };
        }

        if (result == c.U_ERROR_COMMON_TIMEOUT) {
            // The start of a message, still arriving. Keep it from its first
            // sync byte on. If it has filled the buffer it never will arrive
            // (its length field was garbage), so look past that sync byte.
            const sync = std.mem.indexOfScalar(u8, decoder.buf[0..decoder.len], 0xB5) orelse decoder.len;
            decoder.drop(sync);
            if (decoder.len == decoder.buf.len) {
                const after = std.mem.indexOfScalarPos(u8, decoder.buf[0..decoder.len], 1, 0xB5) orelse decoder.len;
                decoder.drop(after);
            }
            return null;
        }

        // Nothing that could become a message.
        decoder.len = 0;
        return null;
    }

    fn drop(decoder: *Decoder, count: usize) void {
        std.mem.copyForwards(u8, decoder.buf[0 .. decoder.len - count], decoder.buf[count..decoder.len]);
        decoder.len -= count;
    }
};

// ---------------------------------------------------------------------------
// CFG-VALSET
// ---------------------------------------------------------------------------

/// The configuration layers a CFG-VALSET writes to.
pub const Layer = enum(u8) {
    ram = 0x01,
    bbr = 0x02,
    flash = 0x04,
};

/// Builds the body of one CFG-VALSET: up to 64 key and value pairs, written
/// to `layer` together.
pub const Valset = struct {
    body: [4 + 64 * 12]u8 = undefined,
    len: usize = 4,

    pub fn init(layer: Layer) Valset {
        var valset: Valset = .{};
        // Version 0, the layer, two reserved bytes.
        valset.body[0..4].* = .{ 0, @intFromEnum(layer), 0, 0 };
        return valset;
    }

    /// Adds one item. The key says how wide its value is; `value` is cut to
    /// that width.
    pub fn add(valset: *Valset, key: u32, value: u64) void {
        const size: usize = switch ((key >> 28) & 0x07) {
            1, 2 => 1, // one bit (stored in a byte), or one byte
            3 => 2,
            4 => 4,
            5 => 8,
            else => unreachable,
        };
        std.mem.writeInt(u32, valset.body[valset.len..][0..4], key, .little);
        var value_bytes: [8]u8 = undefined;
        std.mem.writeInt(u64, &value_bytes, value, .little);
        @memcpy(valset.body[valset.len + 4 ..][0..size], value_bytes[0..size]);
        valset.len += 4 + size;
    }

    pub fn slice(valset: *const Valset) []const u8 {
        return valset.body[0..valset.len];
    }
};

/// The configuration the driver sends: UBX in and out on UART1, NMEA out off,
/// a measurement every `1000 / rate_hz` ms with a solution from each, and one
/// NAV-PVT per solution, all in the RAM layer.
pub fn boardConfig(rate_hz: u8) Valset {
    var valset: Valset = .init(.ram);
    valset.add(c.U_GNSS_CFG_VAL_KEY_ID_UART1INPROT_UBX_L, 1);
    valset.add(c.U_GNSS_CFG_VAL_KEY_ID_UART1OUTPROT_UBX_L, 1);
    valset.add(c.U_GNSS_CFG_VAL_KEY_ID_UART1OUTPROT_NMEA_L, 0);
    valset.add(c.U_GNSS_CFG_VAL_KEY_ID_RATE_MEAS_U2, 1000 / @as(u16, rate_hz));
    valset.add(c.U_GNSS_CFG_VAL_KEY_ID_RATE_NAV_U2, 1);
    valset.add(c.U_GNSS_CFG_VAL_KEY_ID_MSGOUT_UBX_NAV_PVT_UART1_U1, 1);
    return valset;
}

/// Whether `message` acknowledges (true) or refuses (false) `request`, or is
/// about something else (null).
pub fn ackFor(message: Message, request: MessageId) ?bool {
    const acked = message.id.eql(ack_ack);
    if (!acked and !message.id.eql(ack_nak)) return null;
    if (message.body.len < 2 or message.body[0] != request.class or message.body[1] != request.id) return null;
    return acked;
}

// ---------------------------------------------------------------------------
// NAV-PVT
// ---------------------------------------------------------------------------

/// What kind of solution the receiver has.
pub const FixType = enum(u8) {
    no_fix = c.U_GNSS_DEC_UBX_NAV_PVT_FIX_TYPE_NO_FIX,
    /// Position from dead reckoning alone. The NEO-M9N has no sensor fusion,
    /// so this should not appear.
    dead_reckoning_only = c.U_GNSS_DEC_UBX_NAV_PVT_FIX_TYPE_DEAD_RECKONING_ONLY,
    fix_2d = c.U_GNSS_DEC_UBX_NAV_PVT_FIX_TYPE_2D,
    fix_3d = c.U_GNSS_DEC_UBX_NAV_PVT_FIX_TYPE_3D,
    gnss_plus_dead_reckoning = c.U_GNSS_DEC_UBX_NAV_PVT_FIX_TYPE_GNSS_PLUS_DEAD_RECKONING,
    /// Time is known but position is not trusted.
    time_only = c.U_GNSS_DEC_UBX_NAV_PVT_FIX_TYPE_TIME_ONLY,

    pub fn name(fix: FixType) []const u8 {
        return switch (fix) {
            .no_fix => "no fix",
            .dead_reckoning_only => "dead reckoning",
            .fix_2d => "2D",
            .fix_3d => "3D",
            .gnss_plus_dead_reckoning => "GNSS+DR",
            .time_only => "time only",
        };
    }
};

/// UTC date and time of a solution.
pub const UtcTime = struct {
    year: u16,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: u8,
    /// Fraction of the second, which can be negative: the receiver rounds the
    /// other fields to the nearest second and puts the remainder here.
    nanosecond: i32,
};

/// One navigation solution, decoded from `UBX-NAV-PVT`.
pub const Solution = struct {
    /// GPS time of week of the navigation epoch, milliseconds.
    itow_ms: u32,
    fix_type: FixType,
    /// The fix is within the receiver's DOP and accuracy masks. Treat the
    /// position as usable only when this is set; `hasPosition` does.
    fix_ok: bool,
    /// Differential corrections (e.g. SBAS) were applied.
    differential: bool,
    /// Satellites used in the solution.
    satellites: u8,
    /// Degrees, positive north.
    latitude_deg: f64,
    /// Degrees, positive east.
    longitude_deg: f64,
    /// Height above the WGS-84 ellipsoid, metres.
    height_ellipsoid_m: f32,
    /// Height above mean sea level, metres.
    height_msl_m: f32,
    /// 1σ horizontal position accuracy estimate, metres.
    horizontal_accuracy_m: f32,
    /// 1σ vertical position accuracy estimate, metres.
    vertical_accuracy_m: f32,
    /// Velocity north, east and down, m/s.
    velocity_ned_mps: [3]f32,
    /// Horizontal speed, m/s.
    ground_speed_mps: f32,
    /// 1σ speed accuracy estimate, m/s.
    speed_accuracy_mps: f32,
    /// Direction of travel, degrees clockwise from north. Meaningless when
    /// standing still.
    heading_of_motion_deg: f32,
    /// 1σ heading accuracy estimate, degrees.
    heading_accuracy_deg: f32,
    /// Position dilution of precision.
    pdop: f32,
    /// UTC time of the solution, once the receiver has resolved it fully.
    utc: ?UtcTime,
    /// The receiver flagged latitude, longitude and heights as invalid despite
    /// a fix.
    invalid_position: bool,

    /// Whether the receiver vouches for `latitude_deg` and `longitude_deg`.
    pub fn hasPosition(solution: Solution) bool {
        const positioned = switch (solution.fix_type) {
            .fix_2d, .fix_3d, .gnss_plus_dead_reckoning => true,
            else => false,
        };
        return positioned and solution.fix_ok and !solution.invalid_position;
    }
};

/// Length of a NAV-PVT body.
pub const nav_pvt_len = 92;

fn bit(byte: u8, index: c_int) bool {
    return (byte >> @intCast(index)) & 1 != 0;
}

/// Decodes the body of a NAV-PVT, or null if it is too short to be one.
pub fn decodeNavPvt(body: []const u8) ?Solution {
    if (body.len < nav_pvt_len) return null;
    const b = body[0..nav_pvt_len];
    const u = struct {
        fn int(comptime T: type, bytes: *const [nav_pvt_len]u8, comptime offset: usize) T {
            return std.mem.readInt(T, bytes[offset..][0..@sizeOf(T)], .little);
        }
    }.int;
    const milli = 1e-3;

    const valid = b[11];
    const flags = b[21];
    const utc_known = bit(valid, c.U_GNSS_DEC_UBX_NAV_PVT_VALID_DATE) and
        bit(valid, c.U_GNSS_DEC_UBX_NAV_PVT_VALID_TIME) and
        bit(valid, c.U_GNSS_DEC_UBX_NAV_PVT_VALID_FULLY_RESOLVED);

    return .{
        .itow_ms = u(u32, b, 0),
        // Values 6-255 are reserved; no solution worth using carries one.
        .fix_type = std.enums.fromInt(FixType, b[20]) orelse .no_fix,
        .fix_ok = bit(flags, c.U_GNSS_DEC_UBX_NAV_PVT_FLAGS_GNSS_FIX_OK),
        .differential = bit(flags, c.U_GNSS_DEC_UBX_NAV_PVT_FLAGS_DIFF_SOLN),
        .satellites = b[23],
        .longitude_deg = @as(f64, @floatFromInt(u(i32, b, 24))) * 1e-7,
        .latitude_deg = @as(f64, @floatFromInt(u(i32, b, 28))) * 1e-7,
        .height_ellipsoid_m = @as(f32, @floatFromInt(u(i32, b, 32))) * milli,
        .height_msl_m = @as(f32, @floatFromInt(u(i32, b, 36))) * milli,
        .horizontal_accuracy_m = @as(f32, @floatFromInt(u(u32, b, 40))) * milli,
        .vertical_accuracy_m = @as(f32, @floatFromInt(u(u32, b, 44))) * milli,
        .velocity_ned_mps = .{
            @as(f32, @floatFromInt(u(i32, b, 48))) * milli,
            @as(f32, @floatFromInt(u(i32, b, 52))) * milli,
            @as(f32, @floatFromInt(u(i32, b, 56))) * milli,
        },
        .ground_speed_mps = @as(f32, @floatFromInt(u(i32, b, 60))) * milli,
        .heading_of_motion_deg = @as(f32, @floatFromInt(u(i32, b, 64))) * 1e-5,
        .speed_accuracy_mps = @as(f32, @floatFromInt(u(u32, b, 68))) * milli,
        .heading_accuracy_deg = @as(f32, @floatFromInt(u(u32, b, 72))) * 1e-5,
        .pdop = @as(f32, @floatFromInt(u(u16, b, 76))) * 1e-2,
        .utc = if (utc_known) .{
            .year = u(u16, b, 4),
            .month = b[6],
            .day = b[7],
            .hour = b[8],
            .minute = b[9],
            .second = b[10],
            .nanosecond = u(i32, b, 16),
        } else null,
        .invalid_position = bit(b[78], c.U_GNSS_DEC_UBX_NAV_PVT_FLAGS3_INVALID_LLH),
    };
}

// ---------------------------------------------------------------------------
// MON-VER
// ---------------------------------------------------------------------------

/// What the receiver says it is, from `UBX-MON-VER`.
pub const Version = struct {
    module_buf: [16]u8 = @splat(0),
    module_len: usize = 0,
    firmware_type_buf: [8]u8 = @splat(0),
    firmware_type_len: usize = 0,
    firmware_major: u8 = 0,
    firmware_minor: u8 = 0,
    protocol_major: u8 = 0,
    protocol_minor: u8 = 0,

    /// e.g. `NEO-M9N`.
    pub fn module(version: *const Version) []const u8 {
        return version.module_buf[0..version.module_len];
    }

    /// e.g. `SPG`.
    pub fn firmwareType(version: *const Version) []const u8 {
        return version.firmware_type_buf[0..version.firmware_type_len];
    }
};

/// A `major.minor` pair such as `4.04`.
fn parseMajorMinor(text: []const u8) struct { u8, u8 } {
    var parts = std.mem.splitScalar(u8, std.mem.trim(u8, text, " "), '.');
    const major = std.fmt.parseInt(u8, parts.first(), 10) catch 0;
    const minor = std.fmt.parseInt(u8, parts.next() orelse "", 10) catch 0;
    return .{ major, minor };
}

fn copyInto(dest: []u8, text: []const u8) usize {
    const len = @min(dest.len, text.len);
    @memcpy(dest[0..len], text[0..len]);
    return len;
}

/// Picks the module name and the firmware and protocol versions out of a
/// MON-VER body: two version strings, then extensions such as
/// `FWVER=SPG 4.04`, `PROTVER=32.01` and `MOD=NEO-M9N`, each a 30 byte field.
pub fn parseMonVer(body: []const u8) Version {
    var version: Version = .{};
    var offset: usize = 40;
    while (offset + 30 <= body.len) : (offset += 30) {
        const field = body[offset..][0..30];
        const text = field[0 .. std.mem.indexOfScalar(u8, field, 0) orelse field.len];
        if (std.mem.startsWith(u8, text, "MOD=")) {
            version.module_len = copyInto(&version.module_buf, text[4..]);
        } else if (std.mem.startsWith(u8, text, "FWVER=")) {
            // A type such as SPG, then the version.
            const rest = text[6..];
            const space = std.mem.indexOfScalar(u8, rest, ' ') orelse rest.len;
            version.firmware_type_len = copyInto(&version.firmware_type_buf, rest[0..space]);
            version.firmware_major, version.firmware_minor = parseMajorMinor(rest[space..]);
        } else if (std.mem.startsWith(u8, text, "PROTVER=")) {
            version.protocol_major, version.protocol_minor = parseMajorMinor(text[8..]);
        }
    }
    return version;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

/// The UBX checksum, written out here to check ubxlib's against.
fn fletcher(bytes: []const u8) [2]u8 {
    var a: u8 = 0;
    var b: u8 = 0;
    for (bytes) |byte| {
        a +%= byte;
        b +%= a;
    }
    return .{ a, b };
}

test "encodes a poll with an empty body" {
    var out: [overhead]u8 = undefined;
    // MON-VER poll, as u-center sends it: B5 62 0A 04 00 00 0E 34.
    try testing.expectEqualSlices(u8, &.{ 0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34 }, encode(mon_ver, &.{}, &out));
}

test "board configuration is one CFG-VALSET to RAM" {
    const valset = boardConfig(10);
    const body = valset.slice();
    try testing.expectEqualSlices(u8, &.{ 0, 1, 0, 0 }, body[0..4]);
    // Three one-bit items, two U2 items and a U1 item.
    try testing.expectEqual(@as(usize, 4 + 3 * 5 + 2 * 6 + 5), body.len);
    // RATE-MEAS: key 0x30210001, 100 ms.
    try testing.expectEqualSlices(u8, &.{ 0x01, 0x00, 0x21, 0x30, 100, 0 }, body[19..25]);

    var out: [256]u8 = undefined;
    const frame = encode(cfg_valset, body, &out);
    try testing.expectEqualSlices(u8, &.{ 0xB5, 0x62, 0x06, 0x8A, @intCast(body.len), 0 }, frame[0..6]);
    try testing.expectEqualSlices(u8, &fletcher(frame[2 .. frame.len - 2]), frame[frame.len - 2 ..]);
}

test "decoder finds messages among noise and across reads" {
    var out: [64]u8 = undefined;
    var stream: [128]u8 = undefined;
    const nmea = "$GNGGA,,,,*7A\r\n";
    @memcpy(stream[0..nmea.len], nmea);
    var len: usize = nmea.len;
    const ack = encode(ack_ack, &.{ 0x06, 0x8A }, &out);
    @memcpy(stream[len..][0..ack.len], ack);
    len += ack.len;

    var decoder: Decoder = .{};
    // Everything but the last byte: nothing complete yet.
    @memcpy(decoder.unused()[0 .. len - 1], stream[0 .. len - 1]);
    decoder.commit(len - 1);
    try testing.expectEqual(@as(?Message, null), decoder.next());

    decoder.unused()[0] = stream[len - 1];
    decoder.commit(1);
    const message = decoder.next().?;
    try testing.expect(message.id.eql(ack_ack));
    try testing.expectEqual(@as(?bool, true), ackFor(message, cfg_valset));
    try testing.expectEqual(@as(?bool, null), ackFor(message, mon_ver));
    try testing.expectEqual(@as(?Message, null), decoder.next());
}

test "decodes a NAV-PVT" {
    var body: [nav_pvt_len]u8 = @splat(0);
    std.mem.writeInt(u32, body[0..4], 123_456, .little);
    std.mem.writeInt(u16, body[4..6], 2026, .little);
    body[6] = 10;
    body[7] = 23;
    body[11] = 0b0111; // date, time, fully resolved
    body[20] = 3; // 3D
    body[21] = 0b01; // fix OK
    body[23] = 14;
    std.mem.writeInt(i32, body[24..28], -1_051_234_567, .little);
    std.mem.writeInt(i32, body[28..32], 401_234_567, .little);
    std.mem.writeInt(i32, body[36..40], 1_600_500, .little);
    std.mem.writeInt(u32, body[40..44], 1_500, .little);
    std.mem.writeInt(u16, body[76..78], 123, .little);

    const solution = decodeNavPvt(&body).?;
    try testing.expectEqual(@as(u32, 123_456), solution.itow_ms);
    try testing.expectEqual(FixType.fix_3d, solution.fix_type);
    try testing.expect(solution.hasPosition());
    try testing.expectEqual(@as(u8, 14), solution.satellites);
    try testing.expectApproxEqAbs(@as(f64, 40.1234567), solution.latitude_deg, 1e-9);
    try testing.expectApproxEqAbs(@as(f64, -105.1234567), solution.longitude_deg, 1e-9);
    try testing.expectApproxEqAbs(@as(f32, 1600.5), solution.height_msl_m, 1e-3);
    try testing.expectApproxEqAbs(@as(f32, 1.5), solution.horizontal_accuracy_m, 1e-6);
    try testing.expectApproxEqAbs(@as(f32, 1.23), solution.pdop, 1e-6);
    try testing.expectEqual(@as(u16, 2026), solution.utc.?.year);

    body[78] = 1; // invalid lat, lon and height
    try testing.expect(!decodeNavPvt(&body).?.hasPosition());
    try testing.expectEqual(@as(?Solution, null), decodeNavPvt(body[0..36]));
}

test "parses MON-VER" {
    var body: [40 + 4 * 30]u8 = @splat(0);
    @memcpy(body[0..12], "ROM SPG 5.10");
    for ([_][]const u8{ "FWVER=SPG 4.04", "PROTVER=32.01", "MOD=NEO-M9N", "GPS;GLO;GAL;BDS" }, 0..) |text, i| {
        @memcpy(body[40 + i * 30 ..][0..text.len], text);
    }
    const version = parseMonVer(&body);
    try testing.expectEqualStrings("NEO-M9N", version.module());
    try testing.expectEqualStrings("SPG", version.firmwareType());
    try testing.expectEqual(@as(u8, 4), version.firmware_major);
    try testing.expectEqual(@as(u8, 4), version.firmware_minor);
    try testing.expectEqual(@as(u8, 32), version.protocol_major);
    try testing.expectEqual(@as(u8, 1), version.protocol_minor);
}
