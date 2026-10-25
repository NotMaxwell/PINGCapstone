//! UART driver for the Benewake TF03 long-range LiDAR (TF03-100 and TF03-180,
//! which differ only in maximum range).
//!
//! The TF03 measures distance by pulsed time of flight and, unless told
//! otherwise, streams a nine byte data frame per measurement at 100 Hz without
//! being asked. Configuration commands share the same line under a separate
//! framing; see `tf03_protocol.zig`.
//!
//! # Interface
//!
//! The standard TF03 talks UART or CAN, one at a time, and leaves the factory
//! on UART. It has no I²C. Its UART is 3.3 V LVTTL, so it wires straight to
//! ESP32-S3 GPIOs. This driver sits on ESP-IDF's own UART driver, which
//! buffers what arrives in the background, so a task that is slow to read
//! loses nothing until the 2 KiB ring buffer fills, about two seconds of
//! frames.
//!
//! `Tf03.read` needs only the TF03's output line. Everything else sends a
//! command and also needs the board's TX wired to the TF03.
//!
//! # Errors
//!
//! Failures come back as `Error`. The driver keeps what it knows about the
//! last one (what was heard on the line, the status the TF03 sent, the ESP-IDF
//! error) and `Tf03.describe` turns that into words for logs and the display.
//!
//! # Example
//!
//! ```zig
//! var lidar: tf03.Tf03 = .init(constants.lidar_uart);
//! rtos.must(lidar.begin(constants.lidar_tx, constants.lidar_rx, constants.lidar_baud_rate));
//! while (true) {
//!     const measurement = lidar.read(1000) catch continue;
//!     if (measurement.distance_cm) |cm| { ... }
//! }
//! ```

const std = @import("std");
const idf = @import("idf");
const rtos = @import("rtos.zig");
const protocol = @import("tf03_protocol.zig");

const Command = protocol.Command;
const Packet = protocol.Packet;
const Response = protocol.Response;

/// The distance the TF03 reports when nothing is in range, until configured
/// otherwise with `Tf03.setOutOfRangeThreshold`.
pub const default_out_of_range_cm: u16 = 18_000;

/// Below this signal strength the TF03 does not trust its return, and sends
/// the out-of-range distance in place of a measurement.
pub const min_reliable_strength: u16 = 40;

/// How long the TF03 may take to answer a command. The manual gives a command
/// no response within one second as failed.
pub const response_timeout_ms: u32 = 1_000;

// Command IDs.
const command_firmware_version = 0x01;
const command_system_reset = 0x02;
const command_frame_rate = 0x03;
const command_trigger = 0x04;
const command_baud_rate = 0x06;
const command_output_enable = 0x07;
const command_restore_defaults = 0x10;
const command_save_settings = 0x11;
const command_out_of_range = 0x4F;

/// UART baud rates the TF03 accepts.
///
/// The two editions of the manual disagree at the fast end, so only the rates
/// both list are allowed. An unsupported rate is not refused by the TF03; it
/// silently falls back to 115200 and the link is lost.
const supported_baud_rates = [_]u32{
    9_600,   14_400,  19_200,  38_400,  56_000,  57_600,  115_200,   128_000,
    230_400, 256_000, 460_800, 512_000, 750_000, 921_600, 1_000_000,
};

/// Room the ESP-IDF driver keeps for received bytes between reads.
const rx_ring_buffer = 2048;

/// What arrived on the TF03's line while the driver waited for an answer that
/// never came, from least to most of the link proven working.
///
/// The TF03 streams measurements without being asked from the moment it is
/// powered, so listening alone shows whether it is powered and its output
/// reaches the board.
pub const Heard = enum(u8) {
    /// Not one byte. The TF03 has no power, its TxD wire is not on the
    /// board's RX pin, or the two do not share a ground.
    nothing,
    /// Bytes, but no sound packet: the baud rates differ, the ground is
    /// loose, or the RX wire is picking up interference.
    noise,
    /// Only the command just sent, copied back. The board's TX is reaching its
    /// own RX: through a short, or through an RX wire with nothing driving it
    /// picking up the TX wire beside it. Nothing else answered.
    own_command,
    /// Measurements, but no answer. The TF03 is powered and its TxD reaches
    /// the board, but the board's TX does not reach the TF03's RxD wire.
    measurements,

    pub fn name(heard: Heard) []const u8 {
        return switch (heard) {
            .nothing => "nothing",
            .noise => "noise",
            .own_command => "own command",
            .measurements => "measurements",
        };
    }
};

pub const Error = error{
    /// The ESP-IDF UART driver returned an error; see `Tf03.uart_error`.
    Uart,
    /// No measurement arrived within the time asked for.
    Timeout,
    /// The TF03 did not answer a command in time. `Tf03.heard` says what came
    /// over the line instead, which narrows down why.
    NoReply,
    /// The TF03 answered a command with the nonzero status in `Tf03.status`.
    Rejected,
    /// The TF03 answered a command with something other than the manual
    /// describes.
    Protocol,
    /// The TF03 does not support the value asked for.
    Unsupported,
};

/// One distance reading.
pub const Measurement = struct {
    /// Distance to the target in centimetres, or null when nothing was in
    /// range or the return was too weak to trust.
    distance_cm: ?u16,
    /// The distance field as sent, which reads as the out-of-range threshold
    /// when there was no usable return.
    raw_distance_cm: u16,
    /// Strength of the return, from 0 to 3500. Readings between 40 and 1200
    /// are the most reliable; highly reflective targets read above 1500.
    strength: u16,
};

/// The firmware version, which the TF03 reports as `major.minor.patch`.
pub const FirmwareVersion = struct {
    major: u8,
    minor: u8,
    patch: u8,
};

/// Whether `hz` is of the form `n × 10^k` with `n` in 1..=9 and `k` in 0..=3.
fn isSupportedFrameRate(hz: u16) bool {
    for ([_]u16{ 1, 10, 100, 1000 }) |scale| {
        if (hz % scale == 0 and hz / scale >= 1 and hz / scale <= 9) return true;
    }
    return false;
}

fn le16(value: u16) [2]u8 {
    return .{ @truncate(value), @truncate(value >> 8) };
}

/// A TF03 on one of the ESP32's UARTs.
pub const Tf03 = struct {
    port: idf.uart_port_t,
    parser: protocol.Parser = .{},
    rx_buf: [64]u8 = undefined,
    rx_start: usize = 0,
    rx_end: usize = 0,
    out_of_range_cm: u16 = default_out_of_range_cm,
    /// The last command sent, to recognise it if it comes back.
    sent: ?Command = null,
    /// The most telling thing received since the last command was sent.
    heard: Heard = .nothing,
    /// After `error.Rejected`: the status byte the TF03 sent.
    status: u8 = 0,
    /// After `error.Uart`: what the ESP-IDF driver returned.
    uart_error: rtos.EspError = error.Fail,

    pub fn init(port: idf.uart_port_t) Tf03 {
        return .{ .port = port };
    }

    /// Installs the ESP-IDF UART driver on the port and routes it to the
    /// given pins, 8N1 at `baud`. Nothing is sent.
    pub fn begin(lidar: *Tf03, tx: idf.gpio_num_t, rx: idf.gpio_num_t, baud: c_int) rtos.EspError!void {
        try rtos.check(idf.uart_driver_install(lidar.port, rx_ring_buffer, 0, 0, null, 0));
        try rtos.check(idf.ping_uart_config_8n1(lidar.port, baud));
        try rtos.check(idf.uart_set_pin(lidar.port, tx, rx, idf.UART_PIN_NO_CHANGE, idf.UART_PIN_NO_CHANGE));
    }

    /// A human readable description of `err`, as the driver last saw it, for
    /// logs and the display.
    pub fn describe(lidar: *const Tf03, err: Error) []const u8 {
        return switch (err) {
            error.Uart => @errorName(lidar.uart_error),
            error.Timeout => "timeout",
            error.NoReply => switch (lidar.heard) {
                .nothing => "no reply, heard nothing",
                .noise => "no reply, heard noise",
                .own_command => "no reply, heard own command",
                .measurements => "no reply, heard measurements",
            },
            error.Rejected => "rejected",
            error.Protocol => "protocol",
            error.Unsupported => "unsupported",
        };
    }

    /// Tells the driver about an out-of-range threshold configured on the
    /// TF03 earlier, so readings at that distance are reported as nothing. The
    /// TF03 keeps the threshold in flash and has no command to read it back.
    pub fn setKnownOutOfRangeThreshold(lidar: *Tf03, threshold_cm: u16) void {
        lidar.out_of_range_cm = threshold_cm;
    }

    /// Waits up to `timeout_ms` for the next measurement. Any command
    /// responses that arrive in the meantime are dropped.
    pub fn read(lidar: *Tf03, timeout_ms: u32) Error!Measurement {
        const deadline = rtos.nowUs() + @as(i64, timeout_ms) * 1000;
        while (true) {
            switch (try lidar.readPacket(deadline)) {
                .data => |frame| return lidar.measurement(frame),
                .response => {},
            }
        }
    }

    /// Reads the firmware version. Also a cheap way to check the TF03 is there
    /// and both wires work.
    pub fn firmwareVersion(lidar: *Tf03) Error!FirmwareVersion {
        const response = try lidar.transact(.init(command_firmware_version, &.{}), false);
        if (response.payload.len != 3) return error.Protocol;
        const b = response.payload.bytes;
        return .{ .major = b[2], .minor = b[1], .patch = b[0] };
    }

    /// Reboots the TF03. Settings in flash are kept.
    pub fn systemReset(lidar: *Tf03) Error!void {
        try lidar.checkStatus(try lidar.transact(.init(command_system_reset, &.{}), false));
    }

    /// Sets how many measurements per second the TF03 streams.
    ///
    /// The TF03 supports rates of the form `n × 10^k` for `n` from 1 to 9 and
    /// `k` from 0 to 3; Benewake rates it for up to 1000 Hz. Anything else is
    /// refused here, since the TF03 itself would quietly fall back to 100 Hz.
    pub fn setFrameRate(lidar: *Tf03, hz: u16) Error!void {
        if (!isSupportedFrameRate(hz)) return error.Unsupported;
        try lidar.transactEcho(.init(command_frame_rate, &le16(hz)));
    }

    /// Starts or stops the automatic stream of measurements. With output
    /// stopped, the TF03 measures only when `trigger` asks.
    pub fn setOutputEnabled(lidar: *Tf03, enabled: bool) Error!void {
        try lidar.transactEcho(.init(command_output_enable, &.{@intFromBool(enabled)}));
    }

    /// Takes a single measurement. Meant for when output has been stopped;
    /// otherwise it returns whichever streamed measurement arrives next.
    pub fn trigger(lidar: *Tf03) Error!Measurement {
        try lidar.send(.init(command_trigger, &.{}));
        return lidar.read(response_timeout_ms) catch |err| switch (err) {
            error.Timeout => error.NoReply,
            else => err,
        };
    }

    /// Changes the TF03's UART baud rate.
    ///
    /// The host UART has to be reconfigured to match afterwards, and the
    /// manual does not say which of the two rates the TF03 answers this
    /// command at. A no-reply error here therefore does not mean the change
    /// failed: switch the host UART over and confirm with `firmwareVersion`.
    pub fn setBaudRate(lidar: *Tf03, baud: u32) Error!void {
        if (std.mem.indexOfScalar(u32, &supported_baud_rates, baud) == null) return error.Unsupported;
        var payload: [4]u8 = undefined;
        std.mem.writeInt(u32, &payload, baud, .little);
        try lidar.transactEcho(.init(command_baud_rate, &payload));
    }

    /// Sets the distance the TF03 reports when nothing is in range, and which
    /// the driver will then report as nothing.
    pub fn setOutOfRangeThreshold(lidar: *Tf03, threshold_cm: u16) Error!void {
        try lidar.checkStatus(try lidar.transact(.init(command_out_of_range, &le16(threshold_cm)), false));
        lidar.out_of_range_cm = threshold_cm;
    }

    /// Writes the current settings to flash.
    pub fn saveSettings(lidar: *Tf03) Error!void {
        try lidar.checkStatus(try lidar.transact(.init(command_save_settings, &.{}), false));
    }

    /// Restores factory settings, including a baud rate of 115200 and a frame
    /// rate of 100 Hz.
    pub fn restoreDefaults(lidar: *Tf03) Error!void {
        try lidar.checkStatus(try lidar.transact(.init(command_restore_defaults, &.{}), false));
        lidar.out_of_range_cm = default_out_of_range_cm;
    }

    fn measurement(lidar: *const Tf03, frame: protocol.DataFrame) Measurement {
        const usable = frame.distance_cm < lidar.out_of_range_cm and frame.strength >= min_reliable_strength;
        return .{
            .distance_cm = if (usable) frame.distance_cm else null,
            .raw_distance_cm = frame.distance_cm,
            .strength = frame.strength,
        };
    }

    /// Waits for the next packet of either kind until `deadline_us`, noting
    /// in `heard` what it was.
    fn readPacket(lidar: *Tf03, deadline_us: i64) Error!Packet {
        while (true) {
            while (lidar.rx_start < lidar.rx_end) {
                const byte = lidar.rx_buf[lidar.rx_start];
                lidar.rx_start += 1;
                if (lidar.parser.push(byte)) |packet| {
                    const heard: Heard = switch (packet) {
                        .data => .measurements,
                        .response => |response| if (lidar.isOwnCommand(response)) .own_command else .noise,
                    };
                    lidar.noteHeard(heard);
                    return packet;
                }
            }

            const remaining_us = deadline_us - rtos.nowUs();
            if (remaining_us <= 0) return error.Timeout;

            // Wait for one byte, then take whatever else has already arrived,
            // so a read never sits waiting to fill the whole buffer.
            const wait = rtos.ticks(@intCast(@divTrunc(remaining_us, 1000))) + 1;
            var count = idf.uart_read_bytes(lidar.port, &lidar.rx_buf, 1, wait);
            if (count > 0) {
                var buffered: usize = 0;
                _ = idf.uart_get_buffered_data_len(lidar.port, &buffered);
                buffered = @min(buffered, lidar.rx_buf.len - 1);
                if (buffered > 0) {
                    const more = idf.uart_read_bytes(lidar.port, lidar.rx_buf[1..].ptr, @intCast(buffered), 0);
                    count += @max(more, 0);
                }
            }

            if (count < 0) {
                // Bytes were probably lost, so whatever the parser holds would
                // be spliced onto an unrelated later packet.
                lidar.parser.reset();
                lidar.uart_error = error.Fail;
                return error.Uart;
            }
            if (count > 0) lidar.noteHeard(.noise);
            lidar.rx_start = 0;
            lidar.rx_end = @intCast(count);
        }
    }

    fn noteHeard(lidar: *Tf03, heard: Heard) void {
        lidar.heard = @enumFromInt(@max(@intFromEnum(lidar.heard), @intFromEnum(heard)));
    }

    fn isOwnCommand(lidar: *const Tf03, response: Response) bool {
        const sent = lidar.sent orelse return false;
        return response.id == sent.id() and response.payload.eql(sent.payload());
    }

    fn send(lidar: *Tf03, command: Command) Error!void {
        lidar.sent = command;
        lidar.heard = .nothing;
        const bytes = command.bytes();
        if (idf.uart_write_bytes(lidar.port, bytes.ptr, bytes.len) < 0) {
            lidar.uart_error = error.Fail;
            return error.Uart;
        }
        rtos.check(idf.uart_wait_tx_done(lidar.port, rtos.ticks(response_timeout_ms))) catch |err| {
            lidar.uart_error = err;
            return error.Uart;
        };
    }

    /// Sends a command and waits for the TF03's answer to it, skipping data
    /// frames and answers to anything else.
    ///
    /// A copy of the command itself is skipped too unless `echoed` says the
    /// TF03 answers by sending the command back. For those commands an echo on
    /// the wire cannot be told from a real answer.
    fn transact(lidar: *Tf03, command: Command, echoed: bool) Error!Response {
        try lidar.send(command);

        const deadline = rtos.nowUs() + @as(i64, response_timeout_ms) * 1000;
        while (true) {
            const packet = lidar.readPacket(deadline) catch |err| switch (err) {
                error.Timeout => return error.NoReply,
                // A UART error is skipped: the answer may still come through
                // on the reads that follow, and the deadline bounds the wait.
                else => continue,
            };
            switch (packet) {
                .response => |response| if (response.id == command.id() and (echoed or !lidar.isOwnCommand(response))) {
                    return response;
                },
                .data => {},
            }
        }
    }

    /// For commands the TF03 acknowledges by sending the command back.
    fn transactEcho(lidar: *Tf03, command: Command) Error!void {
        const response = try lidar.transact(command, true);
        if (!response.payload.eql(command.payload())) return error.Protocol;
    }

    /// For commands the TF03 acknowledges with a single status byte.
    fn checkStatus(lidar: *Tf03, response: Response) Error!void {
        if (response.payload.len != 1) return error.Protocol;
        lidar.status = response.payload.bytes[0];
        if (lidar.status != 0) return error.Rejected;
    }
};
