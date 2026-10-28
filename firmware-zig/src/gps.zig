//! The u-blox NEO-M9N GNSS receiver (as on the SparkFun GPS-15733 breakout),
//! over UART.
//!
//! UBX framing and checksums are u-blox's own code (ubxlib, vendored in
//! `components/ubxlib_ubx`), and `ubx.zig` lays the messages out on top of
//! it. This file runs the UART: it sends the polls and the configuration,
//! waits for their answers and pulls navigation solutions out of the stream.
//!
//! # Configuration
//!
//! `Gps.configure` turns NMEA off on UART1, asks for one `UBX-NAV-PVT` per
//! navigation solution and sets the rate, all in the receiver's RAM layer
//! only. Nothing is left behind in its flash, so a fresh board and a reused
//! one behave the same after a power cycle; the price is that a power glitch
//! puts the receiver back to factory NMEA output, which is why the
//! application configures it again whenever it goes quiet.

const std = @import("std");
const idf = @import("idf");
const rtos = @import("rtos.zig");
const ubx = @import("ubx.zig");

pub const FixType = ubx.FixType;
pub const Solution = ubx.Solution;
pub const UtcTime = ubx.UtcTime;
pub const Version = ubx.Version;

/// The fastest navigation rate the NEO-M9N supports.
pub const max_nav_rate_hz: u8 = 25;

/// Room the ESP-IDF driver keeps for received bytes between reads: a second
/// of NAV-PVTs at the fastest rate, and then some.
const rx_ring_buffer = 4096;

/// How long the receiver may take to answer a poll or acknowledge a
/// configuration. u-blox answers within a navigation epoch; one second covers
/// the slowest rate this configures.
const response_timeout_ms: u32 = 1_000;

/// A NEO-M9N on one of the ESP32's UARTs.
pub const Gps = struct {
    port: idf.uart_port_t,
    decoder: ubx.Decoder = .{},

    pub fn init(port: idf.uart_port_t) Gps {
        return .{ .port = port };
    }

    /// Installs the ESP-IDF UART driver on the port and routes it to the
    /// given pins, 8N1 at `baud`. Nothing is sent.
    pub fn begin(gps: *Gps, tx: idf.gpio_num_t, rx: idf.gpio_num_t, baud: c_int) rtos.EspError!void {
        try rtos.check(idf.uart_driver_install(gps.port, rx_ring_buffer, 0, 0, null, 0));
        try rtos.check(idf.ping_uart_config_8n1(gps.port, baud));
        // uart_set_pin is a variadic macro translate-c cannot follow; this is
        // the four-pin function it picks.
        try rtos.check(idf._uart_set_pin4(gps.port, tx, rx, idf.UART_PIN_NO_CHANGE, idf.UART_PIN_NO_CHANGE));
    }

    /// Asks the receiver what it is and waits for the answer. Also a cheap
    /// way to check it is there and the baud rate matches.
    pub fn connect(gps: *Gps) ?Version {
        gps.send(ubx.mon_ver, &.{}) catch return null;
        const deadline = rtos.nowUs() + @as(i64, response_timeout_ms) * 1000;
        while (gps.nextMessage(deadline)) |message| {
            if (message.id.eql(ubx.mon_ver)) return ubx.parseMonVer(message.body);
        }
        return null;
    }

    /// Switches UART1 to UBX-only output and sets the receiver to send one
    /// `UBX-NAV-PVT` per solution, `rate_hz` solutions a second. `rate_hz`
    /// must be between 1 and `max_nav_rate_hz`.
    pub fn configure(gps: *Gps, rate_hz: u8) bool {
        if (rate_hz < 1 or rate_hz > max_nav_rate_hz) return false;
        const valset = ubx.boardConfig(rate_hz);
        gps.send(ubx.cfg_valset, valset.slice()) catch return false;
        const deadline = rtos.nowUs() + @as(i64, response_timeout_ms) * 1000;
        while (gps.nextMessage(deadline)) |message| {
            if (ubx.ackFor(message, ubx.cfg_valset)) |acknowledged| return acknowledged;
        }
        return false;
    }

    /// The newest navigation solution that has arrived since the last call,
    /// without waiting.
    ///
    /// Solutions arrive once per navigation epoch whether or not there is a
    /// fix; check `Solution.hasPosition` before using the position.
    pub fn poll(gps: *Gps) ?Solution {
        gps.receive(0);
        var latest: ?Solution = null;
        while (gps.decoder.next()) |message| {
            if (message.id.eql(ubx.nav_pvt)) {
                if (ubx.decodeNavPvt(message.body)) |solution| latest = solution;
            }
        }
        return latest;
    }

    fn send(gps: *Gps, id: ubx.MessageId, body: []const u8) rtos.EspError!void {
        var frame: [ubx.overhead + @sizeOf(ubx.Valset)]u8 = undefined;
        const bytes = ubx.encode(id, body, &frame);
        if (idf.uart_write_bytes(gps.port, bytes.ptr, bytes.len) < 0) return error.Fail;
        try rtos.check(idf.uart_wait_tx_done(gps.port, rtos.ticks(response_timeout_ms)));
    }

    /// Moves what the UART has received into the decoder, waiting up to
    /// `wait` ticks for the first byte.
    fn receive(gps: *Gps, wait: idf.TickType_t) void {
        var room = gps.decoder.unused();
        if (room.len == 0) {
            // Full of something that never became a message.
            gps.decoder.reset();
            room = gps.decoder.unused();
        }
        // One byte with the wait, then whatever else has arrived at once:
        // uart_read_bytes would otherwise wait for the whole length.
        var count = idf.uart_read_bytes(gps.port, room.ptr, 1, wait);
        if (count > 0 and room.len > 1) {
            var buffered: usize = 0;
            _ = idf.uart_get_buffered_data_len(gps.port, &buffered);
            buffered = @min(buffered, room.len - 1);
            if (buffered > 0) {
                const more = idf.uart_read_bytes(gps.port, room[1..].ptr, @intCast(buffered), 0);
                count += @max(more, 0);
            }
        }
        if (count > 0) gps.decoder.commit(@intCast(count));
    }

    /// The next message to arrive before `deadline_us`.
    fn nextMessage(gps: *Gps, deadline_us: i64) ?ubx.Message {
        while (true) {
            if (gps.decoder.next()) |message| return message;
            const remaining_us = deadline_us - rtos.nowUs();
            if (remaining_us <= 0) return null;
            gps.receive(rtos.ticks(@intCast(@divTrunc(remaining_us, 1000))) + 1);
        }
    }
};
