//! The CEVA/Hillcrest BNO085 nine axis sensor hub, over SPI.
//!
//! The SH-2 protocol is handled by CEVA's own reference driver, vendored in
//! `components/sh2`. It is platform independent and reaches the hub through an
//! `sh2_Hal_t`: open, close, read, write and a microsecond clock. This file
//! implements that HAL on ESP-IDF's SPI master and GPIO drivers, brings the
//! hub up on the board's pins and hands out what the application uses from
//! each rotation vector report.
//!
//! # The SPI link
//!
//! SHTP, the hub's transport, starts every transfer with a four byte header
//! that gives the transfer's length. A read therefore clocks the header
//! first and then the rest, with chip select held low across both. Chip
//! select is a GPIO rather than the SPI peripheral's own so that it can be
//! held like that.
//!
//! The hub's INT line (active low) says it has something to send, or is ready
//! to receive. Before a write the host asserts WAKE (PS0 low), waits for INT,
//! writes, and releases WAKE. SPI is full duplex, so whatever the hub was
//! sending during the write is kept and handed to the driver as its next
//! read; no report is lost to a write.
//!
//! INT is polled rather than wired to an interrupt. The tick is 1 ms and the
//! hub reports every 10 ms, so polling costs nothing noticeable and keeps the
//! GPIO interrupt service free.
//!
//! # SPI mode
//!
//! The hub selects SPI when PS0 and PS1 are both high as it leaves reset. PS1
//! is tied high on the board; PS0 is the breakout's P0, wired to
//! `constants.imu_wake`, which this holds high through every reset.
//!
//! CEVA's driver keeps its state in statics, so there is one hub per program.

const std = @import("std");
const idf = @import("idf");
const sh2 = @import("sh2");
const constants = @import("constants.zig");
const geo = @import("geo.zig");
const gpio = @import("gpio.zig");
const rtos = @import("rtos.zig");

const log = std.log.scoped(.imu);

/// How far the hub trusts the calibration behind a report.
pub const Accuracy = enum(u8) {
    unreliable = 0,
    low = 1,
    medium = 2,
    high = 3,

    /// A short name, for logs and the display.
    pub fn name(accuracy: Accuracy) []const u8 {
        return switch (accuracy) {
            .unreliable => "bad",
            .low => "low",
            .medium => "med",
            .high => "high",
        };
    }
};

/// What the application keeps from each rotation vector report.
pub const Orientation = struct {
    quaternion: geo.Quaternion,
    /// The hub's estimate of its heading error, radians.
    heading_accuracy_rad: f32,
    /// How far the hub trusts its magnetometer calibration.
    calibration: Accuracy,
};

/// What the hub says about itself: firmware version, part number, why it
/// last reset.
pub const ProductId = sh2.sh2_ProductId_t;

/// SPI mode 3 (clock idles high, data on the rising edge), per the datasheet.
const spi_mode: u8 = 3;

/// How long the reset line is held low.
const reset_pulse_ms: u32 = 10;

/// How long the hub may take to assert INT: after a reset (it boots in well
/// under this) or after WAKE is asserted.
const int_timeout_ms: u32 = 500;

/// Length of an SHTP header.
const header_len = 4;

const rotation_vector: sh2.sh2_SensorId_t = sh2.SH2_ROTATION_VECTOR;

/// A BNO085 on the board's IMU pins (see `constants.zig`).
pub const Imu = struct {
    hal: sh2.sh2_Hal_t = .{
        .open = halOpen,
        .close = halClose,
        .read = halRead,
        .write = halWrite,
        .getTimeUs = halTimeUs,
    },
    device: idf.spi_device_handle_t = null,
    /// The pins and the SPI bus are set up.
    bus_ready: bool = false,
    /// `sh2_open` succeeded and has not been undone.
    open: bool = false,
    interval_us: u32 = 0,
    /// Identification the hub sent while starting.
    product_ids: sh2.sh2_ProductIds_t = std.mem.zeroes(sh2.sh2_ProductIds_t),

    /// What the hub sent during the last write, waiting to be read.
    pending: [sh2.SH2_HAL_MAX_TRANSFER_IN]u8 = undefined,
    pending_len: usize = 0,
    pending_at_us: u32 = 0,

    latest: ?Orientation = null,
    /// `latest` has not been handed out yet.
    fresh: bool = false,
    /// The hub reported a reset since reports were enabled.
    reset_seen: bool = false,

    // ------------------------------------------------------------------
    // Application interface
    // ------------------------------------------------------------------

    /// Resets the hub and starts its rotation vector every `interval_us`
    /// microseconds. Safe to call again to start over.
    pub fn start(imu: *Imu, interval_us: u32) bool {
        if (!imu.bus_ready) {
            imu.setUpBus() catch |err| {
                log.warn("SPI setup failed: {s}", .{@errorName(err)});
                return false;
            };
            imu.bus_ready = true;
        }
        if (imu.open) {
            sh2.sh2_close();
            imu.open = false;
        }
        imu.interval_us = interval_us;
        imu.latest = null;
        imu.fresh = false;

        // sh2_open resets the hub (through halOpen) and waits for it to
        // announce itself.
        const status = sh2.sh2_open(&imu.hal, onEvent, imu);
        if (status != sh2.SH2_OK) {
            log.warn("sh2_open failed: {d}", .{status});
            return false;
        }
        imu.open = true;
        imu.reset_seen = false;

        if (sh2.sh2_getProdIds(&imu.product_ids) != sh2.SH2_OK) {
            log.warn("the hub did not answer the product ID request", .{});
            return false;
        }
        return imu.enableRotationVector();
    }

    /// Identification the hub sent while starting, valid after `start`.
    pub fn productId(imu: *const Imu) ProductId {
        // The first entry describes the SH-2 firmware itself.
        return if (imu.product_ids.numEntries > 0) imu.product_ids.entry[0] else std.mem.zeroes(ProductId);
    }

    /// Waits up to `timeout_ms` for the next rotation vector report.
    pub fn wait(imu: *Imu, timeout_ms: u32) ?Orientation {
        if (!imu.open) return null;
        const deadline = rtos.nowMs() + timeout_ms;
        while (true) {
            // Reads what the hub has and runs the callbacks below. When the
            // hub has nothing, halRead waits a tick, so this does not spin.
            sh2.sh2_service();
            if (imu.reset_seen) {
                // A reset stops every report; ask for them again.
                imu.reset_seen = false;
                log.warn("the hub reset itself; enabling reports again", .{});
                _ = imu.enableRotationVector();
            }
            if (imu.fresh) {
                imu.fresh = false;
                return imu.latest;
            }
            if (rtos.nowMs() >= deadline) return null;
        }
    }

    fn setUpBus(imu: *Imu) rtos.EspError!void {
        // The hub is held in reset until sh2_open releases it.
        gpio.output(constants.imu_reset, 0);
        gpio.output(constants.imu_wake, 1);
        gpio.output(constants.imu_cs, 1);
        gpio.input(constants.imu_int, .up);
        try rtos.check(idf.ping_spi_bus_initialize(constants.imu_spi, constants.imu_mosi, constants.imu_miso, constants.imu_sck));
        try rtos.check(idf.ping_spi_add_device(constants.imu_spi, constants.imu_spi_frequency_hz, spi_mode, &imu.device));
    }

    fn enableRotationVector(imu: *Imu) bool {
        var config = std.mem.zeroes(sh2.sh2_SensorConfig_t);
        config.reportInterval_us = imu.interval_us;
        const status = sh2.sh2_setSensorConfig(rotation_vector, &config);
        if (status != sh2.SH2_OK) log.warn("enabling the rotation vector failed: {d}", .{status});
        return status == sh2.SH2_OK;
    }

    // ------------------------------------------------------------------
    // Callbacks from CEVA's driver
    // ------------------------------------------------------------------

    fn onEvent(cookie: ?*anyopaque, event: [*c]sh2.sh2_AsyncEvent_t) callconv(.c) void {
        const imu: *Imu = @ptrCast(@alignCast(cookie.?));
        if (event.*.eventId == sh2.SH2_RESET) imu.reset_seen = true;
    }

    fn onSensorEvent(cookie: ?*anyopaque, event: [*c]sh2.sh2_SensorEvent_t) callconv(.c) void {
        const imu: *Imu = @ptrCast(@alignCast(cookie.?));
        var value: sh2.sh2_SensorValue_t = undefined;
        if (sh2.sh2_decodeSensorEvent(&value, event) != sh2.SH2_OK) return;
        if (value.sensorId != rotation_vector) return;
        const rv = value.un.rotationVector;
        imu.latest = .{
            .quaternion = .{ .i = rv.i, .j = rv.j, .k = rv.k, .real = rv.real },
            .heading_accuracy_rad = rv.accuracy,
            .calibration = @enumFromInt(value.status & 0x03),
        };
        imu.fresh = true;
    }

    // ------------------------------------------------------------------
    // The HAL
    // ------------------------------------------------------------------

    fn fromHal(self: [*c]sh2.sh2_Hal_t) *Imu {
        const hal: *sh2.sh2_Hal_t = self;
        return @fieldParentPtr("hal", hal);
    }

    fn intAsserted() bool {
        return gpio.read(constants.imu_int) == 0;
    }

    /// Waits up to `timeout_ms` for INT to go low.
    fn waitForInt(timeout_ms: u32) bool {
        const deadline = rtos.nowMs() + timeout_ms;
        while (!intAsserted()) {
            if (rtos.nowMs() >= deadline) return false;
            rtos.delayMs(1);
        }
        return true;
    }

    fn transfer(imu: *Imu, tx: ?[*]const u8, rx: ?[*]u8, len: usize) rtos.EspError!void {
        try rtos.check(idf.ping_spi_transfer(imu.device, tx, rx, len));
    }

    /// The length field of an SHTP header, without its continuation bit.
    fn transferLen(header: []const u8) usize {
        return (@as(usize, header[0]) | @as(usize, header[1]) << 8) & 0x7FFF;
    }

    /// Resets the hub, with PS0 high so it comes up in SPI mode, and waits
    /// for it to assert INT with its first message.
    fn halOpen(self: [*c]sh2.sh2_Hal_t) callconv(.c) c_int {
        const imu = fromHal(self);
        imu.pending_len = 0;
        _ = sh2.sh2_setSensorCallback(onSensorEvent, imu);

        gpio.write(constants.imu_cs, 1);
        gpio.write(constants.imu_wake, 1);
        gpio.write(constants.imu_reset, 0);
        rtos.delayMs(reset_pulse_ms);
        gpio.write(constants.imu_reset, 1);

        if (!waitForInt(int_timeout_ms)) {
            log.warn("the hub did not assert INT after a reset", .{});
            return sh2.SH2_ERR_IO;
        }
        return sh2.SH2_OK;
    }

    /// Holds the hub in reset.
    fn halClose(self: [*c]sh2.sh2_Hal_t) callconv(.c) void {
        _ = fromHal(self);
        gpio.write(constants.imu_reset, 0);
    }

    /// One SHTP transfer from the hub into `buffer`, or 0 if it has nothing
    /// to send. Waits at most one tick, so the driver's service loop does
    /// not spin.
    fn halRead(self: [*c]sh2.sh2_Hal_t, buffer: [*c]u8, len: c_uint, t_us: [*c]u32) callconv(.c) c_int {
        const imu = fromHal(self);
        const out = buffer[0..len];

        if (imu.pending_len > 0) {
            // Received during a write; see halWrite.
            const count = imu.pending_len;
            imu.pending_len = 0;
            if (count > out.len) return 0;
            @memcpy(out[0..count], imu.pending[0..count]);
            t_us.* = imu.pending_at_us;
            return @intCast(count);
        }

        if (!intAsserted()) {
            rtos.delayMs(1);
            if (!intAsserted()) return 0;
        }
        if (out.len < header_len) return sh2.SH2_ERR_BAD_PARAM;
        const at_us = halTimeUs(self);

        gpio.write(constants.imu_cs, 0);
        defer gpio.write(constants.imu_cs, 1);

        imu.transfer(null, out.ptr, header_len) catch return sh2.SH2_ERR_IO;
        const count = transferLen(out[0..header_len]);
        // Nothing to send reads as a zero length; a missing hub (MISO
        // floating high) as 0x7FFF, longer than any buffer.
        if (count < header_len or count > out.len) return 0;
        imu.transfer(null, out.ptr + header_len, count - header_len) catch return sh2.SH2_ERR_IO;

        t_us.* = at_us;
        return @intCast(count);
    }

    /// One SHTP transfer to the hub. Wakes it first, and keeps whatever it
    /// sent back at the same time for the next read.
    fn halWrite(self: [*c]sh2.sh2_Hal_t, buffer: [*c]u8, len: c_uint) callconv(.c) c_int {
        const imu = fromHal(self);
        if (len > imu.pending.len) return sh2.SH2_ERR_BAD_PARAM;

        gpio.write(constants.imu_wake, 0);
        defer gpio.write(constants.imu_wake, 1);
        if (!waitForInt(int_timeout_ms)) {
            log.warn("the hub did not wake for a write", .{});
            return sh2.SH2_ERR_IO;
        }
        const at_us = halTimeUs(self);

        gpio.write(constants.imu_cs, 0);
        defer gpio.write(constants.imu_cs, 1);

        imu.transfer(buffer, &imu.pending, len) catch return sh2.SH2_ERR_IO;
        const count = transferLen(imu.pending[0..header_len]);
        if (count >= header_len and count <= imu.pending.len) {
            // The hub was sending a message of its own, and still is if it
            // is longer than what was written.
            if (count > len) {
                imu.transfer(null, imu.pending[len..].ptr, count - len) catch return sh2.SH2_ERR_IO;
            }
            imu.pending_len = count;
            imu.pending_at_us = at_us;
        }
        return @intCast(len);
    }

    fn halTimeUs(_: [*c]sh2.sh2_Hal_t) callconv(.c) u32 {
        return @truncate(@as(u64, @intCast(rtos.nowUs())));
    }
};
