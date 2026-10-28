//! A 128 by 64 monochrome OLED with an SH1106 or SSD1306 controller, on I²C,
//! showing lines of text.
//!
//! Drawing, fonts and the controllers' init sequences come from u8g2. This
//! file connects u8g2 to ESP-IDF's `i2c_master` driver and lays text out the
//! way the Rust firmware's `ssd1306` module does: the 8 by 13 pixel font on
//! rows two pages (16 pixels) apart, `rows` lines of `columns` characters.
//! The font covers ISO 8859-1 and text is UTF-8, so `°` and `±` show as
//! themselves.
//!
//! Only the two pages of a row are sent when that row changes: 260 bytes,
//! about 6 ms at 400 kHz.
//!
//! # Example
//!
//! ```zig
//! var lcd: display.Display = .{};
//! rtos.must(lcd.begin());
//! rtos.must(lcd.init());
//! lcd.show(&.{ .text("distance"), .fmt("{d} cm", .{1234}) });
//! ```

const std = @import("std");
const idf = @import("idf");
const u8g2 = @import("u8g2");
const constants = @import("constants.zig");
const rtos = @import("rtos.zig");

const log = std.log.scoped(.display);

/// Characters per line.
pub const columns = 16;
/// Lines of text, each two pages tall.
pub const rows = 4;

/// How long one I²C transaction may take before the driver gives up.
const i2c_timeout_ms = 50;

const display_normal: u8 = 0xA6;
const display_inverted: u8 = 0xA7;

/// One line of text, formatted into a buffer of its own, so a list of them
/// can be handed to `Display.show`.
pub const Line = struct {
    /// A little more than a screen line's worth of bytes; UTF-8 takes up to
    /// two per Latin-1 character, and the screen cuts off what is too wide.
    /// One more for the terminator u8g2 wants.
    buf: [4 * columns + 1]u8 = undefined,
    len: usize = 0,

    /// A plain string, without formatting.
    pub fn text(string: []const u8) Line {
        return fmt("{s}", .{string});
    }

    /// `std.fmt` formatting. Text that does not fit is cut short.
    pub fn fmt(comptime format: []const u8, args: anytype) Line {
        var line: Line = .{};
        var writer: std.Io.Writer = .fixed(line.buf[0 .. line.buf.len - 1]);
        writer.print(format, args) catch {};
        line.len = writer.end;
        return line;
    }

    pub fn slice(line: *const Line) []const u8 {
        return line.buf[0..line.len];
    }

    /// NUL terminated, for u8g2.
    pub fn cstr(line: *Line) [*:0]const u8 {
        line.buf[line.len] = 0;
        return @ptrCast(&line.buf);
    }
};

/// Paces the demos' display updates. The sensors report faster than anyone
/// can read, and each rewrite holds up the task for a few milliseconds.
pub const RefreshTimer = struct {
    period_us: i64,
    next_us: i64 = 0,

    pub fn init(period_ms: u32) RefreshTimer {
        return .{ .period_us = @as(i64, period_ms) * 1000 };
    }

    /// Whether it is time to show the next reading, at most once a period.
    pub fn due(timer: *RefreshTimer) bool {
        const now = rtos.nowUs();
        if (now < timer.next_us) return false;
        timer.next_us = now + timer.period_us;
        return true;
    }
};

/// The I²C addresses that answered a `Display.scan`.
pub const Addresses = std.StaticBitSet(128);

/// The board's display (see `constants.zig` for its pins, address and
/// controller).
pub const Display = struct {
    bus: idf.i2c_master_bus_handle_t = null,
    device: idf.i2c_master_dev_handle_t = null,
    u8g2: u8g2.u8g2_t = std.mem.zeroes(u8g2.u8g2_t),
    ready: bool = false,
    failing: bool = false,
    /// One I²C write as u8g2 builds it: a control byte and up to a page of
    /// data.
    tx: [160]u8 = undefined,
    tx_len: usize = 0,
    /// The first error the I²C driver returned during the current u8g2 call.
    err: idf.esp_err_t = idf.ESP_OK,

    /// Sets up the I²C bus. Nothing is sent.
    pub fn begin(display: *Display) rtos.EspError!void {
        try rtos.check(idf.ping_i2c_new_master_bus(constants.lcd_i2c, constants.lcd_sda, constants.lcd_scl, &display.bus));
        try rtos.check(idf.ping_i2c_add_device(display.bus, constants.lcd_address, constants.lcd_i2c_frequency_hz, &display.device));
    }

    /// Sets the panel up, blanks it and turns it on. Safe to call again at
    /// any time.
    pub fn init(display: *Display) rtos.EspError!void {
        switch (constants.lcd_controller) {
            .sh1106 => u8g2.u8g2_Setup_sh1106_i2c_128x64_noname_f(&display.u8g2, &u8g2.u8g2_cb_r0, byteCb, gpioAndDelayCb),
            .ssd1306 => u8g2.u8g2_Setup_ssd1306_i2c_128x64_noname_f(&display.u8g2, &u8g2.u8g2_cb_r0, byteCb, gpioAndDelayCb),
        }
        display.u8g2.u8x8.user_ptr = display;
        // u8g2 addresses devices with the read/write bit in place.
        display.u8g2.u8x8.i2c_address = constants.lcd_address << 1;

        display.err = idf.ESP_OK;
        u8g2.u8x8_InitDisplay(&display.u8g2.u8x8);
        u8g2.u8x8_SetPowerSave(&display.u8g2.u8x8, 0);
        u8g2.u8g2_SetFont(&display.u8g2, u8g2.u8g2_font_8x13_tf);
        u8g2.u8g2_SetFontPosTop(&display.u8g2);
        display.ready = display.err == idf.ESP_OK;
        try rtos.check(display.err);
        return display.clear();
    }

    /// Blanks the whole screen.
    pub fn clear(display: *Display) rtos.EspError!void {
        display.err = idf.ESP_OK;
        u8g2.u8g2_ClearBuffer(&display.u8g2);
        u8g2.u8g2_SendBuffer(&display.u8g2);
        return rtos.check(display.err);
    }

    /// Swaps light and dark across the whole screen, or swaps them back.
    /// Takes effect straight away and leaves the picture alone.
    pub fn setInverted(display: *Display, inverted: bool) rtos.EspError!void {
        display.err = idf.ESP_OK;
        const u8x8 = &display.u8g2.u8x8;
        _ = u8g2.u8x8_cad_StartTransfer(u8x8);
        _ = u8g2.u8x8_cad_SendCmd(u8x8, if (inverted) display_inverted else display_normal);
        _ = u8g2.u8x8_cad_EndTransfer(u8x8);
        return rtos.check(display.err);
    }

    /// Replaces line `row` with `line`. Rows past the bottom are ignored, and
    /// text past the right edge is cut off.
    pub fn writeLine(display: *Display, row: usize, line: *Line) rtos.EspError!void {
        if (row >= rows) return;
        const top: u8g2.u8g2_uint_t = @intCast(16 * row);

        u8g2.u8g2_SetDrawColor(&display.u8g2, 0);
        u8g2.u8g2_DrawBox(&display.u8g2, 0, top, 128, 16);
        u8g2.u8g2_SetDrawColor(&display.u8g2, 1);
        _ = u8g2.u8g2_DrawUTF8(&display.u8g2, 0, top, line.cstr());

        // Send only the two eight-pixel tile rows this line covers.
        display.err = idf.ESP_OK;
        u8g2.u8g2_UpdateDisplayArea(&display.u8g2, 0, @intCast(2 * row), 16, 2);
        return rtos.check(display.err);
    }

    /// Shows `lines` one per row, blanking the rows below them, and logs
    /// them on one line. A failed display write is logged once and otherwise
    /// ignored, so a loose display never stops a sensor being read.
    pub fn show(display: *Display, lines: []const Line) void {
        var joined: rtos.Text(rows * (4 * columns + 3)) = .{};
        {
            var writer: std.Io.Writer = .fixed(&joined.buf);
            for (lines, 0..) |*line, i| {
                writer.print("{s}{s}", .{ if (i == 0) "" else " | ", line.slice() }) catch {};
            }
            joined.len = writer.end;
        }
        log.info("{s}", .{joined.slice()});

        if (!display.ready) {
            // Try to bring a display back that was unplugged or never
            // answered.
            display.init() catch {
                display.failing = true;
                return;
            };
        }

        for (0..rows) |row| {
            var line: Line = if (row < lines.len) lines[row] else .text("");
            display.writeLine(row, &line) catch |err| {
                if (!display.failing) log.warn("write failed: {s}", .{@errorName(err)});
                display.failing = true;
                display.ready = false;
                return;
            };
        }
        if (display.failing) {
            log.info("display back", .{});
            display.failing = false;
        }
    }

    /// Asks every ordinary I²C address in turn whether anything is there.
    ///
    /// For finding out why the display does not answer: an empty bus points
    /// at power or wiring, and a device somewhere else is usually the display
    /// at another address.
    pub fn scan(display: *Display) Addresses {
        var found: Addresses = .empty;
        var address: u16 = 0x08;
        while (address <= 0x77) : (address += 1) {
            if (idf.i2c_master_probe(display.bus, address, i2c_timeout_ms) == idf.ESP_OK) found.set(address);
        }
        return found;
    }

    /// Whether the display answers at its address.
    pub fn present(display: *Display) bool {
        return idf.i2c_master_probe(display.bus, constants.lcd_address, i2c_timeout_ms) == idf.ESP_OK;
    }

    // ------------------------------------------------------------------
    // u8g2 callbacks
    // ------------------------------------------------------------------

    fn fromU8x8(u8x8: [*c]u8g2.u8x8_t) *Display {
        return @ptrCast(@alignCast(u8x8.*.user_ptr.?));
    }

    fn byteCb(u8x8: [*c]u8g2.u8x8_t, msg: u8, arg_int: u8, arg_ptr: ?*anyopaque) callconv(.c) u8 {
        const display = fromU8x8(u8x8);
        switch (msg) {
            u8g2.U8X8_MSG_BYTE_INIT, u8g2.U8X8_MSG_BYTE_SET_DC => return 1,
            u8g2.U8X8_MSG_BYTE_START_TRANSFER => {
                display.tx_len = 0;
                return 1;
            },
            u8g2.U8X8_MSG_BYTE_SEND => {
                const data: [*]const u8 = @ptrCast(arg_ptr.?);
                for (data[0..arg_int]) |byte| {
                    if (display.tx_len == display.tx.len) break;
                    display.tx[display.tx_len] = byte;
                    display.tx_len += 1;
                }
                return 1;
            },
            u8g2.U8X8_MSG_BYTE_END_TRANSFER => {
                const err = idf.i2c_master_transmit(display.device, &display.tx, display.tx_len, i2c_timeout_ms);
                if (display.err == idf.ESP_OK) display.err = err;
                return 1;
            },
            else => return 0,
        }
    }

    fn gpioAndDelayCb(_: [*c]u8g2.u8x8_t, msg: u8, arg_int: u8, _: ?*anyopaque) callconv(.c) u8 {
        if (msg == u8g2.U8X8_MSG_DELAY_MILLI) rtos.delayMs(@as(u32, arg_int) + 1);
        // No reset or chip select pins on an I²C module; nothing else to do.
        return 1;
    }
};
