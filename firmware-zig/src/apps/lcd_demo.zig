//! Standalone demo for the OLED display, which checks its wiring step by
//! step.
//!
//! 1. Idle levels: with the internal pull-downs on, SDA and SCL read high only
//!    if a powered module's pull-ups are on them.
//! 2. An I²C scan, which tells a display at the wrong address from no display
//!    at all.
//! 3. Once the display answers, a count, the character set and the screen
//!    inverted every other second, so a working panel is plain to see.
//!
//! Steps 2 and 3 repeat, so changes to the wiring show up live.

const std = @import("std");
const idf = @import("idf");
const ping = @import("ping");

const constants = ping.constants;
const display = ping.display;
const gpio = ping.gpio;
const rtos = ping.rtos;

pub const std_options: std.Options = .{ .logFn = rtos.logFn, .log_level = .info };
pub const panic = rtos.panic;

const log = std.log.scoped(.lcd);

var lcd: display.Display = .{};

const Wire = struct { name: []const u8, pin: idf.gpio_num_t };
const wires = [_]Wire{
    .{ .name = "SDA (GPIO1)", .pin = constants.lcd_sda },
    .{ .name = "SCL (GPIO2)", .pin = constants.lcd_scl },
};

/// Step 1. The internal pull-downs (about 45 kΩ) lose to the module's
/// pull-ups (a few kΩ), so a line reads high only if a powered module is on
/// it.
fn checkIdleLevels() void {
    for (wires) |wire| gpio.input(wire.pin, .down);
    rtos.delayMs(10);
    for (wires) |wire| {
        if (gpio.read(wire.pin) != 0) {
            log.info("{s} idles high: pulled up, good", .{wire.name});
        } else {
            log.warn("{s} idles LOW: check the display's VCC and GND, this wire, and that the module has pull-up resistors", .{wire.name});
        }
    }
    for (wires) |wire| gpio.release(wire.pin);
}

/// The addresses in `found`, as `0x3c, 0x3d`, or `none`.
fn list(found: display.Addresses) rtos.Text(128 * 6) {
    var text: rtos.Text(128 * 6) = .{};
    var writer: std.Io.Writer = .fixed(&text.buf);
    var addresses = found.iterator(.{});
    var first = true;
    while (addresses.next()) |address| {
        writer.print("{s}0x{x:0>2}", .{ if (first) "" else ", ", address }) catch {};
        first = false;
    }
    if (first) writer.print("none", .{}) catch {};
    text.len = writer.end;
    return text;
}

export fn app_main() void {
    checkIdleLevels();

    rtos.must(lcd.begin());

    var ready = false;
    var count: u32 = 0;
    while (true) {
        if (!lcd.present()) {
            ready = false;
            const found = lcd.scan();
            if (found.count() == 0) {
                log.warn("I2C scan: nothing answers. Check VCC to 3V3, GND, SDA to GPIO1 (J3-4) and SCL to GPIO2 (J3-5); swapped SDA and SCL is the usual mistake", .{});
            } else {
                log.warn("I2C scan: nothing at 0x{x:0>2}, but found {s}. Set lcd_address in constants.zig to the display's address (0x3C or 0x3D)", .{ constants.lcd_address, list(found).slice() });
            }
        } else {
            if (!ready) {
                log.info("I2C scan: display answers at 0x{x:0>2}", .{constants.lcd_address});
                if (lcd.init()) {
                    ready = true;
                } else |err| {
                    log.warn("display init failed: {s}", .{@errorName(err)});
                }
            }

            if (ready) {
                count += 1;
                const inverted = count % 2 == 0;
                var lines = [display.rows]display.Line{
                    .text("PING display"),
                    .fmt("count {d}", .{count}),
                    .text("0123456789ABCDEF"),
                    .text("12.5° ±0.1 m"),
                };
                const written = write(inverted, &lines);
                if (written) {
                    log.info("wrote count {d}, {s}", .{ count, if (inverted) "inverted" else "normal" });
                } else |err| {
                    log.warn("display write failed: {s}", .{@errorName(err)});
                    ready = false;
                }
            }
        }

        rtos.delayMs(1000);
    }
}

fn write(inverted: bool, lines: []display.Line) rtos.EspError!void {
    try lcd.setInverted(inverted);
    for (lines, 0..) |*line, row| try lcd.writeLine(row, line);
}
