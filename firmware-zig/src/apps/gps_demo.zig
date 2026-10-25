//! Standalone demo for the NEO-M9N GPS.
//!
//! Asks the receiver what it is, configures it for UBX at
//! `constants.gps_rate_hz` and shows the position, its accuracy and the
//! satellite count on the display and in the log, five times a second. Kept
//! separate from main.zig so the application entry point stays clear of
//! sensor bring-up code.

const std = @import("std");
const ping = @import("ping");

const constants = ping.constants;
const display = ping.display;
const gps = ping.gps;
const rtos = ping.rtos;

pub const std_options: std.Options = .{ .logFn = rtos.logFn, .log_level = .info };
pub const panic = rtos.panic;

var lcd: display.Display = .{};
var receiver: gps.Gps = .init(constants.gps_uart);

fn sign(value: f64) []const u8 {
    return if (value < 0) "-" else "+";
}

export fn app_main() void {
    rtos.must(lcd.begin());
    rtos.must(receiver.begin(constants.gps_tx, constants.gps_rx, constants.gps_baud_rate));

    lcd.show(&.{ .text("GPS"), .text("starting") });
    while (true) {
        if (receiver.connect()) |version| {
            lcd.show(&.{
                .text("GPS up"),
                .text(version.module()),
                .fmt("fw {s} {d}.{d:0>2}", .{ version.firmwareType(), version.firmware_major, version.firmware_minor }),
                .fmt("protocol {d}.{d:0>2}", .{ version.protocol_major, version.protocol_minor }),
            });
            break;
        }
        lcd.show(&.{ .text("GPS no reply"), .text("retrying") });
        rtos.delayMs(constants.retry_delay_ms);
    }

    while (!receiver.configure(constants.gps_rate_hz)) {
        lcd.show(&.{ .text("GPS refused"), .text("config"), .text("retrying") });
        rtos.delayMs(constants.retry_delay_ms);
    }

    var refresh: display.RefreshTimer = .init(constants.lcd_refresh_ms);
    while (true) {
        const solution = receiver.poll() orelse {
            rtos.delayMs(10);
            continue;
        };
        if (!refresh.due()) continue;

        if (solution.hasPosition()) {
            lcd.show(&.{
                .fmt("lat {s}{d:.7}", .{ sign(solution.latitude_deg), @abs(solution.latitude_deg) }),
                .fmt("lon {s}{d:.7}", .{ sign(solution.longitude_deg), @abs(solution.longitude_deg) }),
                .fmt("acc ±{d:.1} m", .{solution.horizontal_accuracy_m}),
                .fmt("{d} satellites", .{solution.satellites}),
            });
        } else {
            lcd.show(&.{ .text("no fix"), .text(solution.fix_type.name()), .fmt("{d} satellites", .{solution.satellites}) });
        }
    }
}
