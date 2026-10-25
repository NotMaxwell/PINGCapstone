//! Standalone demo for the TF03 LiDAR.
//!
//! Asks the TF03 for its firmware version, for the log, then shows each
//! distance and signal strength on the display and in the log, five times a
//! second. When nothing arrives it says what to check. Kept separate from
//! main.zig so the application entry point stays clear of sensor bring-up
//! code. `docs/tf03-test-plan.md` walks through using it on the bench.

const std = @import("std");
const ping = @import("ping");

const constants = ping.constants;
const display = ping.display;
const rtos = ping.rtos;
const tf03 = ping.tf03;

pub const std_options: std.Options = .{ .logFn = rtos.logFn, .log_level = .info };
pub const panic = rtos.panic;

const log = std.log.scoped(.lidar);

var lcd: display.Display = .{};
var lidar: tf03.Tf03 = .init(constants.lidar_uart);

export fn app_main() void {
    rtos.must(lcd.begin());
    rtos.must(lidar.begin(constants.lidar_tx, constants.lidar_rx, constants.lidar_baud_rate));

    lcd.show(&.{ .text("TF03"), .text("starting") });

    // The TF03 streams measurements unasked, so distances need only its TxD
    // wire. Answering a command also needs its RxD wire, so the firmware
    // version is asked for once, for the log, and its answer waited for no
    // further.
    if (lidar.firmwareVersion()) |version| {
        log.info("TF03 firmware {d}.{d}.{d}", .{ version.major, version.minor, version.patch });
    } else |err| {
        log.warn("TF03 did not answer the firmware version request: {s}. Reading distances anyway.", .{lidar.describe(err)});
    }

    var refresh: display.RefreshTimer = .init(constants.lcd_refresh_ms);
    while (true) {
        const reading = lidar.read(constants.lidar_silence_ms) catch |err| {
            if (err == error.Timeout) {
                lcd.show(&.{ .text("TF03 no data"), .text("check power,"), .text("brown wire, GND") });
            } else {
                lcd.show(&.{ .text("TF03 read failed"), .text(lidar.describe(err)) });
            }
            continue;
        };
        if (!refresh.due()) continue;

        if (reading.distance_cm) |distance_cm| {
            lcd.show(&.{
                .fmt("dist {d:>9.2} m", .{@as(f32, @floatFromInt(distance_cm)) / 100.0}),
                .fmt("strength {d:>7}", .{reading.strength}),
            });
        } else {
            lcd.show(&.{
                .text("out of range"),
                .fmt("raw {d} cm", .{reading.raw_distance_cm}),
                .fmt("strength {d:>7}", .{reading.strength}),
            });
        }
    }
}
