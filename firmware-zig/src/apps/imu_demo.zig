//! Standalone demo for the BNO085 IMU.
//!
//! Brings up the sensor hub over SPI, starts the rotation vector at 100 Hz and
//! shows the orientation as yaw, pitch and roll on the display and in the
//! log, five times a second. Kept separate from main.zig so the application
//! entry point stays clear of sensor bring-up code.
//!
//! The angles are those of the breakout itself, with its printed +Y axis as
//! forward; see `geo.toEuler`. Yaw is from magnetic north.

const std = @import("std");
const ping = @import("ping");

const constants = ping.constants;
const display = ping.display;
const geo = ping.geo;
const imu = ping.imu;
const rtos = ping.rtos;

pub const std_options: std.Options = .{ .logFn = rtos.logFn, .log_level = .info };
pub const panic = rtos.panic;

/// How long to wait for one report before trying again.
const wait_ms = 100;

var lcd: display.Display = .{};
var hub: imu.Imu = .{};

fn sign(value: f32) []const u8 {
    return if (value < 0) "-" else "+";
}

export fn app_main() void {
    rtos.must(lcd.begin());

    lcd.show(&.{ .text("BNO085"), .text("starting") });
    while (!hub.start(constants.imu_report_interval_us)) {
        lcd.show(&.{ .text("BNO085 no reply"), .text("retrying") });
        rtos.delayMs(constants.retry_delay_ms);
    }
    const id = hub.productId();
    lcd.show(&.{
        .text("BNO085 up"),
        .fmt("fw {d}.{d}.{d}", .{ id.swVersionMajor, id.swVersionMinor, id.swVersionPatch }),
        .fmt("reset cause {d}", .{id.resetCause}),
    });

    var refresh: display.RefreshTimer = .init(constants.lcd_refresh_ms);
    while (true) {
        const orientation = hub.wait(wait_ms) orelse continue;
        if (!refresh.due()) continue;

        const angles = geo.toEuler(orientation.quaternion);
        const accuracy_deg = std.math.radiansToDegrees(orientation.heading_accuracy_rad);
        lcd.show(&.{
            .fmt("yaw   {d:>7.1}°", .{angles.yaw_deg}),
            .fmt("pitch {s}{d:>6.1}°", .{ sign(angles.pitch_deg), @abs(angles.pitch_deg) }),
            .fmt("roll  {s}{d:>6.1}°", .{ sign(angles.roll_deg), @abs(angles.roll_deg) }),
            .fmt("acc {s} ±{d:.1}°", .{ orientation.calibration.name(), accuracy_deg }),
        });
    }
}
