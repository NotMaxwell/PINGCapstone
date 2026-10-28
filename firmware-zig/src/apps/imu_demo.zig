//! Standalone demo for the BNO085 IMU.
//!
//! Brings up the sensor hub over SPI, starts the rotation vector at 100 Hz and
//! shows the compass heading on the display and in the log, five times a
//! second. Kept separate from main.zig so the application entry point stays
//! clear of sensor bring-up code.
//!
//! The heading is the direction `constants.lidar_boresight` points, worked
//! out the same way the application aims, by `geo.Aim.fromOrientation`. It is
//! shown from true north, using `constants.magnetic_declination_deg`, with the
//! nearest of the 16 compass points, and again from magnetic north. The hub
//! fuses the accelerometer with the magnetometer, so the heading holds while
//! the device is tilted. Pitch and roll are those of the breakout itself; see
//! `geo.toEuler`.

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

/// The nearest of the 16 points of the compass to `bearing_deg`, degrees
/// clockwise from north, from 0 up to 360.
fn compassPoint(bearing_deg: f32) []const u8 {
    const points = [_][]const u8{
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",
    };
    // Each point covers 22.5°, centred on its own bearing. The float to int
    // cast saturates, so a bearing a hair below zero still lands on north.
    const sector: usize = @intFromFloat(@max(0.0, bearing_deg / 22.5 + 0.5));
    return points[sector % points.len];
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

        const q = orientation.quaternion;
        const heading = geo.Aim.fromOrientation(q, constants.lidar_boresight, constants.magnetic_declination_deg).azimuth_deg;
        const magnetic = geo.Aim.fromOrientation(q, constants.lidar_boresight, 0.0).azimuth_deg;
        const angles = geo.toEuler(q);
        const accuracy_deg = std.math.radiansToDegrees(orientation.heading_accuracy_rad);
        lcd.show(&.{
            .fmt("hdg {d:>6.1}° {s}", .{ heading, compassPoint(heading) }),
            .fmt("mag {d:>6.1}°", .{magnetic}),
            .fmt("p{s}{d:>5.1} r{s}{d:>6.1}", .{ sign(angles.pitch_deg), @abs(angles.pitch_deg), sign(angles.roll_deg), @abs(angles.roll_deg) }),
            .fmt("acc {s} ±{d:.1}°", .{ orientation.calibration.name(), accuracy_deg }),
        });
    }
}
