//! PING: aim the device at something, press the button, and it reports that
//! thing's GPS coordinate.
//!
//! Each sensor runs in its own FreeRTOS task and keeps its latest reading in a
//! shared slot:
//!
//! * the NEO-M9N GPS gives the device's own position,
//! * the BNO085 IMU gives its orientation, and from that the direction the
//!   LiDAR points: a compass bearing and an elevation angle,
//! * the TF03 LiDAR gives the distance to whatever it points at.
//!
//! When the button is pressed, the main task takes the latest reading from
//! each and walks the LiDAR's range out from the GPS position along that
//! direction (see `geo.zig`). The result goes to the USB serial port, one line
//! per press, alongside the logs, where `idf.py monitor` or any serial
//! monitor shows it.
//!
//! Before trusting the bearings, set `constants.lidar_boresight` to how the
//! LiDAR is mounted against the IMU and `constants.magnetic_declination_deg`
//! to where the device is used. Both live in `constants.zig`, with the pin
//! map and every other fixed setting.

const std = @import("std");
const idf = @import("idf");
const ping = @import("ping");

const constants = ping.constants;
const geo = ping.geo;
const gpio = ping.gpio;
const gps = ping.gps;
const imu = ping.imu;
const rtos = ping.rtos;
const tf03 = ping.tf03;

pub const std_options: std.Options = .{ .logFn = rtos.logFn, .log_level = .info };
pub const panic = rtos.panic;

const log = std.log.scoped(.ping);

/// Each sensor task's stack, in bytes.
const sensor_stack = 6 * 1024;

/// How often the main task looks at the button.
const button_poll_ms = 5;

/// How long the IMU task waits for one report before checking the clock.
const imu_wait_ms = 100;

/// Why a press produced no coordinate, as a sentence.
const Problem = rtos.Text(96);

/// The latest reading from one sensor, and when it arrived. The sensor's task
/// overwrites it; a button press copies it out.
fn Latest(comptime T: type) type {
    return struct {
        const Self = @This();

        pub const Result = union(enum) {
            reading: T,
            problem: Problem,
        };

        sensor: []const u8,
        mutex: rtos.Mutex = .{},
        reading: ?T = null,
        at_ms: i64 = 0,

        pub fn set(self: *Self, value: T) void {
            self.mutex.lock();
            defer self.mutex.unlock();
            self.reading = value;
            self.at_ms = rtos.nowMs();
        }

        /// The reading, as long as it is no older than `max_age_ms`.
        pub fn get(self: *Self, max_age_ms: u32) Result {
            self.mutex.lock();
            defer self.mutex.unlock();
            const reading = self.reading orelse return .{ .problem = .init("no {s} reading yet", .{self.sensor}) };
            const age = rtos.nowMs() - self.at_ms;
            if (age > max_age_ms) return .{ .problem = .init("last {s} reading is {d} ms old", .{ self.sensor, age }) };
            return .{ .reading = reading };
        }
    };
}

var g_gps: Latest(gps.Solution) = .{ .sensor = "GPS" };
var g_imu: Latest(imu.Orientation) = .{ .sensor = "IMU" };
var g_lidar: Latest(tf03.Measurement) = .{ .sensor = "LiDAR" };

/// A located target, and what it was located from.
const Sighting = struct {
    target: geo.Position,
    origin: geo.Position,
    aim: geo.Aim,
    range_m: f32,
    /// The GPS's 1σ estimate of its horizontal error, metres.
    origin_accuracy_m: f32,
    satellites: u8,
    /// The IMU's estimate of its heading error, degrees.
    heading_accuracy_deg: f32,
    /// How far the IMU trusts its magnetometer calibration.
    compass: imu.Accuracy,

    pub fn describe(sighting: *const Sighting) rtos.Text(320) {
        return .init("{d:.7}, {d:.7}, alt {d:.1} m" ++
            " | range {d:.2} m, bearing {d:.1} deg, elevation {d:.1} deg" ++
            " | from {d:.7}, {d:.7}, alt {d:.1} m, +/-{d:.1} m, {d} satellites" ++
            " | heading +/-{d:.1} deg, compass calibration {s}", .{
            sighting.target.latitude_deg,     sighting.target.longitude_deg,  sighting.target.altitude_m,
            sighting.range_m,                 sighting.aim.azimuth_deg,       sighting.aim.elevation_deg,
            sighting.origin.latitude_deg,     sighting.origin.longitude_deg,  sighting.origin.altitude_m,
            sighting.origin_accuracy_m,       sighting.satellites,            sighting.heading_accuracy_deg,
            sighting.compass.name(),
        });
    }
};

const Outcome = union(enum) {
    sighting: Sighting,
    problem: Problem,
};

/// Locates what the LiDAR is pointing at, from the latest sensor readings.
fn takeSighting() Outcome {
    const solution = switch (g_gps.get(constants.max_gps_age_ms)) {
        .reading => |reading| reading,
        .problem => |problem| return .{ .problem = problem },
    };
    const orientation = switch (g_imu.get(constants.max_imu_age_ms)) {
        .reading => |reading| reading,
        .problem => |problem| return .{ .problem = problem },
    };
    const measurement = switch (g_lidar.get(constants.max_lidar_age_ms)) {
        .reading => |reading| reading,
        .problem => |problem| return .{ .problem = problem },
    };

    if (!solution.hasPosition()) {
        return .{ .problem = .init("GPS has no usable fix ({s}, {d} satellites)", .{ solution.fix_type.name(), solution.satellites }) };
    }
    const distance_cm = measurement.distance_cm orelse {
        return .{ .problem = .init("LiDAR has no return it trusts (strength {d})", .{measurement.strength}) };
    };

    const origin: geo.Position = .{
        .latitude_deg = solution.latitude_deg,
        .longitude_deg = solution.longitude_deg,
        .altitude_m = solution.height_msl_m,
    };
    const aim = geo.Aim.fromOrientation(orientation.quaternion, constants.lidar_boresight, constants.magnetic_declination_deg);
    const range_m = @as(f32, @floatFromInt(distance_cm)) / 100.0;

    return .{ .sighting = .{
        .target = origin.project(aim, range_m),
        .origin = origin,
        .aim = aim,
        .range_m = range_m,
        .origin_accuracy_m = solution.horizontal_accuracy_m,
        .satellites = solution.satellites,
        .heading_accuracy_deg = std.math.radiansToDegrees(orientation.heading_accuracy_rad),
        .compass = orientation.calibration,
    } };
}

/// Keeps the latest navigation solution from the NEO-M9N.
fn gpsTask(_: ?*anyopaque) callconv(.c) void {
    var receiver: gps.Gps = .init(constants.gps_uart);
    rtos.must(receiver.begin(constants.gps_tx, constants.gps_rx, constants.gps_baud_rate));

    while (true) {
        // The configuration lives in the receiver's RAM, so this runs again
        // whenever the receiver goes quiet, in case a power glitch has put it
        // back to its factory NMEA output.
        while (!receiver.configure(constants.gps_rate_hz)) {
            log.warn("GPS configuration failed, retrying", .{});
            rtos.delayMs(constants.retry_delay_ms);
        }
        log.info("GPS up: {d} solutions a second", .{constants.gps_rate_hz});

        var last = rtos.nowMs();
        while (true) {
            if (receiver.poll()) |solution| {
                g_gps.set(solution);
                last = rtos.nowMs();
            } else if (rtos.nowMs() - last > constants.gps_silence_ms) {
                log.warn("GPS went quiet, configuring it again", .{});
                break;
            } else {
                rtos.delayMs(10);
            }
        }
    }
}

/// Keeps the latest orientation from the BNO085.
fn imuTask(_: ?*anyopaque) callconv(.c) void {
    var hub: imu.Imu = .{};

    while (true) {
        if (!hub.start(constants.imu_report_interval_us)) {
            log.warn("BNO085 bring-up failed, retrying", .{});
            rtos.delayMs(constants.retry_delay_ms);
            continue;
        }
        const id = hub.productId();
        log.info("BNO085 up: firmware {d}.{d}.{d}, reset cause {d}", .{ id.swVersionMajor, id.swVersionMinor, id.swVersionPatch, id.resetCause });

        var last = rtos.nowMs();
        while (true) {
            if (hub.wait(imu_wait_ms)) |orientation| {
                g_imu.set(orientation);
                last = rtos.nowMs();
            } else if (rtos.nowMs() - last > constants.imu_silence_ms) {
                log.warn("BNO085 went quiet, starting it again", .{});
                break;
            }
        }
    }
}

/// Keeps the latest distance from the TF03.
fn lidarTask(_: ?*anyopaque) callconv(.c) void {
    var lidar: tf03.Tf03 = .init(constants.lidar_uart);
    rtos.must(lidar.begin(constants.lidar_tx, constants.lidar_rx, constants.lidar_baud_rate));

    // The TF03 streams measurements unasked, so distances need only its TxD
    // wire. Answering a command also needs its RxD wire, so the firmware
    // version is asked for once, for the log, and its answer waited for no
    // further.
    if (lidar.firmwareVersion()) |version| {
        log.info("TF03 firmware {d}.{d}.{d}", .{ version.major, version.minor, version.patch });
    } else |err| {
        log.warn("TF03 did not answer the firmware version request: {s}. Reading distances anyway.", .{lidar.describe(err)});
    }

    while (true) {
        if (lidar.read(constants.lidar_silence_ms)) |measurement| {
            g_lidar.set(measurement);
        } else |err| {
            log.warn("TF03 read failed: {s}", .{lidar.describe(err)});
        }
    }
}

fn buttonDown() bool {
    return gpio.read(constants.button) == 0;
}

/// Waits for the button to be let go and to stay up past any contact bounce.
fn waitForRelease() void {
    while (true) {
        while (buttonDown()) rtos.delayMs(button_poll_ms);
        rtos.delayMs(constants.debounce_ms);
        if (!buttonDown()) return;
    }
}

export fn app_main() void {
    // Trigger button, wired from its GPIO to ground.
    gpio.input(constants.button, .up);

    // The slots' mutexes, before anything can race to create them.
    g_gps.mutex.init();
    g_imu.mutex.init();
    g_lidar.mutex.init();

    rtos.spawn(gpsTask, "gps", sensor_stack, 5);
    rtos.spawn(imuTask, "imu", sensor_stack, 5);
    rtos.spawn(lidarTask, "lidar", sensor_stack, 5);

    var presses: u32 = 0;
    while (true) {
        while (!buttonDown()) rtos.delayMs(button_poll_ms);
        // Take the readings from when the button went down, before pressing
        // it has had time to jog where the device points.
        const outcome = takeSighting();
        rtos.delayMs(constants.debounce_ms);
        if (!buttonDown()) {
            // Too short to be a press: noise, or contacts bouncing.
            continue;
        }

        presses += 1;
        switch (outcome) {
            .sighting => |sighting| {
                const text = sighting.describe();
                _ = idf.printf("target #%lu: %.*s\n", @as(c_ulong, presses), @as(c_int, @intCast(text.len)), &text.buf);
            },
            .problem => |problem| {
                _ = idf.printf("target #%lu: none, %.*s\n", @as(c_ulong, presses), @as(c_int, @intCast(problem.len)), &problem.buf);
            },
        }

        // One coordinate per press, however long the button is held.
        waitForRelease();
    }
}
