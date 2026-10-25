//! Turning a LiDAR range and the IMU's orientation into a coordinate.
//!
//! The device stands at a position the GPS knows, points the LiDAR at
//! something and measures how far away it is. That makes a polar observation:
//! a slant range along a direction given by an azimuth, the compass bearing,
//! and an elevation, the angle above the horizon. `Aim.fromOrientation` works
//! out that direction from the IMU, and `Position.project` walks the range out
//! along it from the GPS position.
//!
//! # Accuracy
//!
//! The walk happens in the plane tangent to the WGS-84 ellipsoid at the
//! device, and metres north and east become degrees through the ellipsoid's
//! radii of curvature there. Checked against the exact straight line in
//! Earth-centred coordinates, over the TF03's 180 m maximum range, the result
//! is within 5 mm up to 60° of latitude and 3 cm at 85°. That is far below
//! what the sensors themselves can resolve. Of those, the heading from the IMU
//! usually dominates: each degree of heading error moves the result sideways
//! by 1.7 m per 100 m of range.
//!
//! Nothing here touches hardware, so `zig build test` exercises it directly.

const std = @import("std");
const math = std.math;

/// WGS-84 semi-major axis, metres.
const wgs84_a = 6_378_137.0;
/// WGS-84 flattening.
const wgs84_f = 1.0 / 298.257_223_563;
/// WGS-84 first eccentricity squared.
const wgs84_e2 = wgs84_f * (2.0 - wgs84_f);

/// A three axis vector, in whatever frame the caller says.
pub const Vec3 = struct {
    x: f32,
    y: f32,
    z: f32,
};

/// A unit quaternion describing orientation, as the BNO085 reports it.
pub const Quaternion = struct {
    i: f32,
    j: f32,
    k: f32,
    real: f32,
};

/// An orientation as yaw, pitch and roll; see `toEuler`.
pub const Euler = struct {
    /// Compass bearing of +Y, degrees clockwise from magnetic north, from 0 up
    /// to 360.
    yaw_deg: f32,
    /// Degrees +Y points above the horizon, negative below, from -90 to 90.
    pitch_deg: f32,
    /// Degrees the right side (+X) has dipped, negative when it has risen,
    /// from -180 to 180.
    roll_deg: f32,
};

/// The same orientation as yaw, pitch and roll, the way an aircraft would
/// describe it if it flew with the breakout's printed axes: +Y forward, +X out
/// to the right and +Z up.
///
/// The rotation vector turns the sensor's axes into a world frame of east,
/// magnetic north and up, so lying level with +Y towards magnetic north reads
/// zero on all three. Yaw is applied first, then pitch, then roll. Straight up
/// or down (pitch ±90°), yaw and roll turn about the same axis and only their
/// difference is meaningful.
pub fn toEuler(orientation: Quaternion) Euler {
    const x = orientation.i;
    const y = orientation.j;
    const z = orientation.k;
    const w = orientation.real;

    // Elements of the rotation matrix, written out in full so that rounding in
    // the fixed point components scales them rather than skewing them. atan2
    // ignores the scale; asin needs it divided out.
    const norm = w * w + x * x + y * y + z * z;
    const r01 = 2.0 * (x * y - w * z);
    const r11 = w * w - x * x + y * y - z * z;
    const r20 = 2.0 * (x * z - w * y);
    const r21 = 2.0 * (y * z + w * x);
    const r22 = w * w - x * x - y * y + z * z;

    var yaw_deg = math.radiansToDegrees(math.atan2(r01, r11));
    if (yaw_deg < 0.0) yaw_deg += 360.0;
    return .{
        .yaw_deg = yaw_deg,
        .pitch_deg = math.radiansToDegrees(math.asin(math.clamp(r21 / norm, -1.0, 1.0))),
        .roll_deg = math.radiansToDegrees(math.atan2(-r20, r22)),
    };
}

/// A direction from the device, relative to the local horizon.
pub const Aim = struct {
    /// Compass bearing, degrees clockwise from true north, from 0 up to 360.
    azimuth_deg: f32,
    /// Degrees above the horizon, negative below it.
    elevation_deg: f32,

    /// The direction `boresight` points, given the IMU's orientation.
    ///
    /// `orientation` is the BNO085's rotation vector. The BNO085 uses
    /// Android's frames of reference, so the quaternion turns a vector in the
    /// sensor's own frame (the axes printed on the breakout) into a world
    /// frame whose X axis points east, Y to magnetic north and Z up.
    /// `boresight` is the direction the LiDAR looks, in the sensor's frame; it
    /// need not be a unit vector. `declination_deg` is how far magnetic north
    /// lies east of true north where the device is, which turns the magnetic
    /// bearing into a true one.
    pub fn fromOrientation(orientation: Quaternion, boresight: Vec3, declination_deg: f32) Aim {
        const x = orientation.i;
        const y = orientation.j;
        const z = orientation.k;
        const w = orientation.real;
        const bx = boresight.x;
        const by = boresight.y;
        const bz = boresight.z;

        // The boresight rotated into the world frame, q b q*. This is written
        // out in full rather than assuming |q| = 1, so that rounding in the
        // quaternion's fixed point components scales the result slightly
        // instead of skewing it. The angles below do not depend on scale.
        const east = (w * w + x * x - y * y - z * z) * bx + 2.0 * (x * y - w * z) * by + 2.0 * (x * z + w * y) * bz;
        const north = 2.0 * (x * y + w * z) * bx + (w * w - x * x + y * y - z * z) * by + 2.0 * (y * z - w * x) * bz;
        const up = 2.0 * (x * z - w * y) * bx + 2.0 * (y * z + w * x) * by + (w * w - x * x - y * y + z * z) * bz;

        var azimuth_deg = math.radiansToDegrees(math.atan2(east, north)) + declination_deg;
        if (azimuth_deg < 0.0) azimuth_deg += 360.0;
        if (azimuth_deg >= 360.0) azimuth_deg -= 360.0;

        return .{
            .azimuth_deg = azimuth_deg,
            .elevation_deg = math.radiansToDegrees(math.atan2(up, math.hypot(east, north))),
        };
    }
};

/// A point on the Earth.
pub const Position = struct {
    /// WGS-84 latitude, degrees, positive north.
    latitude_deg: f64,
    /// WGS-84 longitude, degrees, positive east.
    longitude_deg: f64,
    /// Height, metres, above mean sea level or the ellipsoid. The two differ
    /// by less than 110 m, which changes nothing `project` does by more than
    /// 3 mm, so either will do; the application uses sea level.
    altitude_m: f64,

    /// The point `range_m` metres from here along `aim`.
    pub fn project(here: Position, aim: Aim, range_m: f32) Position {
        const range: f64 = range_m;
        const azimuth = math.degreesToRadians(@as(f64, aim.azimuth_deg));
        const elevation = math.degreesToRadians(@as(f64, aim.elevation_deg));

        // The slant range as metres east, north and up.
        const horizontal = range * @cos(elevation);
        const east = horizontal * @sin(azimuth);
        const north = horizontal * @cos(azimuth);
        const up = range * @sin(elevation);

        // How many metres a radian spans at this height: along the meridian
        // for latitude, and at right angles to it for longitude, which also
        // shrinks with the circle of latitude. That circle is taken halfway
        // along the move, since near the poles it changes over even a short
        // one.
        const latitude = math.degreesToRadians(here.latitude_deg);
        const sin_latitude = @sin(latitude);
        const w = 1.0 - wgs84_e2 * sin_latitude * sin_latitude;
        const prime_vertical_radius = wgs84_a / @sqrt(w) + here.altitude_m;
        const meridian_radius = wgs84_a * (1.0 - wgs84_e2) / (w * @sqrt(w)) + here.altitude_m;

        const delta_latitude = north / meridian_radius;
        var longitude = here.longitude_deg +
            math.radiansToDegrees(east / (prime_vertical_radius * @cos(latitude + delta_latitude / 2.0)));
        // Stay within (-180, 180] across the antimeridian.
        if (longitude > 180.0) {
            longitude -= 360.0;
        } else if (longitude <= -180.0) {
            longitude += 360.0;
        }

        return .{
            .latitude_deg = here.latitude_deg + math.radiansToDegrees(delta_latitude),
            .longitude_deg = longitude,
            .altitude_m = here.altitude_m + up,
        };
    }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

const level_facing_north: Quaternion = .{ .i = 0, .j = 0, .k = 0, .real = 1 };
const forward: Vec3 = .{ .x = 0, .y = 1, .z = 0 };

/// A rotation of `degrees` about the axis (x, y, z), which must be a unit
/// vector.
fn rotation(degrees: f32, x: f32, y: f32, z: f32) Quaternion {
    const half = degrees * math.pi / 360.0;
    const s = @sin(half);
    return .{ .i = x * s, .j = y * s, .k = z * s, .real = @cos(half) };
}

test "aim: level and facing magnetic north is bearing zero" {
    const aim = Aim.fromOrientation(level_facing_north, forward, 0.0);
    try testing.expectApproxEqAbs(@as(f32, 0.0), aim.azimuth_deg, 1e-4);
    try testing.expectApproxEqAbs(@as(f32, 0.0), aim.elevation_deg, 1e-4);
}

test "aim: turning clockwise increases the bearing" {
    // Android's frame has Z up, so turning clockwise seen from above is a
    // negative rotation about Z.
    const aim = Aim.fromOrientation(rotation(-90.0, 0, 0, 1), forward, 0.0);
    try testing.expectApproxEqAbs(@as(f32, 90.0), aim.azimuth_deg, 1e-3);
}

test "aim: tilting up raises the elevation" {
    // Pitching +Y up is a positive rotation about X.
    const aim = Aim.fromOrientation(rotation(30.0, 1, 0, 0), forward, 0.0);
    try testing.expectApproxEqAbs(@as(f32, 30.0), aim.elevation_deg, 1e-3);
}

test "aim: declination shifts the bearing and wraps" {
    try testing.expectApproxEqAbs(@as(f32, 10.0), Aim.fromOrientation(level_facing_north, forward, 10.0).azimuth_deg, 1e-4);
    try testing.expectApproxEqAbs(@as(f32, 350.0), Aim.fromOrientation(level_facing_north, forward, -10.0).azimuth_deg, 1e-4);
}

test "aim: ignores the quaternion's scale" {
    // Fixed point rounding leaves the hub's quaternion slightly off unit
    // length; a scaled one must point the same way.
    var q = rotation(-45.0, 0, 0, 1);
    q.i *= 1.01;
    q.j *= 1.01;
    q.k *= 1.01;
    q.real *= 1.01;
    try testing.expectApproxEqAbs(@as(f32, 45.0), Aim.fromOrientation(q, forward, 0.0).azimuth_deg, 1e-3);
}

test "euler: level and facing magnetic north is all zero" {
    const angles = toEuler(level_facing_north);
    try testing.expectApproxEqAbs(@as(f32, 0.0), angles.yaw_deg, 1e-4);
    try testing.expectApproxEqAbs(@as(f32, 0.0), angles.pitch_deg, 1e-4);
    try testing.expectApproxEqAbs(@as(f32, 0.0), angles.roll_deg, 1e-4);
}

test "euler: agrees with the aim for yaw and pitch" {
    const q = rotation(-60.0, 0, 0, 1);
    try testing.expectApproxEqAbs(Aim.fromOrientation(q, forward, 0.0).azimuth_deg, toEuler(q).yaw_deg, 1e-3);
    try testing.expectApproxEqAbs(@as(f32, 20.0), toEuler(rotation(20.0, 1, 0, 0)).pitch_deg, 1e-3);
}

test "project: zero range stays put" {
    const here: Position = .{ .latitude_deg = 40.0, .longitude_deg = -105.0, .altitude_m = 1600.0 };
    const there = here.project(.{ .azimuth_deg = 123.0, .elevation_deg = 10.0 }, 0.0);
    try testing.expectEqual(here.latitude_deg, there.latitude_deg);
    try testing.expectEqual(here.longitude_deg, there.longitude_deg);
    try testing.expectEqual(here.altitude_m, there.altitude_m);
}

test "project: due north moves only the latitude" {
    // At the equator a degree of latitude is 110 574 m on WGS-84.
    const here: Position = .{ .latitude_deg = 0.0, .longitude_deg = 0.0, .altitude_m = 0.0 };
    const there = here.project(.{ .azimuth_deg = 0.0, .elevation_deg = 0.0 }, 100.0);
    try testing.expectApproxEqAbs(@as(f64, 100.0 / 110_574.0), there.latitude_deg, 1e-8);
    try testing.expectApproxEqAbs(@as(f64, 0.0), there.longitude_deg, 1e-12);
}

test "project: due east at the equator uses the equatorial radius" {
    // 111 319.5 m per degree of longitude at the equator.
    const here: Position = .{ .latitude_deg = 0.0, .longitude_deg = 10.0, .altitude_m = 0.0 };
    const there = here.project(.{ .azimuth_deg = 90.0, .elevation_deg = 0.0 }, 180.0);
    try testing.expectApproxEqAbs(@as(f64, 10.0 + 180.0 / 111_319.49), there.longitude_deg, 1e-8);
    try testing.expectApproxEqAbs(@as(f64, 0.0), there.latitude_deg, 1e-9);
}

test "project: straight up changes only the altitude" {
    const here: Position = .{ .latitude_deg = 51.5, .longitude_deg = -0.1, .altitude_m = 20.0 };
    const there = here.project(.{ .azimuth_deg = 0.0, .elevation_deg = 90.0 }, 50.0);
    try testing.expectApproxEqAbs(@as(f64, 70.0), there.altitude_m, 1e-4);
    try testing.expectApproxEqAbs(here.latitude_deg, there.latitude_deg, 1e-9);
}

test "project: wraps across the antimeridian" {
    const here: Position = .{ .latitude_deg = 0.0, .longitude_deg = 179.9999, .altitude_m = 0.0 };
    const there = here.project(.{ .azimuth_deg = 90.0, .elevation_deg = 0.0 }, 100.0);
    try testing.expect(there.longitude_deg < -179.0);
    try testing.expect(there.longitude_deg > -180.0);
}
