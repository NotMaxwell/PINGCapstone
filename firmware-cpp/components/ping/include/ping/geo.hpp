/// @file
/// Turning a LiDAR range and the IMU's orientation into a coordinate.
///
/// The device stands at a position the GPS knows, points the LiDAR at
/// something and measures how far away it is. That makes a polar observation:
/// a slant range along a direction given by an azimuth, the compass bearing,
/// and an elevation, the angle above the horizon. @ref Aim::from_orientation
/// works out that direction from the IMU, and @ref Position::project walks the
/// range out along it from the GPS position.
///
/// # Accuracy
///
/// The walk happens in the plane tangent to the WGS-84 ellipsoid at the
/// device, and metres north and east become degrees through the ellipsoid's
/// radii of curvature there. Checked against the exact straight line in
/// Earth-centred coordinates, over the TF03's 180 m maximum range, the result
/// is within 5 mm up to 60° of latitude and 3 cm at 85°. That is far below
/// what the sensors themselves can resolve. Of those, the heading from the
/// IMU usually dominates: each degree of heading error moves the result
/// sideways by 1.7 m per 100 m of range.
///
/// Nothing here touches hardware, so the host tests exercise it directly.

#pragma once

namespace ping::geo {

/// A three axis vector, in whatever frame the caller says.
struct Vec3 {
    float x;
    float y;
    float z;
};

/// A unit quaternion describing orientation, as the BNO085 reports it.
struct Quaternion {
    float i;
    float j;
    float k;
    float real;
};

/// An orientation as yaw, pitch and roll; see @ref to_euler.
struct Euler {
    /// Compass bearing of +Y, degrees clockwise from magnetic north, from 0
    /// up to 360.
    float yaw_deg;
    /// Degrees +Y points above the horizon, negative below, from -90 to 90.
    float pitch_deg;
    /// Degrees the right side (+X) has dipped, negative when it has risen,
    /// from -180 to 180.
    float roll_deg;
};

/// The same orientation as yaw, pitch and roll, the way an aircraft would
/// describe it if it flew with the breakout's printed axes: +Y forward, +X
/// out to the right and +Z up.
///
/// The rotation vector turns the sensor's axes into a world frame of east,
/// magnetic north and up, so lying level with +Y towards magnetic north reads
/// zero on all three. Yaw is applied first, then pitch, then roll. Straight up
/// or down (pitch ±90°), yaw and roll turn about the same axis and only their
/// difference is meaningful.
///
/// The esp32_BNO08x library has its own `get_euler()`, but it uses a
/// different axis convention, so this one is kept to match the Rust firmware.
Euler to_euler(const Quaternion& orientation);

/// A direction from the device, relative to the local horizon.
struct Aim {
    /// Compass bearing, degrees clockwise from true north, from 0 up to 360.
    float azimuth_deg;
    /// Degrees above the horizon, negative below it.
    float elevation_deg;

    /// The direction `boresight` points, given the IMU's orientation.
    ///
    /// `orientation` is the BNO085's rotation vector. The BNO085 uses
    /// Android's frames of reference, so the quaternion turns a vector in the
    /// sensor's own frame (the axes printed on the breakout) into a world
    /// frame whose X axis points east, Y to magnetic north and Z up.
    /// `boresight` is the direction the LiDAR looks, in the sensor's frame;
    /// it need not be a unit vector. `declination_deg` is how far magnetic
    /// north lies east of true north where the device is, which turns the
    /// magnetic bearing into a true one.
    static Aim from_orientation(const Quaternion& orientation, const Vec3& boresight, float declination_deg);
};

/// A point on the Earth.
struct Position {
    /// WGS-84 latitude, degrees, positive north.
    double latitude_deg;
    /// WGS-84 longitude, degrees, positive east.
    double longitude_deg;
    /// Height, metres, above mean sea level or the ellipsoid. The two differ
    /// by less than 110 m, which changes nothing @ref project does by more
    /// than 3 mm, so either will do; the application uses sea level.
    double altitude_m;

    /// The point `range_m` metres from here along `aim`.
    [[nodiscard]] Position project(const Aim& aim, float range_m) const;
};

} // namespace ping::geo
