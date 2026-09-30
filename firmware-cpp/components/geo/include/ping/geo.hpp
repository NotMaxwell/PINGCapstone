/// @file
/// Turning a LiDAR range and the IMU's orientation into a coordinate.
///
/// The device stands at a position the GPS knows, points the LiDAR at
/// something and measures how far away it is. That makes a polar
/// observation: a slant range along a direction given by an azimuth, the
/// compass bearing, and an elevation, the angle above the horizon.
/// Aim::from_orientation() works out that direction from the IMU, and
/// Position::project() walks the range out along it from the GPS position.
///
/// # Accuracy
///
/// The walk happens in the plane tangent to the WGS-84 ellipsoid at the
/// device, and metres north and east become degrees through the ellipsoid's
/// radii of curvature there. Checked against the exact straight line in
/// Earth-centred coordinates, over the TF03's 180 m maximum range, the
/// result is within about 5 mm up to 60° of latitude and 3 cm at 85° (see
/// test/host/geo_test.cpp). That is far below what the sensors themselves
/// can resolve. Of those, the heading from the IMU usually dominates: each
/// degree of heading error moves the result sideways by 1.7 m per 100 m of
/// range.
///
/// Pure C++, with no ESP-IDF in it, so it builds and runs on a host as well.

#pragma once

namespace ping::geo {

/// A rotation, as a unit quaternion.
struct Quaternion {
    float i;
    float j;
    float k;
    float real;
};

/// A direction in the IMU's own frame.
struct Vec3 {
    float x;
    float y;
    float z;
};

/// A direction from the device, relative to the local horizon.
struct Aim {
    /// Compass bearing, degrees clockwise from true north, from 0 up to 360.
    float azimuth_deg;
    /// Degrees above the horizon, negative below it.
    float elevation_deg;

    /// The direction `boresight` points, given the IMU's orientation.
    ///
    /// `orientation` is the BNO085's rotation vector. The BNO085 uses
    /// Android's frames of reference, so the quaternion turns a vector in
    /// the sensor's own frame (the axes printed on the breakout) into a
    /// world frame whose X axis points east, Y to magnetic north and Z up.
    /// `boresight` is the direction the LiDAR looks, in the sensor's frame;
    /// it need not be a unit vector. `declination_deg` is how far magnetic
    /// north lies east of true north where the device is, which turns the
    /// magnetic bearing into a true one.
    static Aim from_orientation(const Quaternion& orientation, Vec3 boresight,
                                float declination_deg);
};

/// A point on the Earth.
struct Position {
    /// WGS-84 latitude, degrees, positive north.
    double latitude_deg;
    /// WGS-84 longitude, degrees, positive east.
    double longitude_deg;
    /// Height, metres, above mean sea level or the ellipsoid. The two differ
    /// by less than 110 m, which changes nothing project() does by more than
    /// 3 mm, so either will do; the application uses sea level.
    double altitude_m;

    /// The point `range_m` metres from here along `aim`.
    [[nodiscard]] Position project(Aim aim, float range_m) const;
};

}  // namespace ping::geo
