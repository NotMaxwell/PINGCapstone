//! Turning a LiDAR range and the IMU's orientation into a coordinate.
//!
//! The device stands at a position the GPS knows, points the LiDAR at
//! something and measures how far away it is. That makes a polar
//! observation: a slant range along a direction given by an azimuth, the
//! compass bearing, and an elevation, the angle above the horizon.
//! [`Aim::from_orientation`] works out that direction from the IMU, and
//! [`Position::project`] walks the range out along it from the GPS position.
//!
//! # Accuracy
//!
//! The walk happens in the plane tangent to the WGS-84 ellipsoid at the
//! device, and metres north and east become degrees through the ellipsoid's
//! radii of curvature there. Checked against the exact straight line in
//! Earth-centred coordinates, over the TF03's 180 m maximum range, the result
//! is within 5 mm up to 60° of latitude and 3 cm at 85°. That is far below
//! what the sensors themselves can resolve. Of those, the heading from the
//! IMU usually dominates: each degree of heading error moves the result
//! sideways by 1.7 m per 100 m of range.

use libm::{atan2f, cos, hypotf, sin, sqrt};

use crate::bno085::{Quaternion, Vec3};

/// WGS-84 semi-major axis, metres.
const WGS84_A: f64 = 6_378_137.0;
/// WGS-84 flattening.
const WGS84_F: f64 = 1.0 / 298.257_223_563;
/// WGS-84 first eccentricity squared.
const WGS84_E2: f64 = WGS84_F * (2.0 - WGS84_F);

/// A point on the Earth.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Position {
    /// WGS-84 latitude, degrees, positive north.
    pub latitude_deg: f64,
    /// WGS-84 longitude, degrees, positive east.
    pub longitude_deg: f64,
    /// Height, metres, above mean sea level or the ellipsoid. The two differ
    /// by less than 110 m, which changes nothing [`Position::project`] does
    /// by more than 3 mm, so either will do; the application uses sea level.
    pub altitude_m: f64,
}

/// A direction from the device, relative to the local horizon.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Aim {
    /// Compass bearing, degrees clockwise from true north, from 0 up to 360.
    pub azimuth_deg: f32,
    /// Degrees above the horizon, negative below it.
    pub elevation_deg: f32,
}

impl Aim {
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
    pub fn from_orientation(
        orientation: &Quaternion,
        boresight: Vec3,
        declination_deg: f32,
    ) -> Self {
        let Quaternion {
            i: x,
            j: y,
            k: z,
            real: w,
        } = *orientation;
        let Vec3 {
            x: bx,
            y: by,
            z: bz,
        } = boresight;

        // The boresight rotated into the world frame, q b q*. This is written
        // out in full rather than assuming |q| = 1, so that rounding in the
        // quaternion's fixed point components scales the result slightly
        // instead of skewing it. The angles below do not depend on scale.
        let east = (w * w + x * x - y * y - z * z) * bx
            + 2.0 * (x * y - w * z) * by
            + 2.0 * (x * z + w * y) * bz;
        let north = 2.0 * (x * y + w * z) * bx
            + (w * w - x * x + y * y - z * z) * by
            + 2.0 * (y * z - w * x) * bz;
        let up = 2.0 * (x * z - w * y) * bx
            + 2.0 * (y * z + w * x) * by
            + (w * w - x * x - y * y + z * z) * bz;

        let mut azimuth_deg = atan2f(east, north).to_degrees() + declination_deg;
        if azimuth_deg < 0.0 {
            azimuth_deg += 360.0;
        }
        if azimuth_deg >= 360.0 {
            azimuth_deg -= 360.0;
        }

        Self {
            azimuth_deg,
            elevation_deg: atan2f(up, hypotf(east, north)).to_degrees(),
        }
    }
}

impl Position {
    /// The point `range_m` metres from here along `aim`.
    pub fn project(&self, aim: Aim, range_m: f32) -> Self {
        let range = f64::from(range_m);
        let azimuth = f64::from(aim.azimuth_deg).to_radians();
        let elevation = f64::from(aim.elevation_deg).to_radians();

        // The slant range as metres east, north and up.
        let horizontal = range * cos(elevation);
        let east = horizontal * sin(azimuth);
        let north = horizontal * cos(azimuth);
        let up = range * sin(elevation);

        // How many metres a radian spans at this height: along the meridian
        // for latitude, and at right angles to it for longitude, which also
        // shrinks with the circle of latitude. That circle is taken halfway
        // along the move, since near the poles it changes over even a
        // short one.
        let latitude = self.latitude_deg.to_radians();
        let sin_latitude = sin(latitude);
        let w = 1.0 - WGS84_E2 * sin_latitude * sin_latitude;
        let prime_vertical_radius = WGS84_A / sqrt(w) + self.altitude_m;
        let meridian_radius = WGS84_A * (1.0 - WGS84_E2) / (w * sqrt(w)) + self.altitude_m;

        let delta_latitude = north / meridian_radius;
        let mut longitude_deg = self.longitude_deg
            + (east / (prime_vertical_radius * cos(latitude + delta_latitude / 2.0))).to_degrees();
        // Stay within (-180, 180] across the antimeridian.
        if longitude_deg > 180.0 {
            longitude_deg -= 360.0;
        } else if longitude_deg <= -180.0 {
            longitude_deg += 360.0;
        }

        Self {
            latitude_deg: self.latitude_deg + delta_latitude.to_degrees(),
            longitude_deg,
            altitude_m: self.altitude_m + up,
        }
    }
}
