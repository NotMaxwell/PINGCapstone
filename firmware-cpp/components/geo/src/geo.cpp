#include "ping/geo.hpp"

#include <cmath>
#include <numbers>

namespace ping::geo {

namespace {

/// WGS-84 semi-major axis, metres.
constexpr double kWgs84A = 6'378'137.0;
/// WGS-84 flattening.
constexpr double kWgs84F = 1.0 / 298.257'223'563;
/// WGS-84 first eccentricity squared.
constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);

constexpr float degrees(float radians)
{
    return radians * (180.0F / std::numbers::pi_v<float>);
}

constexpr double degrees(double radians)
{
    return radians * (180.0 / std::numbers::pi);
}

constexpr double radians(double degrees)
{
    return degrees * (std::numbers::pi / 180.0);
}

}  // namespace

Aim Aim::from_orientation(const Quaternion& orientation, Vec3 boresight, float declination_deg)
{
    const auto [x, y, z, w] = orientation;
    const auto [bx, by, bz] = boresight;

    // The boresight rotated into the world frame, q b q*. This is written
    // out in full rather than assuming |q| = 1, so that rounding in the
    // quaternion's fixed point components scales the result slightly instead
    // of skewing it. The angles below do not depend on scale.
    const float east = (w * w + x * x - y * y - z * z) * bx + 2.0F * (x * y - w * z) * by +
                       2.0F * (x * z + w * y) * bz;
    const float north = 2.0F * (x * y + w * z) * bx + (w * w - x * x + y * y - z * z) * by +
                        2.0F * (y * z - w * x) * bz;
    const float up = 2.0F * (x * z - w * y) * bx + 2.0F * (y * z + w * x) * by +
                     (w * w - x * x - y * y + z * z) * bz;

    float azimuth_deg = degrees(std::atan2(east, north)) + declination_deg;
    if (azimuth_deg < 0.0F) {
        azimuth_deg += 360.0F;
    }
    if (azimuth_deg >= 360.0F) {
        azimuth_deg -= 360.0F;
    }

    return Aim{
        .azimuth_deg = azimuth_deg,
        .elevation_deg = degrees(std::atan2(up, std::hypot(east, north))),
    };
}

Position Position::project(Aim aim, float range_m) const
{
    const double range = range_m;
    const double azimuth = radians(aim.azimuth_deg);
    const double elevation = radians(aim.elevation_deg);

    // The slant range as metres east, north and up.
    const double horizontal = range * std::cos(elevation);
    const double east = horizontal * std::sin(azimuth);
    const double north = horizontal * std::cos(azimuth);
    const double up = range * std::sin(elevation);

    // How many metres a radian spans at this height: along the meridian for
    // latitude, and at right angles to it for longitude, which also shrinks
    // with the circle of latitude. That circle is taken halfway along the
    // move, since near the poles it changes over even a short one.
    const double latitude = radians(latitude_deg);
    const double sin_latitude = std::sin(latitude);
    const double w = 1.0 - kWgs84E2 * sin_latitude * sin_latitude;
    const double prime_vertical_radius = kWgs84A / std::sqrt(w) + altitude_m;
    const double meridian_radius = kWgs84A * (1.0 - kWgs84E2) / (w * std::sqrt(w)) + altitude_m;

    const double delta_latitude = north / meridian_radius;
    double longitude = longitude_deg + degrees(east / (prime_vertical_radius *
                                                       std::cos(latitude + delta_latitude / 2.0)));
    // Stay within (-180, 180] across the antimeridian.
    if (longitude > 180.0) {
        longitude -= 360.0;
    } else if (longitude <= -180.0) {
        longitude += 360.0;
    }

    return Position{
        .latitude_deg = latitude_deg + degrees(delta_latitude),
        .longitude_deg = longitude,
        .altitude_m = altitude_m + up,
    };
}

}  // namespace ping::geo
