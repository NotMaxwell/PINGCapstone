// Checks ping::geo on the host: the aim a known orientation gives, and how
// far Position::project() strays from the exact straight line computed in
// Earth-centred, Earth-fixed coordinates.

#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <numbers>

#include "ping/geo.hpp"

namespace {

using ping::geo::Aim;
using ping::geo::Position;
using ping::geo::Quaternion;

constexpr double kA = 6'378'137.0;
constexpr double kF = 1.0 / 298.257'223'563;
constexpr double kE2 = kF * (2.0 - kF);
constexpr double kRad = std::numbers::pi / 180.0;

int g_failures = 0;

void check(bool ok, const char* what)
{
    std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
    g_failures += ok ? 0 : 1;
}

bool near(double a, double b, double tolerance)
{
    return std::fabs(a - b) <= tolerance;
}

struct Ecef {
    double x;
    double y;
    double z;
};

Ecef to_ecef(double latitude_deg, double longitude_deg, double height_m)
{
    const double lat = latitude_deg * kRad;
    const double lon = longitude_deg * kRad;
    const double n = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
    return {(n + height_m) * std::cos(lat) * std::cos(lon), (n + height_m) * std::cos(lat) * std::sin(lon),
            (n * (1.0 - kE2) + height_m) * std::sin(lat)};
}

/// The largest distance, over a spread of bearings and elevations, between
/// project() and the exact end of a `range_m` straight line from `origin`.
double worst_projection_error(const Position& origin, float range_m)
{
    const double lat = origin.latitude_deg * kRad;
    const double lon = origin.longitude_deg * kRad;
    const Ecef start = to_ecef(origin.latitude_deg, origin.longitude_deg, origin.altitude_m);

    double worst = 0.0;
    for (float azimuth = 0.0F; azimuth < 360.0F; azimuth += 15.0F) {
        for (const float elevation : {-30.0F, 0.0F, 30.0F}) {
            const Position got = origin.project({.azimuth_deg = azimuth, .elevation_deg = elevation}, range_m);
            const Ecef got_ecef = to_ecef(got.latitude_deg, got.longitude_deg, got.altitude_m);

            // The same line as east/north/up, turned into ECEF.
            const double az = azimuth * kRad;
            const double el = elevation * kRad;
            const double e = range_m * std::cos(el) * std::sin(az);
            const double n = range_m * std::cos(el) * std::cos(az);
            const double u = range_m * std::sin(el);
            const Ecef exact{
                start.x - std::sin(lon) * e - std::sin(lat) * std::cos(lon) * n + std::cos(lat) * std::cos(lon) * u,
                start.y + std::cos(lon) * e - std::sin(lat) * std::sin(lon) * n + std::cos(lat) * std::sin(lon) * u,
                start.z + std::cos(lat) * n + std::sin(lat) * u,
            };

            worst = std::fmax(worst, std::hypot(got_ecef.x - exact.x, got_ecef.y - exact.y,
                                                got_ecef.z - exact.z));
        }
    }
    return worst;
}

}  // namespace

int main()
{
    const float half = std::numbers::pi_v<float> / 4.0F;     // 90° rotation, halved
    const float twelfth = std::numbers::pi_v<float> / 12.0F;  // 30° rotation, halved

    // Lying level with +Y to magnetic north, the BNO085 reports no rotation.
    Aim aim = Aim::from_orientation({0, 0, 0, 1}, {0, 1, 0}, 0.0F);
    check(near(aim.azimuth_deg, 0.0, 1e-4) && near(aim.elevation_deg, 0.0, 1e-4), "identity looks north, level");

    // Turned 90° anticlockwise seen from above, +Y looks west.
    aim = Aim::from_orientation({0, 0, std::sin(half), std::cos(half)}, {0, 1, 0}, 0.0F);
    check(near(aim.azimuth_deg, 270.0, 1e-3), "yaw of +90 degrees looks west");

    // Pitched up 30° about X, +Y looks 30° above the horizon; declination
    // moves the bearing.
    aim = Aim::from_orientation({std::sin(twelfth), 0, 0, std::cos(twelfth)}, {0, 1, 0}, 10.0F);
    check(near(aim.azimuth_deg, 10.0, 1e-3) && near(aim.elevation_deg, 30.0, 1e-3),
          "pitch of +30 degrees, declination 10 degrees");

    // A west declination past north wraps into [0, 360).
    aim = Aim::from_orientation({0, 0, 0, 1}, {0, 1, 0}, -5.0F);
    check(near(aim.azimuth_deg, 355.0, 1e-3), "negative declination wraps");

    // Over the TF03's full 180 m, against the exact straight line.
    for (const double latitude : {0.0, 30.0, 45.0, 60.0, 85.0}) {
        const double worst = worst_projection_error({latitude, -97.0, 200.0}, 180.0F);
        const double limit = latitude <= 60.0 ? 0.006 : 0.03;
        char what[96];
        std::snprintf(what, sizeof what, "180 m at latitude %.0f: worst error %.1f mm (limit %.0f mm)", latitude,
                      worst * 1e3, limit * 1e3);
        check(worst <= limit, what);
    }

    // East across the antimeridian comes back in at -180.
    const Position wrapped = Position{0.0, 179.9999, 0.0}.project({.azimuth_deg = 90.0F, .elevation_deg = 0.0F}, 100.0F);
    check(wrapped.longitude_deg < -179.99 && wrapped.longitude_deg > -180.0, "longitude wraps at the antimeridian");

    std::printf("%s\n", g_failures == 0 ? "all passed" : "FAILED");
    return g_failures == 0 ? 0 : 1;
}
