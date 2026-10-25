/// @file
/// Geometry tests: the aim taken from a quaternion, and the projection of a
/// range from a position.

#include "ping/geo.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

using namespace ping::geo;

namespace {

constexpr Quaternion kLevelFacingNorth{.i = 0, .j = 0, .k = 0, .real = 1};
constexpr Vec3 kForward{.x = 0, .y = 1, .z = 0};

/// A rotation of `degrees` about the axis (x, y, z), which must be a unit
/// vector.
Quaternion rotation(float degrees, float x, float y, float z)
{
    const float half = degrees * std::numbers::pi_v<float> / 360.0F;
    const float s = std::sin(half);
    return {.i = x * s, .j = y * s, .k = z * s, .real = std::cos(half)};
}

} // namespace

TEST(Aim, LevelAndFacingMagneticNorthIsBearingZero)
{
    const Aim aim = Aim::from_orientation(kLevelFacingNorth, kForward, 0.0F);
    EXPECT_NEAR(aim.azimuth_deg, 0.0F, 1e-4F);
    EXPECT_NEAR(aim.elevation_deg, 0.0F, 1e-4F);
}

TEST(Aim, TurningClockwiseIncreasesTheBearing)
{
    // Android's frame has Z up, so turning clockwise seen from above is a
    // negative rotation about Z.
    const Aim aim = Aim::from_orientation(rotation(-90.0F, 0, 0, 1), kForward, 0.0F);
    EXPECT_NEAR(aim.azimuth_deg, 90.0F, 1e-3F);
}

TEST(Aim, TiltingUpRaisesTheElevation)
{
    // Pitching +Y up is a positive rotation about X.
    const Aim aim = Aim::from_orientation(rotation(30.0F, 1, 0, 0), kForward, 0.0F);
    EXPECT_NEAR(aim.elevation_deg, 30.0F, 1e-3F);
}

TEST(Aim, DeclinationShiftsTheBearingAndWraps)
{
    EXPECT_NEAR(Aim::from_orientation(kLevelFacingNorth, kForward, 10.0F).azimuth_deg, 10.0F, 1e-4F);
    EXPECT_NEAR(Aim::from_orientation(kLevelFacingNorth, kForward, -10.0F).azimuth_deg, 350.0F, 1e-4F);
}

TEST(Aim, IgnoresTheQuaternionsScale)
{
    // Fixed point rounding leaves the hub's quaternion slightly off unit
    // length; a scaled one must point the same way.
    Quaternion q = rotation(-45.0F, 0, 0, 1);
    q.i *= 1.01F;
    q.j *= 1.01F;
    q.k *= 1.01F;
    q.real *= 1.01F;
    EXPECT_NEAR(Aim::from_orientation(q, kForward, 0.0F).azimuth_deg, 45.0F, 1e-3F);
}

TEST(Euler, LevelAndFacingMagneticNorthIsAllZero)
{
    const Euler angles = to_euler(kLevelFacingNorth);
    EXPECT_NEAR(angles.yaw_deg, 0.0F, 1e-4F);
    EXPECT_NEAR(angles.pitch_deg, 0.0F, 1e-4F);
    EXPECT_NEAR(angles.roll_deg, 0.0F, 1e-4F);
}

TEST(Euler, AgreesWithTheAimForYawAndPitch)
{
    const Quaternion q = rotation(-60.0F, 0, 0, 1);
    const Euler yawed = to_euler(q);
    EXPECT_NEAR(yawed.yaw_deg, Aim::from_orientation(q, kForward, 0.0F).azimuth_deg, 1e-3F);

    const Euler pitched = to_euler(rotation(20.0F, 1, 0, 0));
    EXPECT_NEAR(pitched.pitch_deg, 20.0F, 1e-3F);
}

TEST(Project, ZeroRangeStaysPut)
{
    const Position here{.latitude_deg = 40.0, .longitude_deg = -105.0, .altitude_m = 1600.0};
    const Position there = here.project({.azimuth_deg = 123.0F, .elevation_deg = 10.0F}, 0.0F);
    EXPECT_DOUBLE_EQ(there.latitude_deg, here.latitude_deg);
    EXPECT_DOUBLE_EQ(there.longitude_deg, here.longitude_deg);
    EXPECT_DOUBLE_EQ(there.altitude_m, here.altitude_m);
}

TEST(Project, DueNorthMovesOnlyTheLatitude)
{
    // At the equator a degree of latitude is 110 574 m on WGS-84.
    const Position here{.latitude_deg = 0.0, .longitude_deg = 0.0, .altitude_m = 0.0};
    const Position there = here.project({.azimuth_deg = 0.0F, .elevation_deg = 0.0F}, 100.0F);
    EXPECT_NEAR(there.latitude_deg, 100.0 / 110'574.0, 1e-8);
    EXPECT_NEAR(there.longitude_deg, 0.0, 1e-12);
}

TEST(Project, DueEastAtTheEquatorUsesTheEquatorialRadius)
{
    // 111 319.5 m per degree of longitude at the equator.
    const Position here{.latitude_deg = 0.0, .longitude_deg = 10.0, .altitude_m = 0.0};
    const Position there = here.project({.azimuth_deg = 90.0F, .elevation_deg = 0.0F}, 180.0F);
    EXPECT_NEAR(there.longitude_deg, 10.0 + 180.0 / 111'319.49, 1e-8);
    EXPECT_NEAR(there.latitude_deg, 0.0, 1e-9);
}

TEST(Project, StraightUpChangesOnlyTheAltitude)
{
    const Position here{.latitude_deg = 51.5, .longitude_deg = -0.1, .altitude_m = 20.0};
    const Position there = here.project({.azimuth_deg = 0.0F, .elevation_deg = 90.0F}, 50.0F);
    EXPECT_NEAR(there.altitude_m, 70.0, 1e-4);
    EXPECT_NEAR(there.latitude_deg, here.latitude_deg, 1e-9);
}

TEST(Project, WrapsAcrossTheAntimeridian)
{
    const Position here{.latitude_deg = 0.0, .longitude_deg = 179.9999, .altitude_m = 0.0};
    const Position there = here.project({.azimuth_deg = 90.0F, .elevation_deg = 0.0F}, 100.0F);
    EXPECT_LT(there.longitude_deg, -179.0);
    EXPECT_GT(there.longitude_deg, -180.0);
}
