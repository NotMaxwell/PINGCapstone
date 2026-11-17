/// @file
/// Magnetometer calibration tests: the collector, the ellipsoid and hard iron
/// fits, start-up calibration from handling, and the compass heading.

#include "ping/mag_cal.hpp"

#include "ping/geo.hpp"
#include "ping/settings.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

using namespace ping;
using namespace ping::mag_cal;
using geo::Quaternion;
using geo::Vec3;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kDegree = kPi / 180.0F;

/// A rotation of `degrees` about the unit axis (x, y, z).
Quaternion rotation(float degrees, float x, float y, float z)
{
    const float half = degrees * kDegree / 2.0F;
    const float s = std::sin(half);
    return {.i = x * s, .j = y * s, .k = z * s, .real = std::cos(half)};
}

/// The Hamilton product: rotating by it applies `b` first, then `a`.
Quaternion multiply(const Quaternion& a, const Quaternion& b)
{
    return {
        .i = a.real * b.i + a.i * b.real + a.j * b.k - a.k * b.j,
        .j = a.real * b.j - a.i * b.k + a.j * b.real + a.k * b.i,
        .k = a.real * b.k + a.i * b.j - a.j * b.i + a.k * b.real,
        .real = a.real * b.real - a.i * b.i - a.j * b.j - a.k * b.k,
    };
}

Quaternion conjugate(const Quaternion& q) { return {.i = -q.i, .j = -q.j, .k = -q.k, .real = q.real}; }

Vec3 rotate(const Quaternion& q, const Vec3& v)
{
    const Quaternion p = multiply(multiply(q, {.i = v.x, .j = v.y, .k = v.z, .real = 0.0F}), conjugate(q));
    return {.x = p.i, .y = p.j, .z = p.k};
}

/// Earth's field in a frame of east, magnetic north and up.
Vec3 earth_field()
{
    const float dip = kExpectedDipDeg * kDegree;
    return {.x = 0.0F, .y = kExpectedFieldUt * std::cos(dip), .z = -kExpectedFieldUt * std::sin(dip)};
}

/// `n` unit vectors spread evenly over the sphere (a Fibonacci lattice).
std::vector<Vec3> sphere(std::size_t n)
{
    std::vector<Vec3> points;
    const float golden = kPi * (3.0F - std::sqrt(5.0F));
    for (std::size_t i = 0; i < n; ++i) {
        const float z = 1.0F - 2.0F * (static_cast<float>(i) + 0.5F) / static_cast<float>(n);
        const float r = std::sqrt(1.0F - z * z);
        const float phi = golden * static_cast<float>(i);
        points.push_back({.x = r * std::cos(phi), .y = r * std::sin(phi), .z = z});
    }
    return points;
}

/// A symmetric soft iron distortion and hard iron offset, applied to a field.
constexpr std::array<std::array<float, 3>, 3> kStretch{{{1.15F, 0.05F, -0.03F}, {0.05F, 0.92F, 0.04F}, {-0.03F, 0.04F, 1.02F}}};
constexpr Vec3 kOffset{.x = 22.0F, .y = -13.5F, .z = 40.0F};

Vec3 distort(const Vec3& v)
{
    const auto row = [&](const std::array<float, 3>& r) { return r[0] * v.x + r[1] * v.y + r[2] * v.z; };
    return {.x = row(kStretch[0]) + kOffset.x, .y = row(kStretch[1]) + kOffset.y, .z = row(kStretch[2]) + kOffset.z};
}

float length(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

/// Raw readings from handling: turning through `turns` full circles while
/// tilting up to `tilt_deg` about two axes, with only a hard iron offset.
std::vector<Vec3> handling(std::size_t n, float turns, float tilt_deg)
{
    std::vector<Vec3> readings;
    const Vec3 field = earth_field();
    for (std::size_t i = 0; i < n; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(n);
        const Quaternion yaw = rotation(360.0F * turns * t, 0.0F, 0.0F, 1.0F);
        const Quaternion pitch = rotation(tilt_deg * std::sin(2.0F * kPi * 7.0F * t), 1.0F, 0.0F, 0.0F);
        const Quaternion roll = rotation(tilt_deg * std::sin(2.0F * kPi * 11.3F * t), 0.0F, 1.0F, 0.0F);
        const Quaternion orientation = multiply(yaw, multiply(pitch, roll));
        const Vec3 sensed = rotate(conjugate(orientation), field);
        readings.push_back({.x = sensed.x + kOffset.x, .y = sensed.y + kOffset.y, .z = sensed.z + kOffset.z});
    }
    return readings;
}

MagCalibration offset_only()
{
    return {.offset = {kOffset.x, kOffset.y, kOffset.z}, .matrix = kIdentity3, .field_ut = kExpectedFieldUt, .residual = 0.005F};
}

float yaw_of(const Quaternion& q) { return geo::to_euler(q).yaw_deg; }

float angle_between(float a_deg, float b_deg)
{
    const float d = std::fmod(a_deg - b_deg + 540.0F, 360.0F) - 180.0F;
    return std::abs(d);
}

} // namespace

TEST(Coverage, FullSphereCoversEverySector)
{
    const auto points = sphere(400);
    EXPECT_EQ(coverage(points, {0.0F, 0.0F, 0.0F}), kSectors);
}

TEST(Coverage, OneHemisphereCoversAboutHalf)
{
    std::vector<Vec3> upper;
    for (const auto& p : sphere(400)) {
        if (p.z > 0.5F) {
            upper.push_back(p);
        }
    }
    EXPECT_LT(coverage(upper, {0.0F, 0.0F, 0.0F}), kRequiredSectors);
}

TEST(Collector, RejectsCrowdedAndNonFiniteReadings)
{
    Collector collector;
    EXPECT_EQ(collector.covered(), 0U);
    EXPECT_TRUE(collector.add({.x = 10.0F, .y = 0.0F, .z = 0.0F}));
    EXPECT_FALSE(collector.add({.x = 12.0F, .y = 1.0F, .z = 0.0F}));
    EXPECT_FALSE(collector.add({.x = NAN, .y = 0.0F, .z = 0.0F}));
    EXPECT_FALSE(collector.add({.x = INFINITY, .y = 0.0F, .z = 0.0F}));
    EXPECT_TRUE(collector.add({.x = 20.0F, .y = 0.0F, .z = 0.0F}));
    EXPECT_EQ(collector.samples().size(), 2U);
    EXPECT_FALSE(collector.is_full());
}

TEST(Collector, StopsWhenFull)
{
    Collector collector;
    std::size_t kept = 0;
    for (int x = 0; x < 30 && !collector.is_full(); ++x) {
        for (int y = 0; y < 30 && !collector.is_full(); ++y) {
            if (collector.add({.x = 10.0F * static_cast<float>(x), .y = 10.0F * static_cast<float>(y), .z = 0.0F})) {
                ++kept;
            }
        }
    }
    EXPECT_TRUE(collector.is_full());
    EXPECT_EQ(kept, kCapacity);
    EXPECT_FALSE(collector.add({.x = -1000.0F, .y = 0.0F, .z = 0.0F}));
}

TEST(Collector, CoverageOfAnOffsetSphere)
{
    Collector collector;
    for (const auto& p : sphere(600)) {
        collector.add(distort({.x = p.x * kExpectedFieldUt, .y = p.y * kExpectedFieldUt, .z = p.z * kExpectedFieldUt}));
    }
    EXPECT_GE(collector.covered(), kRequiredSectors);
    EXPECT_LT(collector.samples().size(), kCapacity);
}

TEST(Fit, TooFewSamples)
{
    const auto points = sphere(49);
    EXPECT_EQ(fit(points).error(), FitError::kTooFewSamples);
    EXPECT_EQ(fit_hard_iron(points, kIdentity3).error(), FitError::kTooFewSamples);
}

TEST(Fit, PlanarReadingsAreRejected)
{
    std::vector<Vec3> circle;
    for (int i = 0; i < 100; ++i) {
        const float a = static_cast<float>(i) * 2.0F * kPi / 100.0F;
        circle.push_back({.x = 40.0F * std::cos(a) + 5.0F, .y = 40.0F * std::sin(a), .z = 3.0F});
    }
    EXPECT_FALSE(fit(circle).has_value());
}

TEST(Fit, RecoversAnOffsetStretchedEllipsoid)
{
    std::vector<Vec3> raw;
    for (const auto& p : sphere(300)) {
        raw.push_back(distort({.x = p.x * kExpectedFieldUt, .y = p.y * kExpectedFieldUt, .z = p.z * kExpectedFieldUt}));
    }
    const auto result = fit(raw);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->offset[0], kOffset.x, 0.01F);
    EXPECT_NEAR(result->offset[1], kOffset.y, 0.01F);
    EXPECT_NEAR(result->offset[2], kOffset.z, 0.01F);
    EXPECT_LT(result->residual, 1e-4F);
    EXPECT_LT(result->residual, kMaxResidual);

    // A is symmetric with determinant 1.
    const auto& a = result->matrix;
    EXPECT_NEAR(a[0][1], a[1][0], 1e-5F);
    EXPECT_NEAR(a[0][2], a[2][0], 1e-5F);
    EXPECT_NEAR(a[1][2], a[2][1], 1e-5F);
    const float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                      a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    EXPECT_NEAR(det, 1.0F, 1e-4F);

    // Every corrected reading lands on the sphere of radius β, and β is the
    // true field scaled by det(stretch)^(1/3).
    for (const auto& r : raw) {
        EXPECT_NEAR(length(result->apply(r)), result->field_ut, 0.01F);
    }
    const auto& s = kStretch;
    const float stretch_det = s[0][0] * (s[1][1] * s[2][2] - s[1][2] * s[2][1]) - s[0][1] * (s[1][0] * s[2][2] - s[1][2] * s[2][0]) +
                              s[0][2] * (s[1][0] * s[2][1] - s[1][1] * s[2][0]);
    EXPECT_NEAR(result->field_ut, kExpectedFieldUt * std::cbrt(stretch_det), 0.01F);
}

TEST(FitHardIron, FindsTheOffsetFromPartialCoverage)
{
    const auto raw = handling(400, 3.0F, 40.0F);
    const auto result = fit_hard_iron(raw, kIdentity3);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->calibration.offset[0], kOffset.x, 0.05F);
    EXPECT_NEAR(result->calibration.offset[1], kOffset.y, 0.05F);
    EXPECT_NEAR(result->calibration.offset[2], kOffset.z, 0.05F);
    EXPECT_NEAR(result->calibration.field_ut, kExpectedFieldUt, 0.05F);
    EXPECT_LT(result->calibration.residual, 1e-4F);
    EXPECT_GT(result->spread_ut, 7.0F);
}

TEST(FitHardIron, AppliesTheSoftIronMatrixFirst)
{
    // Readings b + S u with A = S⁻¹ known: the offset is still b.
    const auto& s = kStretch;
    const float det = s[0][0] * (s[1][1] * s[2][2] - s[1][2] * s[2][1]) - s[0][1] * (s[1][0] * s[2][2] - s[1][2] * s[2][0]) +
                      s[0][2] * (s[1][0] * s[2][1] - s[1][1] * s[2][0]);
    std::array<std::array<float, 3>, 3> inverse{};
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            const std::size_t r1 = (c + 1) % 3;
            const std::size_t r2 = (c + 2) % 3;
            const std::size_t c1 = (r + 1) % 3;
            const std::size_t c2 = (r + 2) % 3;
            inverse[r][c] = (s[r1][c1] * s[r2][c2] - s[r1][c2] * s[r2][c1]) / det;
        }
    }
    std::vector<Vec3> raw;
    for (const auto& p : sphere(200)) {
        raw.push_back(distort({.x = p.x * kExpectedFieldUt, .y = p.y * kExpectedFieldUt, .z = p.z * kExpectedFieldUt}));
    }
    const auto result = fit_hard_iron(raw, inverse);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->calibration.offset[0], kOffset.x, 0.05F);
    EXPECT_NEAR(result->calibration.offset[1], kOffset.y, 0.05F);
    EXPECT_NEAR(result->calibration.offset[2], kOffset.z, 0.05F);
    EXPECT_NEAR(result->calibration.field_ut, kExpectedFieldUt, 0.05F);
}

TEST(FitHardIron, TurningAboutOneAxisHasNoSpread)
{
    const auto raw = handling(400, 3.0F, 0.0F);
    const auto result = fit_hard_iron(raw, kIdentity3);
    if (result) {
        EXPECT_LT(result->spread_ut, 1.0F);
    }
}

TEST(StartupCalibration, AcceptsGoodHandlingOnce)
{
    StartupCalibration startup(std::nullopt);
    std::optional<MagCalibration> found;
    for (const auto& r : handling(4000, 10.0F, 50.0F)) {
        if (auto calibration = startup.add(r)) {
            ASSERT_FALSE(found.has_value()) << "returned a calibration twice";
            found = calibration;
        }
    }
    ASSERT_TRUE(found.has_value());
    EXPECT_NEAR(found->offset[0], kOffset.x, 1.0F);
    EXPECT_NEAR(found->offset[1], kOffset.y, 1.0F);
    EXPECT_NEAR(found->offset[2], kOffset.z, 1.0F);
    EXPECT_NEAR(found->field_ut, kExpectedFieldUt, 1.0F);
    EXPECT_EQ(found->matrix, kIdentity3);
}

TEST(StartupCalibration, WaitsWhileTurnedAboutOneAxis)
{
    StartupCalibration startup(std::nullopt);
    for (const auto& r : handling(4000, 10.0F, 0.0F)) {
        EXPECT_FALSE(startup.add(r).has_value());
    }
}

TEST(Compass, NothingUntilBothReportsArrive)
{
    Compass compass(offset_only());
    EXPECT_FALSE(compass.heading().has_value());
    compass.update_field({.x = 1.0F, .y = 2.0F, .z = 3.0F});
    EXPECT_FALSE(compass.heading().has_value());
    compass.update_game(rotation(0.0F, 0.0F, 0.0F, 1.0F));
    EXPECT_FALSE(compass.heading().has_value());
}

TEST(Compass, HeadingOfAKnownRotation)
{
    // True orientation: 70° clockwise from north (a turn of -70° about up),
    // tilted 20° nose up. The game frame started 123° off.
    const Quaternion truth = multiply(rotation(-70.0F, 0.0F, 0.0F, 1.0F), rotation(20.0F, 1.0F, 0.0F, 0.0F));
    const Quaternion game = multiply(rotation(123.0F, 0.0F, 0.0F, 1.0F), truth);
    const Vec3 sensed = rotate(conjugate(truth), earth_field());
    const Vec3 raw{.x = sensed.x + kOffset.x, .y = sensed.y + kOffset.y, .z = sensed.z + kOffset.z};

    Compass compass(offset_only());
    compass.update_game(game);
    for (int i = 0; i < 20; ++i) {
        compass.update_field(raw);
    }
    const auto heading = compass.heading();
    ASSERT_TRUE(heading.has_value());
    EXPECT_LT(angle_between(yaw_of(heading->quaternion), 70.0F), 0.1F);
    EXPECT_NEAR(geo::to_euler(heading->quaternion).pitch_deg, 20.0F, 0.1F);
    EXPECT_EQ(heading->status, geo::Accuracy::kHigh);
    EXPECT_LT(heading->accuracy_rad, 0.05F);
}

TEST(Compass, UncalibratedIsUnreliable)
{
    const Quaternion truth = rotation(-30.0F, 0.0F, 0.0F, 1.0F);
    Compass compass(std::nullopt);
    compass.update_game(truth);
    compass.update_field(rotate(conjugate(truth), earth_field()));
    const auto heading = compass.heading();
    ASSERT_TRUE(heading.has_value());
    EXPECT_LT(angle_between(yaw_of(heading->quaternion), 30.0F), 0.1F);
    EXPECT_EQ(heading->status, geo::Accuracy::kUnreliable);
    EXPECT_FLOAT_EQ(heading->accuracy_rad, kPi);
}

TEST(Compass, DisturbedFieldDoesNotMoveTheHeading)
{
    const Quaternion truth = rotation(-45.0F, 0.0F, 0.0F, 1.0F);
    const Vec3 sensed = rotate(conjugate(truth), earth_field());
    const Vec3 raw{.x = sensed.x + kOffset.x, .y = sensed.y + kOffset.y, .z = sensed.z + kOffset.z};

    Compass compass(offset_only());
    compass.update_game(truth);
    for (int i = 0; i < 20; ++i) {
        compass.update_field(raw);
    }
    ASSERT_TRUE(compass.heading().has_value());
    EXPECT_LT(angle_between(yaw_of(compass.heading()->quaternion), 45.0F), 0.1F);

    // A magnet passing by: half again as strong and pointing east.
    const Vec3 magnet = rotate(conjugate(truth), {.x = 1.5F * kExpectedFieldUt, .y = 0.0F, .z = 0.0F});
    const Vec3 disturbed{.x = magnet.x + kOffset.x, .y = magnet.y + kOffset.y, .z = magnet.z + kOffset.z};
    for (int i = 0; i < 100; ++i) {
        compass.update_field(disturbed);
    }
    const auto heading = compass.heading();
    ASSERT_TRUE(heading.has_value());
    EXPECT_LT(angle_between(yaw_of(heading->quaternion), 45.0F), 0.1F);
    EXPECT_EQ(heading->status, geo::Accuracy::kUnreliable);
}

TEST(Compass, NewCalibrationStartsOver)
{
    const Quaternion truth = rotation(-45.0F, 0.0F, 0.0F, 1.0F);
    Compass compass(offset_only());
    compass.update_game(truth);
    const Vec3 sensed = rotate(conjugate(truth), earth_field());
    compass.update_field({.x = sensed.x + kOffset.x, .y = sensed.y + kOffset.y, .z = sensed.z + kOffset.z});
    ASSERT_TRUE(compass.heading().has_value());
    compass.set_calibration(offset_only());
    EXPECT_FALSE(compass.heading().has_value());
}
