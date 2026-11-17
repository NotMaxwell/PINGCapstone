/// @file
/// The magnetometer correction `m = A (x - b)`, on its own so that
/// @ref ping/settings.hpp can hold one without pulling in the fitting code in
/// @ref ping/mag_cal.hpp.

#pragma once

#include "ping/geo.hpp"

#include <array>

namespace ping {

/// The correction for the magnetometer: `m = A (x - b)`. `imu_cal` measures
/// it; see `docs/heading-calibration.md`.
struct MagCalibration {
    /// `b`, the hard iron offset, µT.
    std::array<float, 3> offset;
    /// `A`, the soft iron correction: symmetric, determinant 1.
    std::array<std::array<float, 3>, 3> matrix;
    /// `β`, the field strength the readings imply, µT.
    float field_ut;
    /// How far corrected readings sit from a sphere of radius `β`, as a
    /// fraction of it: MATLAB `magcal`'s `E`, the root mean square of
    /// `(|m|² - β²) / 2β²`.
    float residual;

    /// Corrects a raw reading.
    [[nodiscard]] constexpr geo::Vec3 apply(const geo::Vec3& raw) const
    {
        const float d[3] = {raw.x - offset[0], raw.y - offset[1], raw.z - offset[2]};
        const auto row = [&](const std::array<float, 3>& r) { return r[0] * d[0] + r[1] * d[1] + r[2] * d[2]; };
        return {.x = row(matrix[0]), .y = row(matrix[1]), .z = row(matrix[2])};
    }
};

/// The identity soft iron matrix: leaves the field as the hub reports it.
inline constexpr std::array<std::array<float, 3>, 3> kIdentity3{{{1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}}};

} // namespace ping
