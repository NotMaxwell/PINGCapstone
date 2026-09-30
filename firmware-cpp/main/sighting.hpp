/// @file
/// What a button press produces: the coordinate the LiDAR is pointing at,
/// or why there is none.

#pragma once

#include <cstdint>
#include <expected>
#include <span>
#include <string_view>
#include <variant>

#include "latest.hpp"
#include "ping/geo.hpp"
#include "sensors.hpp"

namespace ping {

/// A located target, and what it was located from.
struct Sighting {
    geo::Position target;
    geo::Position origin;
    geo::Aim aim;
    float range_m;
    /// The GPS's 1σ estimate of its horizontal error, metres.
    float origin_accuracy_m;
    std::uint8_t satellites;
    /// The IMU's estimate of its heading error, degrees.
    float heading_accuracy_deg;
    /// How far the IMU trusts its magnetometer calibration.
    Accuracy compass;
};

/// The GPS is running but has no position it vouches for: no fix yet, or
/// one outside its accuracy limits.
struct NoFix {
    FixType fix_type;
    std::uint8_t satellites;
};

/// The LiDAR saw nothing in range, or nothing it trusts.
struct NoReturn {
    std::uint16_t strength;
};

/// Why a press produced no coordinate.
using Problem = std::variant<Stale, NoFix, NoReturn>;

/// Locates what the LiDAR is pointing at, from the latest sensor readings.
std::expected<Sighting, Problem> take_sighting();

/// Writes one line of text describing a sighting or a problem into `out`,
/// for the serial port, and returns what was written. A line that does not
/// fit is cut short.
std::string_view describe(const Sighting& sighting, std::span<char> out);
std::string_view describe(const Problem& problem, std::span<char> out);

}  // namespace ping
