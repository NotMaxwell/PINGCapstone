#include "sighting.hpp"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <numbers>

#include "config.hpp"
#include "sensors.hpp"

namespace ping {

namespace {

template <typename... Visitors>
struct Overloaded : Visitors... {
    using Visitors::operator()...;
};

/// snprintf into `out`, returning what was written. The format attribute
/// has the compiler check every format string against its arguments.
[[gnu::format(printf, 2, 3)]] std::string_view print_to(std::span<char> out, const char* format, ...)
{
    if (out.empty()) {
        return {};
    }
    std::va_list args;
    va_start(args, format);
    const int written = std::vsnprintf(out.data(), out.size(), format, args);
    va_end(args);
    if (written < 0) {
        return {};
    }
    return {out.data(), std::min(static_cast<std::size_t>(written), out.size() - 1)};
}

}  // namespace

std::expected<Sighting, Problem> take_sighting()
{
    const auto solution = readings::gps.get(config::kMaxGpsAge);
    if (!solution) {
        return std::unexpected(solution.error());
    }
    const auto orientation = readings::imu.get(config::kMaxImuAge);
    if (!orientation) {
        return std::unexpected(orientation.error());
    }
    const auto measurement = readings::lidar.get(config::kMaxLidarAge);
    if (!measurement) {
        return std::unexpected(measurement.error());
    }

    const auto position = solution->position();
    if (!position) {
        return std::unexpected(NoFix{.fix_type = solution->fix_type, .satellites = solution->satellites});
    }
    if (!measurement->distance_cm) {
        return std::unexpected(NoReturn{.strength = measurement->strength});
    }

    const geo::Position origin{
        .latitude_deg = position->latitude_deg,
        .longitude_deg = position->longitude_deg,
        .altitude_m = solution->height_msl_m,
    };
    const geo::Aim aim = geo::Aim::from_orientation(orientation->quaternion, config::kLidarBoresight,
                                                    config::kMagneticDeclinationDeg);
    const float range_m = static_cast<float>(*measurement->distance_cm) / 100.0F;

    return Sighting{
        .target = origin.project(aim, range_m),
        .origin = origin,
        .aim = aim,
        .range_m = range_m,
        .origin_accuracy_m = solution->horizontal_accuracy_m,
        .satellites = solution->satellites,
        .heading_accuracy_deg = orientation->heading_accuracy_rad * (180.0F / std::numbers::pi_v<float>),
        .compass = orientation->calibration,
    };
}

std::string_view describe(const Sighting& s, std::span<char> out)
{
    return print_to(out,
                    "%.7f, %.7f, alt %.1f m"
                    " | range %.2f m, bearing %.1f deg, elevation %.1f deg"
                    " | from %.7f, %.7f, alt %.1f m, +/-%.1f m, %u satellites"
                    " | heading +/-%.1f deg, compass calibration %s",
                    s.target.latitude_deg, s.target.longitude_deg, s.target.altitude_m,
                    static_cast<double>(s.range_m), static_cast<double>(s.aim.azimuth_deg),
                    static_cast<double>(s.aim.elevation_deg), s.origin.latitude_deg,
                    s.origin.longitude_deg, s.origin.altitude_m,
                    static_cast<double>(s.origin_accuracy_m), s.satellites,
                    static_cast<double>(s.heading_accuracy_deg), to_string(s.compass));
}

std::string_view describe(const Problem& problem, std::span<char> out)
{
    return std::visit(
        Overloaded{
            [&](const Stale& stale) {
                return stale.age ? print_to(out, "last %s reading is %lld ms old", stale.sensor,
                                            static_cast<long long>(stale.age->count()))
                                 : print_to(out, "no %s reading yet", stale.sensor);
            },
            [&](const NoFix& no_fix) {
                return print_to(out, "GPS has no usable fix (%s, %u satellites)",
                                to_string(no_fix.fix_type), no_fix.satellites);
            },
            [&](const NoReturn& no_return) {
                return print_to(out, "LiDAR has no return it trusts (strength %u)", no_return.strength);
            },
        },
        problem);
}

}  // namespace ping
