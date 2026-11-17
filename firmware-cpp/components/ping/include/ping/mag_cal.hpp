/// @file
/// Magnetometer calibration on the ESP32, and the compass heading built on it.
///
/// Turned through every orientation, a perfect magnetometer's readings lie on
/// a sphere around the origin whose radius is the strength of Earth's field.
/// Magnets that ride along with the sensor (hard iron) move the centre of
/// that sphere; iron that bends the field (soft iron) stretches and shears it
/// into an ellipsoid. So the raw readings are modelled as an offset ellipsoid,
/// `(x - b)ᵀ R (x - b) = β²`, and undoing it is `m = A (x - b)`, with `b` the
/// hard iron offset, `A` the symmetric square root of `R` scaled to
/// determinant 1, and `β` the field strength the data implies. This is the
/// form MATLAB's `magcal` uses with its `sym` fit, and its residual is the
/// quality figure here too.
///
/// The BNO085 has no way to accept a correction like this, and its rotation
/// vector only ever uses the hub's own magnetometer calibration. So
/// @ref Compass takes the hub's game rotation vector, which is gyroscope and
/// accelerometer only, and works out on the ESP32 which way north lies in
/// that frame.
///
/// Nothing here touches hardware. `imu_cal` runs the sequence that feeds
/// @ref Collector and @ref fit, and prints the @ref ping::MagCalibration to
/// put in @ref ping/settings.hpp.

#pragma once

#include "ping/calibration.hpp"
#include "ping/geo.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <limits>
#include <optional>
#include <span>

namespace ping::mag_cal {

/// How many readings @ref Collector keeps.
inline constexpr std::size_t kCapacity = 512;

/// The directions the readings are sorted into to judge coverage: each axis
/// divided into negative, about zero, and positive gives 3³ cells, less the
/// empty middle one.
inline constexpr std::size_t kSectors = 26;

/// How many @ref kSectors must hold readings. A little short of all of them,
/// so a pose the USB cable makes awkward does not hold up the whole run.
inline constexpr std::size_t kRequiredSectors = 24;

/// The largest acceptable residual, as a fraction of the field: about 3%, or
/// 1.5 µT. A third of that is normal for a clean fit.
inline constexpr float kMaxResidual = 0.03F;

/// How far the fitted field strength may sit from Earth's, as a fraction.
inline constexpr float kFieldTolerance = 0.2F;

/// Why a set of readings gave no calibration.
enum class FitError : unsigned char {
    /// Too few readings to fit.
    kTooFewSamples,
    /// The readings do not pin the fit down: they lie in a plane or on one
    /// small patch.
    kSingular,
    /// The surface that fits best is not an ellipsoid.
    kNotAnEllipsoid,
};

/// How many @ref kSectors around `centre` the readings cover.
std::size_t coverage(std::span<const geo::Vec3> samples, const std::array<float, 3>& centre);

/// Keeps readings spread evenly over the ellipsoid.
class Collector {
public:
    constexpr Collector() = default;

    /// Keeps `field` unless a reading close to it is already held. Returns
    /// whether it was kept.
    bool add(const geo::Vec3& field);

    [[nodiscard]] std::span<const geo::Vec3> samples() const { return {samples_.data(), len_}; }

    [[nodiscard]] bool is_full() const { return len_ == kCapacity; }

    /// How many @ref kSectors are covered so far, judged from the middle of
    /// the range seen on each axis: good enough to steer the operator before
    /// there is a fit to measure from.
    [[nodiscard]] std::size_t covered() const;

private:
    std::array<geo::Vec3, kCapacity> samples_{};
    std::size_t len_ = 0;
    std::array<float, 3> min_{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max()};
    std::array<float, 3> max_{std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest()};
};

/// Fits an offset ellipsoid to `samples` and returns the correction that
/// maps it onto a sphere.
///
/// The ellipsoid is `xᵀQx + 2nᵀx + d = 0` with the trace of `Q` fixed at 3,
/// which turns the fit into nine unknowns and one linear least squares
/// problem. A fit is only an ellipsoid if `Q` comes out positive definite,
/// which readings that do not cover enough directions can prevent, so that
/// is checked rather than assumed.
std::expected<MagCalibration, FitError> fit(std::span<const geo::Vec3> samples);

/// A hard iron fit, and how far it can be trusted.
struct HardIronFit {
    MagCalibration calibration;
    /// How widely the readings spread along their narrowest direction, µT:
    /// the standard deviation there. A device only ever turned about one
    /// axis leaves this near zero.
    float spread_ut;
};

/// Fits only the hard iron offset, keeping `matrix`: `magcal`'s `eye` fit,
/// with a known soft iron correction applied first.
///
/// Four unknowns need far less coverage than the nine of @ref fit, which is
/// what makes it possible from ordinary handling. The sphere `|y - c|² = β²`
/// is linear in `c` and `β² - |c|²`.
std::expected<HardIronFit, FitError> fit_hard_iron(std::span<const geo::Vec3> samples,
                                                   const std::array<std::array<float, 3>, 3>& matrix);

/// Works out the hard iron offset from ordinary handling after power up.
///
/// A full @ref fit needs readings from every direction, which only a
/// deliberate sequence gives. What changes from one session to the next is
/// the hard iron, though, and four unknowns can be found from the turning
/// and tilting a device gets as it is carried about. Soft iron stays as
/// `imu_cal` measured it, or uncorrected if it never has.
class StartupCalibration {
public:
    /// Starts collecting. `matrix` is the soft iron correction to keep;
    /// `std::nullopt` leaves the field as the hub reports it.
    explicit constexpr StartupCalibration(const std::optional<std::array<std::array<float, 3>, 3>>& matrix)
        : matrix_(matrix.value_or(kIdentity3))
    {
    }

    /// Takes an uncalibrated field reading. Returns a calibration the one
    /// time it has one it trusts; nothing after that.
    std::optional<MagCalibration> add(const geo::Vec3& raw);

private:
    Collector readings_;
    std::array<std::array<float, 3>, 3> matrix_;
    std::size_t next_attempt_ = kMinStartupSamples;
    bool done_ = false;

    /// The fewest kept readings worth a fit.
    static constexpr std::size_t kMinStartupSamples = 60;
};

/// A heading from @ref Compass.
struct Heading {
    /// The sensor's orientation in a frame of east, magnetic north and up,
    /// the same frame as the hub's rotation vector.
    geo::Quaternion quaternion;
    /// Estimated heading error, radians.
    float accuracy_rad;
    /// How far to trust it. @ref geo::Accuracy::kUnreliable whenever there is
    /// no calibration.
    geo::Accuracy status;
};

/// Turns the game rotation vector into a compass heading using the calibrated
/// magnetometer.
///
/// The game rotation vector has its Z axis up but its heading is wherever
/// the device happened to start. Each corrected field reading, rotated into
/// that frame, says where north lies in it; those are averaged, and the
/// orientation turned about Z by the difference. A field of the wrong
/// strength is something passing by, and is left out, so the gyroscope
/// carries the heading until it has gone.
///
/// Start a new one whenever the hub is reset: the frame starts over with it.
class Compass {
public:
    explicit constexpr Compass(const std::optional<MagCalibration>& calibration) : calibration_(calibration) {}

    /// Switches to a new calibration. Where north lies has to be worked out
    /// afresh, because the old estimate was made with the old correction.
    void set_calibration(const MagCalibration& calibration);

    /// Takes a game rotation vector report.
    void update_game(const geo::Quaternion& quaternion) { game_ = quaternion; }

    /// Takes an uncalibrated magnetic field report's field. Ignored until
    /// the first game rotation vector has arrived.
    void update_field(const geo::Vec3& raw);

    /// The latest orientation, once there is both a game rotation vector
    /// and an undisturbed field reading.
    [[nodiscard]] std::optional<Heading> heading() const;

private:
    std::optional<MagCalibration> calibration_;
    std::optional<geo::Quaternion> game_;
    float east_ = 0.0F;
    float north_ = 0.0F;
    std::uint32_t samples_ = 0;
    /// How far the field has lately been from the calibrated strength, µT.
    float deviation_ = 0.0F;
};

} // namespace ping::mag_cal
