#include "ping/mag_cal.hpp"

#include "ping/settings.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace ping::mag_cal {
namespace {

/// How far apart two kept readings must be, µT.
///
/// Readings taken while the device dwells in one pose would otherwise
/// outvote the rest and drag the fit towards them. At this spacing the
/// surface of a 49 µT sphere holds roughly 350 readings, safely under
/// @ref kCapacity.
constexpr float kMinSpacingUt = 8.0F;

/// How many readings make a sector count as covered.
constexpr std::uint16_t kMinPerSector = 3;

/// Where a component of a unit vector stops counting as about zero: sin 22.5°.
constexpr float kSectorEdge = 0.383F;

/// The fewest readings worth fitting nine parameters to.
constexpr std::size_t kMinFitSamples = 50;

/// How widely readings must spread along their narrowest direction for
/// StartupCalibration to trust a fit, µT. In simulated handling this takes
/// about two minutes of ordinary use with ±30° of tilt, or 40 seconds with
/// ±50°, and the offset then comes out within about 1 µT.
constexpr float kMinStartupSpreadUt = 7.0F;

/// Tighter than @ref kMaxResidual because this fit cannot correct soft iron:
/// one that leaves soft iron uncorrected can pass the looser limit and still
/// be several degrees out. Clean fits come in around a third of this.
constexpr float kMaxStartupResidual = 0.015F;

/// How many newly kept readings to wait for between attempts at a fit.
constexpr std::size_t kStartupRetryReadings = 10;

/// How many field readings the heading is averaged over, at
/// @ref ping::kMagReportIntervalUs: about three seconds. The game rotation
/// vector already carries the heading from moment to moment, so the
/// magnetometer only has to correct its slow drift.
constexpr std::uint32_t kHeadingAverage = 150;

/// How many field readings the disturbance estimate is averaged over.
constexpr float kDeviationAverage = 16.0F;

/// A field further than this from the calibrated strength, as a fraction, is
/// taken to be disturbed and is not allowed to move the heading.
constexpr float kDisturbed = 0.2F;

/// Field deviations, as fractions, up to which the compass reads high and
/// medium. Beyond them it reads low, up to @ref kDisturbed, then unreliable.
constexpr float kDeviationHigh = 0.05F;
constexpr float kDeviationMedium = 0.1F;

template <std::size_t N> using Matrix = std::array<std::array<double, N>, N>;
using Matrix3 = Matrix<3>;

/// Which of the @ref kSectors a direction falls in; none for a zero vector.
std::optional<std::size_t> sector(const std::array<float, 3>& d)
{
    const float length = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (length == 0.0F) {
        return std::nullopt;
    }
    std::size_t index = 0;
    for (const float component : d) {
        const float unit = component / length;
        const std::size_t digit = unit < -kSectorEdge ? 0 : (unit > kSectorEdge ? 2 : 1);
        index = index * 3 + digit;
    }
    // 13 is 1,1,1: about zero on every axis, which a unit vector cannot be.
    return index > 13 ? index - 1 : index;
}

/// Solves `a x = b` by Gaussian elimination with partial pivoting; none if
/// `a` is singular.
template <std::size_t N> std::optional<std::array<double, N>> solve(Matrix<N> a, std::array<double, N> b)
{
    for (std::size_t column = 0; column < N; ++column) {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < N; ++row) {
            if (std::abs(a[row][column]) > std::abs(a[pivot][column])) {
                pivot = row;
            }
        }
        if (std::abs(a[pivot][column]) < 1e-12) {
            return std::nullopt;
        }
        std::swap(a[column], a[pivot]);
        std::swap(b[column], b[pivot]);

        for (std::size_t row = column + 1; row < N; ++row) {
            const double factor = a[row][column] / a[column][column];
            for (std::size_t k = column; k < N; ++k) {
                a[row][k] -= factor * a[column][k];
            }
            b[row] -= factor * b[column];
        }
    }

    std::array<double, N> x{};
    for (std::size_t row = N; row-- > 0;) {
        double sum = b[row];
        for (std::size_t k = row + 1; k < N; ++k) {
            sum -= a[row][k] * x[k];
        }
        x[row] = sum / a[row][row];
    }
    return x;
}

struct Eigen {
    std::array<double, 3> values;
    /// Eigenvectors as columns.
    Matrix3 vectors;
};

/// Eigen decomposition of a symmetric 3×3 matrix, by Jacobi rotations.
Eigen symmetric_eigen(Matrix3 a)
{
    Matrix3 v{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
    constexpr std::array<std::pair<std::size_t, std::size_t>, 3> kPairs{{{0, 1}, {0, 2}, {1, 2}}};
    for (int sweep = 0; sweep < 50; ++sweep) {
        const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        if (off < 1e-24) {
            break;
        }
        for (const auto& [p, q] : kPairs) {
            if (a[p][q] == 0.0) {
                continue;
            }
            const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            const double sign = theta >= 0.0 ? 1.0 : -1.0;
            const double t = sign / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
            const double c = 1.0 / std::sqrt(t * t + 1.0);
            const double s = t * c;
            Matrix3 rotated = a;
            for (std::size_t k = 0; k < 3; ++k) {
                rotated[k][p] = c * a[k][p] - s * a[k][q];
                rotated[k][q] = s * a[k][p] + c * a[k][q];
            }
            a = rotated;
            for (std::size_t k = 0; k < 3; ++k) {
                rotated[p][k] = c * a[p][k] - s * a[q][k];
                rotated[q][k] = s * a[p][k] + c * a[q][k];
            }
            a = rotated;
            for (auto& row : v) {
                const double vp = row[p];
                const double vq = row[q];
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    return {.values = {a[0][0], a[1][1], a[2][2]}, .vectors = v};
}

/// Rotates `v` by the unit quaternion `q`.
geo::Vec3 rotate(const geo::Quaternion& q, const geo::Vec3& v)
{
    const float i = q.i;
    const float j = q.j;
    const float k = q.k;
    const float w = q.real;
    // t = 2 u × v; v' = v + w t + u × t
    const float t[3] = {2.0F * (j * v.z - k * v.y), 2.0F * (k * v.x - i * v.z), 2.0F * (i * v.y - j * v.x)};
    return {
        .x = v.x + w * t[0] + j * t[2] - k * t[1],
        .y = v.y + w * t[1] + k * t[0] - i * t[2],
        .z = v.z + w * t[2] + i * t[1] - j * t[0],
    };
}

std::array<double, 3> to_f64(const geo::Vec3& s)
{
    return {static_cast<double>(s.x), static_cast<double>(s.y), static_cast<double>(s.z)};
}

} // namespace

std::size_t coverage(std::span<const geo::Vec3> samples, const std::array<float, 3>& centre)
{
    std::array<std::uint16_t, kSectors> counts{};
    for (const auto& s : samples) {
        if (const auto cell = sector({s.x - centre[0], s.y - centre[1], s.z - centre[2]})) {
            ++counts[*cell];
        }
    }
    return static_cast<std::size_t>(std::ranges::count_if(counts, [](std::uint16_t n) { return n >= kMinPerSector; }));
}

bool Collector::add(const geo::Vec3& field)
{
    if (len_ == kCapacity || !(std::isfinite(field.x) && std::isfinite(field.y) && std::isfinite(field.z))) {
        return false;
    }
    constexpr float kLimit = kMinSpacingUt * kMinSpacingUt;
    const bool crowded = std::ranges::any_of(samples(), [&](const geo::Vec3& s) {
        const float dx = s.x - field.x;
        const float dy = s.y - field.y;
        const float dz = s.z - field.z;
        return dx * dx + dy * dy + dz * dz < kLimit;
    });
    if (crowded) {
        return false;
    }

    samples_[len_++] = field;
    const float values[3] = {field.x, field.y, field.z};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        min_[axis] = std::min(min_[axis], values[axis]);
        max_[axis] = std::max(max_[axis], values[axis]);
    }
    return true;
}

std::size_t Collector::covered() const
{
    if (len_ == 0) {
        return 0;
    }
    const std::array<float, 3> centre{
        (min_[0] + max_[0]) / 2.0F,
        (min_[1] + max_[1]) / 2.0F,
        (min_[2] + max_[2]) / 2.0F,
    };
    return coverage(samples(), centre);
}

std::expected<MagCalibration, FitError> fit(std::span<const geo::Vec3> samples)
{
    if (samples.size() < kMinFitSamples) {
        return std::unexpected(FitError::kTooFewSamples);
    }

    // Moved to the origin and scaled to about 1 first: a large hard iron
    // offset would otherwise leave the normal equations badly conditioned.
    const auto count = static_cast<double>(samples.size());
    std::array<double, 3> mean{};
    for (const auto& s : samples) {
        const auto x = to_f64(s);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            mean[axis] += x[axis] / count;
        }
    }
    const auto shifted = [&](const geo::Vec3& s) {
        const auto x = to_f64(s);
        return std::array<double, 3>{x[0] - mean[0], x[1] - mean[1], x[2] - mean[2]};
    };
    double scale = 0.0;
    for (const auto& s : samples) {
        for (const double component : shifted(s)) {
            scale = std::max(scale, std::abs(component));
        }
    }
    if (scale == 0.0) {
        return std::unexpected(FitError::kSingular);
    }

    Matrix<9> ata{};
    std::array<double, 9> atb{};
    for (const auto& s : samples) {
        const auto p = shifted(s);
        const double x = p[0] / scale;
        const double y = p[1] / scale;
        const double z = p[2] / scale;
        const double xx = x * x;
        const double yy = y * y;
        const double zz = z * z;
        const std::array<double, 9> row{
            xx + yy - 2.0 * zz, xx + zz - 2.0 * yy, 2.0 * x * y, 2.0 * x * z, 2.0 * y * z, 2.0 * x, 2.0 * y, 2.0 * z,
            1.0,
        };
        const double target = xx + yy + zz;
        for (std::size_t i = 0; i < 9; ++i) {
            for (std::size_t j = 0; j < 9; ++j) {
                ata[i][j] += row[i] * row[j];
            }
            atb[i] += row[i] * target;
        }
    }
    const auto solved = solve(ata, atb);
    if (!solved) {
        return std::unexpected(FitError::kSingular);
    }
    const auto& u = *solved;

    const Matrix3 q{{
        {1.0 - u[0] - u[1], -u[2], -u[3]},
        {-u[2], 1.0 - u[0] + 2.0 * u[1], -u[4]},
        {-u[3], -u[4], 1.0 + 2.0 * u[0] - u[1]},
    }};
    const std::array<double, 3> n{-u[5], -u[6], -u[7]};
    const double d = -u[8];

    const auto [eigenvalues, v] = symmetric_eigen(q);
    if (std::ranges::any_of(eigenvalues, [](double l) { return l <= 0.0; })) {
        return std::unexpected(FitError::kNotAnEllipsoid);
    }

    // The centre is where the gradient vanishes, c = -Q⁻¹n, worked out
    // through the eigenvectors since they are to hand.
    std::array<double, 3> centre{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const double along = v[0][axis] * n[0] + v[1][axis] * n[1] + v[2][axis] * n[2];
        for (std::size_t row = 0; row < 3; ++row) {
            centre[row] -= v[row][axis] * along / eigenvalues[axis];
        }
    }
    double k = -d;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            k += centre[row] * q[row][column] * centre[column];
        }
    }
    if (k <= 0.0) {
        return std::unexpected(FitError::kNotAnEllipsoid);
    }

    // det(A) = 1 keeps A from changing the field strength on average, so
    // the size of the sphere stays in β.
    const double determinant = eigenvalues[0] * eigenvalues[1] * eigenvalues[2];
    const double unit_scale = std::sqrt(1.0 / std::cbrt(determinant));
    Matrix3 a{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            for (std::size_t axis = 0; axis < 3; ++axis) {
                a[row][column] += v[row][axis] * std::sqrt(eigenvalues[axis]) * unit_scale * v[column][axis];
            }
        }
    }
    const double beta = std::sqrt(k / std::cbrt(determinant)) * scale;
    const std::array<double, 3> offset{
        centre[0] * scale + mean[0],
        centre[1] * scale + mean[1],
        centre[2] * scale + mean[2],
    };

    double sum = 0.0;
    for (const auto& s : samples) {
        const auto x = to_f64(s);
        const std::array<double, 3> raw{x[0] - offset[0], x[1] - offset[1], x[2] - offset[2]};
        double squared = 0.0;
        for (const auto& r : a) {
            const double m = r[0] * raw[0] + r[1] * raw[1] + r[2] * raw[2];
            squared += m * m;
        }
        const double excess = squared - beta * beta;
        sum += excess * excess;
    }
    const double residual = std::sqrt(sum / count) / (2.0 * beta * beta);

    MagCalibration result{};
    for (std::size_t row = 0; row < 3; ++row) {
        result.offset[row] = static_cast<float>(offset[row]);
        for (std::size_t column = 0; column < 3; ++column) {
            result.matrix[row][column] = static_cast<float>(a[row][column]);
        }
    }
    result.field_ut = static_cast<float>(beta);
    result.residual = static_cast<float>(residual);
    return result;
}

std::expected<HardIronFit, FitError> fit_hard_iron(std::span<const geo::Vec3> samples,
                                                   const std::array<std::array<float, 3>, 3>& matrix)
{
    if (samples.size() < kMinFitSamples) {
        return std::unexpected(FitError::kTooFewSamples);
    }
    Matrix3 a{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            a[row][column] = static_cast<double>(matrix[row][column]);
        }
    }
    const auto transformed = [&](const geo::Vec3& s) {
        const auto x = to_f64(s);
        return std::array<double, 3>{
            a[0][0] * x[0] + a[0][1] * x[1] + a[0][2] * x[2],
            a[1][0] * x[0] + a[1][1] * x[1] + a[1][2] * x[2],
            a[2][0] * x[0] + a[2][1] * x[1] + a[2][2] * x[2],
        };
    };

    const auto count = static_cast<double>(samples.size());
    std::array<double, 3> mean{};
    for (const auto& s : samples) {
        const auto y = transformed(s);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            mean[axis] += y[axis] / count;
        }
    }
    Matrix3 covariance{};
    Matrix<4> ata{};
    std::array<double, 4> atb{};
    constexpr double kScale = 50.0;
    for (const auto& s : samples) {
        const auto y = transformed(s);
        const std::array<double, 3> d{y[0] - mean[0], y[1] - mean[1], y[2] - mean[2]};
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                covariance[i][j] += d[i] * d[j] / count;
            }
        }
        const std::array<double, 3> u{d[0] / kScale, d[1] / kScale, d[2] / kScale};
        const std::array<double, 4> row{2.0 * u[0], 2.0 * u[1], 2.0 * u[2], 1.0};
        const double target = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
        for (std::size_t i = 0; i < 4; ++i) {
            for (std::size_t j = 0; j < 4; ++j) {
                ata[i][j] += row[i] * row[j];
            }
            atb[i] += row[i] * target;
        }
    }
    const auto solution = solve(ata, atb);
    if (!solution) {
        return std::unexpected(FitError::kSingular);
    }
    const std::array<double, 3> c{(*solution)[0], (*solution)[1], (*solution)[2]};
    const double squared = (*solution)[3] + c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    if (squared <= 0.0) {
        return std::unexpected(FitError::kNotAnEllipsoid);
    }
    const double beta = std::sqrt(squared) * kScale;
    const std::array<double, 3> centre{
        c[0] * kScale + mean[0],
        c[1] * kScale + mean[1],
        c[2] * kScale + mean[2],
    };

    // b = A⁻¹ c, by cofactors.
    const Matrix3& m = a;
    const Matrix3 cof{{
        {
            m[1][1] * m[2][2] - m[1][2] * m[2][1],
            m[0][2] * m[2][1] - m[0][1] * m[2][2],
            m[0][1] * m[1][2] - m[0][2] * m[1][1],
        },
        {
            m[1][2] * m[2][0] - m[1][0] * m[2][2],
            m[0][0] * m[2][2] - m[0][2] * m[2][0],
            m[0][2] * m[1][0] - m[0][0] * m[1][2],
        },
        {
            m[1][0] * m[2][1] - m[1][1] * m[2][0],
            m[0][1] * m[2][0] - m[0][0] * m[2][1],
            m[0][0] * m[1][1] - m[0][1] * m[1][0],
        },
    }};
    const double det = m[0][0] * cof[0][0] + m[0][1] * cof[1][0] + m[0][2] * cof[2][0];
    if (det == 0.0) {
        return std::unexpected(FitError::kSingular);
    }

    double sum = 0.0;
    for (const auto& s : samples) {
        const auto y = transformed(s);
        const std::array<double, 3> d{y[0] - centre[0], y[1] - centre[1], y[2] - centre[2]};
        const double excess = d[0] * d[0] + d[1] * d[1] + d[2] * d[2] - beta * beta;
        sum += excess * excess;
    }
    const auto eigen = symmetric_eigen(covariance);
    const double narrowest = std::max(std::min({eigen.values[0], eigen.values[1], eigen.values[2]}), 0.0);

    HardIronFit result{};
    for (std::size_t row = 0; row < 3; ++row) {
        result.calibration.offset[row] = static_cast<float>(
            (cof[row][0] * centre[0] + cof[row][1] * centre[1] + cof[row][2] * centre[2]) / det);
    }
    result.calibration.matrix = matrix;
    result.calibration.field_ut = static_cast<float>(beta);
    result.calibration.residual = static_cast<float>(std::sqrt(sum / count) / (2.0 * beta * beta));
    result.spread_ut = static_cast<float>(std::sqrt(narrowest));
    return result;
}

std::optional<MagCalibration> StartupCalibration::add(const geo::Vec3& raw)
{
    if (done_ || !readings_.add(raw)) {
        return std::nullopt;
    }
    const auto samples = readings_.samples();
    if (samples.size() < next_attempt_) {
        return std::nullopt;
    }
    next_attempt_ = samples.size() + kStartupRetryReadings;

    const auto found = fit_hard_iron(samples, matrix_);
    if (!found) {
        return std::nullopt;
    }
    const MagCalibration& calibration = found->calibration;
    const bool trusted = found->spread_ut >= kMinStartupSpreadUt && calibration.residual <= kMaxStartupResidual &&
                         std::abs(calibration.field_ut - kExpectedFieldUt) <= kFieldTolerance * kExpectedFieldUt;
    if (!trusted) {
        return std::nullopt;
    }
    done_ = true;
    return calibration;
}

void Compass::set_calibration(const MagCalibration& calibration)
{
    calibration_ = calibration;
    east_ = 0.0F;
    north_ = 0.0F;
    samples_ = 0;
}

void Compass::update_field(const geo::Vec3& raw)
{
    if (!game_) {
        return;
    }
    const geo::Vec3 field = calibration_ ? calibration_->apply(raw) : raw;
    const float strength = std::sqrt(field.x * field.x + field.y * field.y + field.z * field.z);

    const float deviation = calibration_ ? std::abs(strength - calibration_->field_ut) : 0.0F;
    if (samples_ == 0) {
        deviation_ = deviation;
    } else {
        deviation_ += (deviation - deviation_) / kDeviationAverage;
    }
    if (calibration_ && deviation > kDisturbed * calibration_->field_ut) {
        return;
    }

    // A plain average to begin with, so the first reading counts in full,
    // then a moving one.
    samples_ = std::min(samples_ + 1, kHeadingAverage);
    const float weight = 1.0F / static_cast<float>(samples_);
    const geo::Vec3 world = rotate(*game_, field);
    east_ += (world.x - east_) * weight;
    north_ += (world.y - north_) * weight;
}

std::optional<Heading> Compass::heading() const
{
    if (!game_ || samples_ == 0) {
        return std::nullopt;
    }
    const geo::Quaternion& game = *game_;

    // Turning the frame by this much about Z, counter-clockwise, puts
    // north's horizontal part on +Y.
    const float half = std::atan2(east_, north_) / 2.0F;
    const float s = std::sin(half);
    const float c = std::cos(half);
    Heading result{
        .quaternion =
            {
                .i = c * game.i - s * game.j,
                .j = c * game.j + s * game.i,
                .k = c * game.k + s * game.real,
                .real = c * game.real - s * game.k,
            },
        .accuracy_rad = std::numbers::pi_v<float>,
        .status = geo::Accuracy::kUnreliable,
    };

    if (calibration_) {
        const float share = deviation_ / calibration_->field_ut;
        if (share <= kDeviationHigh) {
            result.status = geo::Accuracy::kHigh;
        } else if (share <= kDeviationMedium) {
            result.status = geo::Accuracy::kMedium;
        } else if (share <= kDisturbed) {
            result.status = geo::Accuracy::kLow;
        }
        const float horizontal = std::sqrt(east_ * east_ + north_ * north_);
        const float error_ut = calibration_->residual * calibration_->field_ut + deviation_;
        result.accuracy_rad = std::atan2(error_ut, horizontal);
    }
    return result;
}

} // namespace ping::mag_cal
