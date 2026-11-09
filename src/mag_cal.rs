//! Magnetometer calibration on the ESP32, and the compass heading built on it.
//!
//! Turned through every orientation, a perfect magnetometer's readings lie on
//! a sphere around the origin whose radius is the strength of Earth's field.
//! Magnets that ride along with the sensor (hard iron) move the centre of
//! that sphere; iron that bends the field (soft iron) stretches and shears it
//! into an ellipsoid. So the raw readings are modelled as an offset ellipsoid,
//!
//! ```text
//! (x - b)ᵀ R (x - b) = β²
//! ```
//!
//! and undoing it is `m = A (x - b)`, with `b` the hard iron offset, `A` the
//! symmetric square root of `R` scaled to determinant 1, and `β` the field
//! strength the data implies. This is the form MATLAB's `magcal` uses with
//! its `sym` fit, and its residual is the quality figure here too.
//!
//! The BNO085 has no way to accept a correction like this, and its rotation
//! vector only ever uses the hub's own magnetometer calibration. So
//! [`Compass`] takes the hub's game rotation vector, which is gyroscope and
//! accelerometer only, and works out on the ESP32 which way north lies in
//! that frame.
//!
//! Nothing here touches hardware. `imu_cal` runs the sequence that feeds
//! [`Collector`] and [`fit`], and prints the [`MagCalibration`] to put in
//! `constants.rs`.

use libm::{atan2f, cbrt, cosf, sinf, sqrt, sqrtf};

use crate::bno085::{Accuracy, Quaternion, Vec3};
use crate::constants::EXPECTED_FIELD_UT;

/// How many readings [`Collector`] keeps.
pub const CAPACITY: usize = 512;

/// How far apart two kept readings must be, µT.
///
/// Readings taken while the device dwells in one pose would otherwise
/// outvote the rest and drag the fit towards them. At this spacing the
/// surface of a 49 µT sphere holds roughly 350 readings, safely under
/// [`CAPACITY`].
const MIN_SPACING_UT: f32 = 8.0;

/// The directions the readings are sorted into to judge coverage: each axis
/// divided into negative, about zero, and positive gives 3³ cells, less the
/// empty middle one.
pub const SECTORS: usize = 26;

/// How many [`SECTORS`] must hold readings. A little short of all of them,
/// so a pose the USB cable makes awkward does not hold up the whole run.
pub const REQUIRED_SECTORS: usize = 24;

/// How many readings make a sector count as covered.
const MIN_PER_SECTOR: u16 = 3;

/// Where a component of a unit vector stops counting as about zero: sin 22.5°.
const SECTOR_EDGE: f32 = 0.383;

/// The fewest readings worth fitting nine parameters to.
const MIN_FIT_SAMPLES: usize = 50;

/// The largest acceptable residual, as a fraction of the field: about 3%, or
/// 1.5 µT. A third of that is normal for a clean fit.
pub const MAX_RESIDUAL: f32 = 0.03;

/// How far the fitted field strength may sit from Earth's, as a fraction.
pub const FIELD_TOLERANCE: f32 = 0.2;

/// The fewest kept readings [`StartupCalibration`] will fit.
const MIN_STARTUP_SAMPLES: usize = 60;

/// How widely readings must spread along their narrowest direction for
/// [`StartupCalibration`] to trust a fit, µT. In simulated handling this
/// takes about two minutes of ordinary use with ±30° of tilt, or 40 seconds
/// with ±50°, and the offset then comes out within about 1 µT.
const MIN_STARTUP_SPREAD_UT: f32 = 7.0;

/// The largest residual [`StartupCalibration`] accepts. Tighter than
/// [`MAX_RESIDUAL`] because this fit cannot correct soft iron: one that
/// leaves soft iron uncorrected can pass the looser limit and still be
/// several degrees out. Clean fits come in around a third of this.
const MAX_STARTUP_RESIDUAL: f32 = 0.015;

/// How many newly kept readings to wait for between attempts at a fit.
const STARTUP_RETRY_READINGS: usize = 10;

/// How many field readings the heading is averaged over, at
/// [`MAG_REPORT_INTERVAL_US`](crate::constants::MAG_REPORT_INTERVAL_US): about
/// three seconds. The game rotation vector already carries the heading from
/// moment to moment, so the magnetometer only has to correct its slow drift.
const HEADING_AVERAGE: u32 = 150;

/// How many field readings the disturbance estimate is averaged over.
const DEVIATION_AVERAGE: f32 = 16.0;

/// A field further than this from the calibrated strength, as a fraction, is
/// taken to be disturbed and is not allowed to move the heading.
const DISTURBED: f32 = 0.2;

/// Field deviations, as fractions, up to which the compass reads high and
/// medium. Beyond them it reads low, up to [`DISTURBED`], then unreliable.
const DEVIATION_HIGH: f32 = 0.05;
const DEVIATION_MEDIUM: f32 = 0.1;

/// Why a set of readings gave no calibration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FitError {
    /// Too few readings to fit.
    TooFewSamples,
    /// The readings do not pin the fit down: they lie in a plane or on one
    /// small patch.
    Singular,
    /// The surface that fits best is not an ellipsoid.
    NotAnEllipsoid,
}

/// The correction for the magnetometer: `m = A (x - b)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagCalibration {
    /// `b`, the hard iron offset, µT.
    pub offset: [f32; 3],
    /// `A`, the soft iron correction: symmetric, determinant 1.
    pub matrix: [[f32; 3]; 3],
    /// `β`, the field strength the readings imply, µT.
    pub field_ut: f32,
    /// How far corrected readings sit from a sphere of radius `β`, as a
    /// fraction of it: `magcal`'s `E`, the root mean square of
    /// `(|m|² - β²) / 2β²`.
    pub residual: f32,
}

impl MagCalibration {
    /// Corrects a raw reading.
    pub fn apply(&self, raw: Vec3) -> Vec3 {
        let d = [
            raw.x - self.offset[0],
            raw.y - self.offset[1],
            raw.z - self.offset[2],
        ];
        let row = |r: [f32; 3]| r[0] * d[0] + r[1] * d[1] + r[2] * d[2];
        Vec3 {
            x: row(self.matrix[0]),
            y: row(self.matrix[1]),
            z: row(self.matrix[2]),
        }
    }
}

/// Which of the [`SECTORS`] a direction falls in. `None` for a zero vector.
fn sector(d: [f32; 3]) -> Option<usize> {
    let length = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if length == 0.0 {
        return None;
    }
    let mut index = 0;
    for component in d {
        let unit = component / length;
        let digit = if unit < -SECTOR_EDGE {
            0
        } else if unit > SECTOR_EDGE {
            2
        } else {
            1
        };
        index = index * 3 + digit;
    }
    // 13 is 1,1,1: about zero on every axis, which a unit vector cannot be.
    Some(if index > 13 { index - 1 } else { index })
}

/// How many [`SECTORS`] around `centre` the readings cover.
pub fn coverage(samples: &[Vec3], centre: [f32; 3]) -> usize {
    let mut counts = [0u16; SECTORS];
    for s in samples {
        if let Some(cell) = sector([s.x - centre[0], s.y - centre[1], s.z - centre[2]]) {
            counts[cell] += 1;
        }
    }
    counts.iter().filter(|&&n| n >= MIN_PER_SECTOR).count()
}

/// Keeps readings spread evenly over the ellipsoid.
pub struct Collector {
    samples: [Vec3; CAPACITY],
    len: usize,
    min: [f32; 3],
    max: [f32; 3],
}

impl Collector {
    pub const fn new() -> Self {
        Self {
            samples: [Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }; CAPACITY],
            len: 0,
            min: [f32::MAX; 3],
            max: [f32::MIN; 3],
        }
    }

    /// Keeps `field` unless a reading close to it is already held. Returns
    /// whether it was kept.
    pub fn add(&mut self, field: Vec3) -> bool {
        if self.len == CAPACITY
            || !(field.x.is_finite() && field.y.is_finite() && field.z.is_finite())
        {
            return false;
        }
        let limit = MIN_SPACING_UT * MIN_SPACING_UT;
        let crowded = self.samples[..self.len].iter().any(|s| {
            let (dx, dy, dz) = (s.x - field.x, s.y - field.y, s.z - field.z);
            dx * dx + dy * dy + dz * dz < limit
        });
        if crowded {
            return false;
        }

        self.samples[self.len] = field;
        self.len += 1;
        for (axis, value) in [field.x, field.y, field.z].into_iter().enumerate() {
            self.min[axis] = self.min[axis].min(value);
            self.max[axis] = self.max[axis].max(value);
        }
        true
    }

    pub fn samples(&self) -> &[Vec3] {
        &self.samples[..self.len]
    }

    pub fn is_full(&self) -> bool {
        self.len == CAPACITY
    }

    /// How many [`SECTORS`] are covered so far, judged from the middle of
    /// the range seen on each axis: good enough to steer the operator before
    /// there is a fit to measure from.
    pub fn covered(&self) -> usize {
        if self.len == 0 {
            return 0;
        }
        let centre = [
            (self.min[0] + self.max[0]) / 2.0,
            (self.min[1] + self.max[1]) / 2.0,
            (self.min[2] + self.max[2]) / 2.0,
        ];
        coverage(self.samples(), centre)
    }
}

impl Default for Collector {
    fn default() -> Self {
        Self::new()
    }
}

/// Solves `a x = b` by Gaussian elimination with partial pivoting. `None` if
/// `a` is singular.
fn solve<const N: usize>(mut a: [[f64; N]; N], mut b: [f64; N]) -> Option<[f64; N]> {
    for column in 0..N {
        let mut pivot = column;
        for row in column + 1..N {
            if a[row][column].abs() > a[pivot][column].abs() {
                pivot = row;
            }
        }
        if a[pivot][column].abs() < 1e-12 {
            return None;
        }
        a.swap(column, pivot);
        b.swap(column, pivot);

        for row in column + 1..N {
            let factor = a[row][column] / a[column][column];
            let pivot_row = a[column];
            for (target, value) in a[row].iter_mut().zip(pivot_row).skip(column) {
                *target -= factor * value;
            }
            b[row] -= factor * b[column];
        }
    }

    let mut x = [0.0; N];
    for row in (0..N).rev() {
        let mut sum = b[row];
        for k in row + 1..N {
            sum -= a[row][k] * x[k];
        }
        x[row] = sum / a[row][row];
    }
    Some(x)
}

/// Eigenvalues and eigenvectors (as columns) of a symmetric 3×3 matrix, by
/// Jacobi rotations.
fn symmetric_eigen(mut a: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        if off < 1e-24 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let sign = if theta >= 0.0 { 1.0 } else { -1.0 };
            let t = sign / (theta.abs() + sqrt(theta * theta + 1.0));
            let c = 1.0 / sqrt(t * t + 1.0);
            let s = t * c;
            let mut rotated = a;
            for k in 0..3 {
                rotated[k][p] = c * a[k][p] - s * a[k][q];
                rotated[k][q] = s * a[k][p] + c * a[k][q];
            }
            a = rotated;
            for k in 0..3 {
                rotated[p][k] = c * a[p][k] - s * a[q][k];
                rotated[q][k] = s * a[p][k] + c * a[q][k];
            }
            a = rotated;
            for row in &mut v {
                let (vp, vq) = (row[p], row[q]);
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    ([a[0][0], a[1][1], a[2][2]], v)
}

/// Fits an offset ellipsoid to `samples` and returns the correction that
/// maps it onto a sphere.
///
/// The ellipsoid is `xᵀQx + 2nᵀx + d = 0` with the trace of `Q` fixed at 3,
/// which turns the fit into nine unknowns and one linear least squares
/// problem. A fit is only an ellipsoid if `Q` comes out positive definite,
/// which readings that do not cover enough directions can prevent, so that
/// is checked rather than assumed.
pub fn fit(samples: &[Vec3]) -> Result<MagCalibration, FitError> {
    if samples.len() < MIN_FIT_SAMPLES {
        return Err(FitError::TooFewSamples);
    }

    // Moved to the origin and scaled to about 1 first: a large hard iron
    // offset would otherwise leave the normal equations badly conditioned.
    let count = samples.len() as f64;
    let mut mean = [0.0f64; 3];
    for s in samples {
        mean[0] += f64::from(s.x) / count;
        mean[1] += f64::from(s.y) / count;
        mean[2] += f64::from(s.z) / count;
    }
    let shifted = |s: &Vec3| {
        [
            f64::from(s.x) - mean[0],
            f64::from(s.y) - mean[1],
            f64::from(s.z) - mean[2],
        ]
    };
    let mut scale = 0.0f64;
    for s in samples {
        for component in shifted(s) {
            scale = scale.max(component.abs());
        }
    }
    if scale == 0.0 {
        return Err(FitError::Singular);
    }

    let mut ata = [[0.0f64; 9]; 9];
    let mut atb = [0.0f64; 9];
    for s in samples {
        let [x, y, z] = shifted(s).map(|c| c / scale);
        let (xx, yy, zz) = (x * x, y * y, z * z);
        let row = [
            xx + yy - 2.0 * zz,
            xx + zz - 2.0 * yy,
            2.0 * x * y,
            2.0 * x * z,
            2.0 * y * z,
            2.0 * x,
            2.0 * y,
            2.0 * z,
            1.0,
        ];
        let target = xx + yy + zz;
        for i in 0..9 {
            for j in 0..9 {
                ata[i][j] += row[i] * row[j];
            }
            atb[i] += row[i] * target;
        }
    }
    let u = solve(ata, atb).ok_or(FitError::Singular)?;

    let q = [
        [1.0 - u[0] - u[1], -u[2], -u[3]],
        [-u[2], 1.0 - u[0] + 2.0 * u[1], -u[4]],
        [-u[3], -u[4], 1.0 + 2.0 * u[0] - u[1]],
    ];
    let n = [-u[5], -u[6], -u[7]];
    let d = -u[8];

    let (eigenvalues, v) = symmetric_eigen(q);
    if eigenvalues.iter().any(|&l| l <= 0.0) {
        return Err(FitError::NotAnEllipsoid);
    }

    // The centre is where the gradient vanishes, c = -Q⁻¹n, worked out
    // through the eigenvectors since they are to hand.
    let mut centre = [0.0f64; 3];
    for axis in 0..3 {
        let along = v[0][axis] * n[0] + v[1][axis] * n[1] + v[2][axis] * n[2];
        for (row, c) in centre.iter_mut().enumerate() {
            *c -= v[row][axis] * along / eigenvalues[axis];
        }
    }
    let mut k = -d;
    for row in 0..3 {
        for column in 0..3 {
            k += centre[row] * q[row][column] * centre[column];
        }
    }
    if k <= 0.0 {
        return Err(FitError::NotAnEllipsoid);
    }

    // det(A) = 1 keeps A from changing the field strength on average, so
    // the size of the sphere stays in β.
    let determinant = eigenvalues[0] * eigenvalues[1] * eigenvalues[2];
    let unit_scale = sqrt(1.0 / cbrt(determinant));
    let mut a = [[0.0f64; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            for axis in 0..3 {
                a[row][column] +=
                    v[row][axis] * sqrt(eigenvalues[axis]) * unit_scale * v[column][axis];
            }
        }
    }
    let beta = sqrt(k / cbrt(determinant)) * scale;
    let offset = [
        centre[0] * scale + mean[0],
        centre[1] * scale + mean[1],
        centre[2] * scale + mean[2],
    ];

    let mut sum = 0.0;
    for s in samples {
        let raw = [
            f64::from(s.x) - offset[0],
            f64::from(s.y) - offset[1],
            f64::from(s.z) - offset[2],
        ];
        let mut squared = 0.0;
        for r in &a {
            let m = r[0] * raw[0] + r[1] * raw[1] + r[2] * raw[2];
            squared += m * m;
        }
        let excess = squared - beta * beta;
        sum += excess * excess;
    }
    let residual = sqrt(sum / count) / (2.0 * beta * beta);

    Ok(MagCalibration {
        offset: offset.map(|c| c as f32),
        matrix: a.map(|r| r.map(|c| c as f32)),
        field_ut: beta as f32,
        residual: residual as f32,
    })
}

/// A hard iron fit, and how far it can be trusted.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HardIronFit {
    pub calibration: MagCalibration,
    /// How widely the readings spread along their narrowest direction, µT:
    /// the standard deviation there. A device only ever turned about one
    /// axis leaves this near zero.
    pub spread_ut: f32,
}

/// Fits only the hard iron offset, keeping `matrix`: `magcal`'s `eye` fit,
/// with a known soft iron correction applied first.
///
/// Four unknowns need far less coverage than the nine of [`fit`], which is
/// what makes it possible from ordinary handling. The sphere `|y - c|² = β²`
/// is linear in `c` and `β² - |c|²`.
pub fn fit_hard_iron(samples: &[Vec3], matrix: [[f32; 3]; 3]) -> Result<HardIronFit, FitError> {
    if samples.len() < MIN_FIT_SAMPLES {
        return Err(FitError::TooFewSamples);
    }
    let a = matrix.map(|r| r.map(f64::from));
    let transformed = |s: &Vec3| {
        let x = [f64::from(s.x), f64::from(s.y), f64::from(s.z)];
        [
            a[0][0] * x[0] + a[0][1] * x[1] + a[0][2] * x[2],
            a[1][0] * x[0] + a[1][1] * x[1] + a[1][2] * x[2],
            a[2][0] * x[0] + a[2][1] * x[1] + a[2][2] * x[2],
        ]
    };

    let count = samples.len() as f64;
    let mut mean = [0.0f64; 3];
    for s in samples {
        let y = transformed(s);
        for axis in 0..3 {
            mean[axis] += y[axis] / count;
        }
    }
    let mut covariance = [[0.0f64; 3]; 3];
    let mut ata = [[0.0f64; 4]; 4];
    let mut atb = [0.0f64; 4];
    const SCALE: f64 = 50.0;
    for s in samples {
        let y = transformed(s);
        let d = [y[0] - mean[0], y[1] - mean[1], y[2] - mean[2]];
        for i in 0..3 {
            for j in 0..3 {
                covariance[i][j] += d[i] * d[j] / count;
            }
        }
        let u = d.map(|c| c / SCALE);
        let row = [2.0 * u[0], 2.0 * u[1], 2.0 * u[2], 1.0];
        let target = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
        for i in 0..4 {
            for j in 0..4 {
                ata[i][j] += row[i] * row[j];
            }
            atb[i] += row[i] * target;
        }
    }
    let solution = solve(ata, atb).ok_or(FitError::Singular)?;
    let c = [solution[0], solution[1], solution[2]];
    let squared = solution[3] + c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    if squared <= 0.0 {
        return Err(FitError::NotAnEllipsoid);
    }
    let beta = sqrt(squared) * SCALE;
    let centre = [
        c[0] * SCALE + mean[0],
        c[1] * SCALE + mean[1],
        c[2] * SCALE + mean[2],
    ];

    // b = A⁻¹ c, by cofactors.
    let m = a;
    let cof = [
        [
            m[1][1] * m[2][2] - m[1][2] * m[2][1],
            m[0][2] * m[2][1] - m[0][1] * m[2][2],
            m[0][1] * m[1][2] - m[0][2] * m[1][1],
        ],
        [
            m[1][2] * m[2][0] - m[1][0] * m[2][2],
            m[0][0] * m[2][2] - m[0][2] * m[2][0],
            m[0][2] * m[1][0] - m[0][0] * m[1][2],
        ],
        [
            m[1][0] * m[2][1] - m[1][1] * m[2][0],
            m[0][1] * m[2][0] - m[0][0] * m[2][1],
            m[0][0] * m[1][1] - m[0][1] * m[1][0],
        ],
    ];
    let det = m[0][0] * cof[0][0] + m[0][1] * cof[1][0] + m[0][2] * cof[2][0];
    if det == 0.0 {
        return Err(FitError::Singular);
    }
    let offset = [
        (cof[0][0] * centre[0] + cof[0][1] * centre[1] + cof[0][2] * centre[2]) / det,
        (cof[1][0] * centre[0] + cof[1][1] * centre[1] + cof[1][2] * centre[2]) / det,
        (cof[2][0] * centre[0] + cof[2][1] * centre[1] + cof[2][2] * centre[2]) / det,
    ];

    let mut sum = 0.0;
    for s in samples {
        let y = transformed(s);
        let d = [y[0] - centre[0], y[1] - centre[1], y[2] - centre[2]];
        let excess = d[0] * d[0] + d[1] * d[1] + d[2] * d[2] - beta * beta;
        sum += excess * excess;
    }
    let (eigenvalues, _) = symmetric_eigen(covariance);
    let narrowest = eigenvalues[0]
        .min(eigenvalues[1])
        .min(eigenvalues[2])
        .max(0.0);

    Ok(HardIronFit {
        calibration: MagCalibration {
            offset: offset.map(|v| v as f32),
            matrix,
            field_ut: beta as f32,
            residual: (sqrt(sum / count) / (2.0 * beta * beta)) as f32,
        },
        spread_ut: sqrt(narrowest) as f32,
    })
}

/// Works out the hard iron offset from ordinary handling after power up.
///
/// A full [`fit`] needs readings from every direction, which only a
/// deliberate sequence gives. What changes from one session to the next is
/// the hard iron, though, and four unknowns can be found from the turning
/// and tilting a device gets as it is carried about. Soft iron stays as
/// `imu_cal` measured it, or uncorrected if it never has.
pub struct StartupCalibration {
    readings: Collector,
    matrix: [[f32; 3]; 3],
    next_attempt: usize,
    done: bool,
}

impl StartupCalibration {
    /// Starts collecting. `matrix` is the soft iron correction to keep;
    /// `None` leaves the field as the hub reports it.
    pub const fn new(matrix: Option<[[f32; 3]; 3]>) -> Self {
        Self {
            readings: Collector::new(),
            matrix: match matrix {
                Some(matrix) => matrix,
                None => [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            },
            next_attempt: MIN_STARTUP_SAMPLES,
            done: false,
        }
    }

    /// Takes an uncalibrated field reading. Returns a calibration the one
    /// time it has one it trusts; nothing after that.
    pub fn add(&mut self, raw: Vec3) -> Option<MagCalibration> {
        if self.done || !self.readings.add(raw) {
            return None;
        }
        let samples = self.readings.samples();
        if samples.len() < self.next_attempt {
            return None;
        }
        self.next_attempt = samples.len() + STARTUP_RETRY_READINGS;

        let found = fit_hard_iron(samples, self.matrix).ok()?;
        let calibration = found.calibration;
        let trusted = found.spread_ut >= MIN_STARTUP_SPREAD_UT
            && calibration.residual <= MAX_STARTUP_RESIDUAL
            && (calibration.field_ut - EXPECTED_FIELD_UT).abs()
                <= FIELD_TOLERANCE * EXPECTED_FIELD_UT;
        if !trusted {
            return None;
        }
        self.done = true;
        Some(calibration)
    }
}

/// A heading from [`Compass`].
#[derive(Clone, Copy, Debug)]
pub struct Heading {
    /// The sensor's orientation in a frame of east, magnetic north and up,
    /// the same frame as the hub's rotation vector.
    pub quaternion: Quaternion,
    /// Estimated heading error, radians.
    pub accuracy_rad: f32,
    /// How far to trust it. [`Accuracy::Unreliable`] whenever there is no
    /// calibration.
    pub status: Accuracy,
}

/// Rotates `v` by the unit quaternion `q`.
fn rotate(q: &Quaternion, v: Vec3) -> Vec3 {
    let (i, j, k, w) = (q.i, q.j, q.k, q.real);
    // t = 2 u × v; v' = v + w t + u × t
    let t = [
        2.0 * (j * v.z - k * v.y),
        2.0 * (k * v.x - i * v.z),
        2.0 * (i * v.y - j * v.x),
    ];
    Vec3 {
        x: v.x + w * t[0] + j * t[2] - k * t[1],
        y: v.y + w * t[1] + k * t[0] - i * t[2],
        z: v.z + w * t[2] + i * t[1] - j * t[0],
    }
}

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
pub struct Compass {
    calibration: Option<MagCalibration>,
    game: Option<Quaternion>,
    east: f32,
    north: f32,
    samples: u32,
    /// How far the field has lately been from the calibrated strength, µT.
    deviation: f32,
}

impl Compass {
    pub const fn new(calibration: Option<MagCalibration>) -> Self {
        Self {
            calibration,
            game: None,
            east: 0.0,
            north: 0.0,
            samples: 0,
            deviation: 0.0,
        }
    }

    /// Switches to a new calibration. Where north lies has to be worked out
    /// afresh, because the old estimate was made with the old correction.
    pub fn set_calibration(&mut self, calibration: MagCalibration) {
        self.calibration = Some(calibration);
        self.east = 0.0;
        self.north = 0.0;
        self.samples = 0;
    }

    /// Takes a game rotation vector report.
    pub fn update_game(&mut self, quaternion: Quaternion) {
        self.game = Some(quaternion);
    }

    /// Takes an uncalibrated magnetic field report's field. Ignored until
    /// the first game rotation vector has arrived.
    pub fn update_field(&mut self, raw: Vec3) {
        let Some(game) = self.game else {
            return;
        };
        let field = match &self.calibration {
            Some(calibration) => calibration.apply(raw),
            None => raw,
        };
        let strength = sqrtf(field.x * field.x + field.y * field.y + field.z * field.z);

        let deviation = self
            .calibration
            .map_or(0.0, |c| (strength - c.field_ut).abs());
        if self.samples == 0 {
            self.deviation = deviation;
        } else {
            self.deviation += (deviation - self.deviation) / DEVIATION_AVERAGE;
        }
        if self
            .calibration
            .is_some_and(|c| deviation > DISTURBED * c.field_ut)
        {
            return;
        }

        // A plain average to begin with, so the first reading counts in
        // full, then a moving one.
        self.samples = (self.samples + 1).min(HEADING_AVERAGE);
        let weight = 1.0 / self.samples as f32;
        let world = rotate(&game, field);
        self.east += (world.x - self.east) * weight;
        self.north += (world.y - self.north) * weight;
    }

    /// The latest orientation, once there is both a game rotation vector
    /// and an undisturbed field reading.
    pub fn heading(&self) -> Option<Heading> {
        let game = self.game?;
        if self.samples == 0 {
            return None;
        }

        // Turning the frame by this much about Z, counter-clockwise, puts
        // north's horizontal part on +Y.
        let half = atan2f(self.east, self.north) / 2.0;
        let (s, c) = (sinf(half), cosf(half));
        let quaternion = Quaternion {
            i: c * game.i - s * game.j,
            j: c * game.j + s * game.i,
            k: c * game.k + s * game.real,
            real: c * game.real - s * game.k,
        };

        let (status, accuracy_rad) = match self.calibration {
            None => (Accuracy::Unreliable, core::f32::consts::PI),
            Some(c) => {
                let share = self.deviation / c.field_ut;
                let status = if share <= DEVIATION_HIGH {
                    Accuracy::High
                } else if share <= DEVIATION_MEDIUM {
                    Accuracy::Medium
                } else if share <= DISTURBED {
                    Accuracy::Low
                } else {
                    Accuracy::Unreliable
                };
                let horizontal = sqrtf(self.east * self.east + self.north * self.north);
                let error_ut = c.residual * c.field_ut + self.deviation;
                (status, atan2f(error_ut, horizontal))
            }
        };

        Some(Heading {
            quaternion,
            accuracy_rad,
            status,
        })
    }
}
