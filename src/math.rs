//! Minimal f64 linear algebra.
//!
//! Deliberately dependency-free: photogrammetric blocks live in ECEF where
//! coordinates run to 6.4e6 metres, so everything here is f64 and nothing is
//! allowed to silently narrow to f32.

use std::fmt;
use std::ops::{Add, Div, Index, Mul, Neg, Sub};

// ---------------------------------------------------------------------------
// Vec3
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn splat(v: f64) -> Self {
        Self::new(v, v, v)
    }

    pub fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Self) -> Self {
        Self::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn normalize(self) -> Self {
        let n = self.length();
        if n == 0.0 {
            self
        } else {
            self / n
        }
    }

    pub fn min(self, o: Self) -> Self {
        Self::new(self.x.min(o.x), self.y.min(o.y), self.z.min(o.z))
    }

    pub fn max(self, o: Self) -> Self {
        Self::new(self.x.max(o.x), self.y.max(o.y), self.z.max(o.z))
    }

    pub fn max_element(self) -> f64 {
        self.x.max(self.y).max(self.z)
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    pub fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}

impl Index<usize> for Vec3 {
    type Output = f64;
    fn index(&self, i: usize) -> &f64 {
        match i {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Vec3 index {i} out of range"),
        }
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}
impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}
impl Div<f64> for Vec3 {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        Self::new(self.x / s, self.y / s, self.z / s)
    }
}

impl fmt::Display for Vec3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({:.3}, {:.3}, {:.3})", self.x, self.y, self.z)
    }
}

// ---------------------------------------------------------------------------
// Mat3 — row-major, matching the BlocksExchange M_ij layout
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3 {
    pub m: [[f64; 3]; 3],
}

impl Mat3 {
    pub const IDENTITY: Self = Self {
        m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    pub const fn from_rows(m: [[f64; 3]; 3]) -> Self {
        Self { m }
    }

    /// Diagonal matrix; used for the camera-convention axis flips.
    pub const fn diag(x: f64, y: f64, z: f64) -> Self {
        Self {
            m: [[x, 0.0, 0.0], [0.0, y, 0.0], [0.0, 0.0, z]],
        }
    }

    pub fn row(&self, i: usize) -> Vec3 {
        Vec3::new(self.m[i][0], self.m[i][1], self.m[i][2])
    }

    /// Part of the Mat3 surface for callers extending this; exercised by the
    /// tests but not on any current code path.
    #[allow(dead_code)]
    pub fn col(&self, j: usize) -> Vec3 {
        Vec3::new(self.m[0][j], self.m[1][j], self.m[2][j])
    }

    pub fn transpose(&self) -> Self {
        let m = &self.m;
        Self::from_rows([
            [m[0][0], m[1][0], m[2][0]],
            [m[0][1], m[1][1], m[2][1]],
            [m[0][2], m[1][2], m[2][2]],
        ])
    }

    pub fn det(&self) -> f64 {
        let m = &self.m;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// Nearest true rotation matrix, by Gram-Schmidt.
    ///
    /// XML stores the matrix as decimal text, so round-trip error leaves it
    /// very slightly non-orthogonal. Left uncorrected, that error propagates
    /// into the quaternion and shows up as a fractional-pixel reprojection
    /// bias across the whole block.
    pub fn orthonormalize(&self) -> Self {
        let r0 = self.row(0).normalize();
        let r1 = {
            let v = self.row(1);
            (v - r0 * r0.dot(v)).normalize()
        };
        let r2 = r0.cross(r1);
        let out = Self::from_rows([r0.to_array(), r1.to_array(), r2.to_array()]);
        if out.det() < 0.0 {
            Self::from_rows([r0.to_array(), r1.to_array(), (-r2).to_array()])
        } else {
            out
        }
    }

    /// Quaternion as (w, x, y, z). Shepperd's branch-stable method.
    pub fn to_quat(&self) -> [f64; 4] {
        let m = &self.m;
        let (m00, m01, m02) = (m[0][0], m[0][1], m[0][2]);
        let (m10, m11, m12) = (m[1][0], m[1][1], m[1][2]);
        let (m20, m21, m22) = (m[2][0], m[2][1], m[2][2]);
        let tr = m00 + m11 + m22;

        let q = if tr > 0.0 {
            let s = (tr + 1.0).sqrt() * 2.0;
            [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]
        } else if m00 > m11 && m00 > m22 {
            let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
            [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]
        } else if m11 > m22 {
            let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
            [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]
        } else {
            let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
            [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]
        };
        let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
    }

    /// Euler angles (omega, phi, kappa) in degrees, from a rotation matrix.
    /// Inverse of [`from_opk`], used for the CSV pose table.
    pub fn to_opk_degrees(&self) -> (f64, f64, f64) {
        // from_opk builds camera-to-world as Rx(omega) * Ry(phi) * Rz(kappa),
        // then transposes. Undo the transpose, then decompose in that order:
        //
        //   M[0][2] =  sin(phi)
        //   M[1][2] = -sin(omega) cos(phi)     M[2][2] = cos(omega) cos(phi)
        //   M[0][0] =  cos(phi)  cos(kappa)    M[0][1] = -cos(phi) sin(kappa)
        //
        // Decomposing for the wrong multiplication order yields angles that
        // look plausible and are silently wrong, so the order matters.
        let m = &self.transpose().m;
        let phi = m[0][2].clamp(-1.0, 1.0).asin();
        let (omega, kappa) = if m[0][2].abs() < 1.0 - 1e-12 {
            ((-m[1][2]).atan2(m[2][2]), (-m[0][1]).atan2(m[0][0]))
        } else {
            // Gimbal lock at phi = +/-90 degrees: omega and kappa are no
            // longer separable, so pin omega and fold everything into kappa.
            (0.0, m[1][0].atan2(m[1][1]))
        };
        (omega.to_degrees(), phi.to_degrees(), kappa.to_degrees())
    }

    /// ContextCapture omega/phi/kappa (degrees) to a world-to-camera matrix.
    pub fn from_opk(omega: f64, phi: f64, kappa: f64) -> Self {
        let (o, p, k) = (omega.to_radians(), phi.to_radians(), kappa.to_radians());
        let rx = Self::from_rows([
            [1.0, 0.0, 0.0],
            [0.0, o.cos(), -o.sin()],
            [0.0, o.sin(), o.cos()],
        ]);
        let ry = Self::from_rows([
            [p.cos(), 0.0, p.sin()],
            [0.0, 1.0, 0.0],
            [-p.sin(), 0.0, p.cos()],
        ]);
        let rz = Self::from_rows([
            [k.cos(), -k.sin(), 0.0],
            [k.sin(), k.cos(), 0.0],
            [0.0, 0.0, 1.0],
        ]);
        // Photogrammetric OPK is camera-to-world; transpose for world-to-camera.
        (rx * ry * rz).transpose()
    }
}

impl Mul for Mat3 {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        let mut r = [[0.0f64; 3]; 3];
        for (i, row) in r.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = (0..3).map(|k| self.m[i][k] * o.m[k][j]).sum();
            }
        }
        Self::from_rows(r)
    }
}

impl Mul<Vec3> for Mat3 {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 {
        Vec3::new(self.row(0).dot(v), self.row(1).dot(v), self.row(2).dot(v))
    }
}

// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn quat_roundtrip_is_exact() {
        // A rotation with no special structure, to exercise every branch.
        let r = Mat3::from_opk(17.3, -42.9, 121.5).orthonormalize();
        let q = r.to_quat();
        let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
        let back = Mat3::from_rows([
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]);
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    approx(r.m[i][j], back.m[i][j], 1e-12),
                    "element ({i},{j}): {} vs {}",
                    r.m[i][j],
                    back.m[i][j]
                );
            }
        }
    }

    #[test]
    fn quat_handles_180_degree_rotations() {
        // Trace <= 0 forces the non-trivial Shepperd branches.
        for d in [
            Mat3::diag(1.0, -1.0, -1.0),
            Mat3::diag(-1.0, 1.0, -1.0),
            Mat3::diag(-1.0, -1.0, 1.0),
        ] {
            let q = d.to_quat();
            let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
            assert!(approx(n, 1.0, 1e-12), "quaternion not unit: {q:?}");
        }
    }

    #[test]
    fn orthonormalize_repairs_perturbed_rotation() {
        let mut r = Mat3::from_opk(5.0, 10.0, 15.0);
        r.m[0][1] += 1e-4;
        r.m[2][2] -= 2e-4;
        let o = r.orthonormalize();
        assert!(approx(o.det(), 1.0, 1e-12));
        let should_be_identity = o * o.transpose();
        for i in 0..3 {
            for j in 0..3 {
                let want = if i == j { 1.0 } else { 0.0 };
                assert!(approx(should_be_identity.m[i][j], want, 1e-12));
            }
        }
    }

    #[test]
    fn row_and_col_are_transposes() {
        let m = Mat3::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        for i in 0..3 {
            assert_eq!(m.row(i), m.transpose().col(i));
            assert_eq!(m.col(i), m.transpose().row(i));
        }
    }

    #[test]
    fn opk_roundtrip() {
        for (o, p, k) in [(12.0, -7.0, 95.0), (-3.5, 44.0, -160.0), (0.0, 0.0, 0.0)] {
            let r = Mat3::from_opk(o, p, k);
            let (o2, p2, k2) = r.to_opk_degrees();
            assert!(approx(o, o2, 1e-9), "omega {o} vs {o2}");
            assert!(approx(p, p2, 1e-9), "phi {p} vs {p2}");
            assert!(approx(k, k2, 1e-9), "kappa {k} vs {k2}");
        }
    }
}
