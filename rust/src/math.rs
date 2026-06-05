//! Vec3 / Quat math, ported from C++ `utils.h` + `utils.cpp`.
//!
//! Kept `f64` to preserve the existing AHRS behavior bit-for-bit. The RP2350
//! has only a single-precision FPU, so `f64` math is soft-float — same regime
//! as the RP2040 (M0+, no FPU at all), but with the M33 ALU it's noticeably
//! faster. See PORTING_PLAN.md for the f32 conversion option.

#![allow(clippy::many_single_char_names)]

use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

// libm provides no_std math intrinsics. On host (cargo test) we still use libm
// for bit-identical behavior with the firmware build.
use libm::{atan2, cos, sin, sqrt};
use uom::Kind;
use uom::si::angle::radian;
use uom::si::angular_acceleration::Units::radian_per_second_squared;
use uom::si::angular_velocity::radian_per_second;
use uom::si::f64::*;
use uom::num_traits::Float;
use uom::si::marker::AngleKind;
use uom::si::Quantity;

#[inline]
pub fn clamp<T: PartialOrd>(value: T, min: T, max: T) -> T {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngularVelocityVector {
    pub x: AngularVelocity,
    pub y: AngularVelocity,
    pub z: AngularVelocity,
}
impl AngularVelocityVector {
    #[inline]
    pub const fn new(x: AngularVelocity, y: AngularVelocity, z: AngularVelocity) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn norm(&self) -> AngularVelocity {
        let x = self.x.get::<radian_per_second>();
        let y = self.y.get::<radian_per_second>();
        let z = self.z.get::<radian_per_second>();
        AngularVelocity::new::<radian_per_second>((x * x + y * y + z * z).sqrt())
    }
}
impl Div<AngularVelocity> for AngularVelocityVector {
    type Output = Vec3;
    #[inline]
    fn div(self, s: AngularVelocity) -> Vec3 {
        Vec3::new(
            self.x.get::<radian_per_second>() / s.get::<radian_per_second>(),
            self.y.get::<radian_per_second>() / s.get::<radian_per_second>(),
            self.z.get::<radian_per_second>() / s.get::<radian_per_second>(),
        )
    }
}

impl Mul<Time> for AngularVelocity {
    type Output = Angle;

    fn mul(self, rhs: Time) -> Angle {
        Angle::new::<radian>(
            self.get::<radian_per_second>() * rhs.get::<second>()
        )
    }
}

// And the commutative case:
impl Mul<AngularVelocity> for Time {
    type Output = Angle;

    fn mul(self, rhs: AngularVelocity) -> Angle {
        rhs * self
    }
}

// ────────────────────────────────── Vec3 ────────────────────────────────────
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn dot(self, v: Vec3) -> f64 {
        self.x * v.x + self.y * v.y + self.z * v.z
    }

    #[inline]
    pub fn cross(self, v: Vec3) -> Vec3 {
        Vec3 {
            x: self.y * v.z - self.z * v.y,
            y: self.z * v.x - self.x * v.z,
            z: self.x * v.y - self.y * v.x,
        }
    }

    /// L2 norm. Named `norm3` in the C++ source.
    #[inline]
    pub fn norm(self) -> f64 {
        sqrt(self.dot(self))
    }

    #[inline]
    pub fn mag(self) -> f64 {
        self.norm()
    }

    /// Build a pure quaternion `[w, x, y, z]` from this vector.
    #[inline]
    pub fn to_quat(self, w: f64) -> Quat {
        Quat::new(w, self.x, self.y, self.z)
    }
}

// Scalar ops
impl Mul<f64> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}
impl MulAssign<f64> for Vec3 {
    #[inline]
    fn mul_assign(&mut self, s: f64) {
        self.x *= s;
        self.y *= s;
        self.z *= s;
    }
}
impl Div<f64> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn div(self, s: f64) -> Vec3 {
        Vec3::new(self.x / s, self.y / s, self.z / s)
    }
}
impl DivAssign<f64> for Vec3 {
    #[inline]
    fn div_assign(&mut self, s: f64) {
        self.x /= s;
        self.y /= s;
        self.z /= s;
    }
}
impl Add for Vec3 {
    type Output = Vec3;
    #[inline]
    fn add(self, v: Vec3) -> Vec3 {
        Vec3::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}
impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, v: Vec3) {
        self.x += v.x;
        self.y += v.y;
        self.z += v.z;
    }
}
impl Sub for Vec3 {
    type Output = Vec3;
    #[inline]
    fn sub(self, v: Vec3) -> Vec3 {
        Vec3::new(self.x - v.x, self.y - v.y, self.z - v.z)
    }
}
impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, v: Vec3) {
        self.x -= v.x;
        self.y -= v.y;
        self.z -= v.z;
    }
}

// ────────────────────────────────── Quat ────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quat { // TODO: Maybe use nalgebra quaternion?
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    #[inline]
    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    pub fn from_angle(w: f64, x_angle: Angle, y_angle: Angle, z_angle: Angle) -> Self {
        Self {
            w,
            x: x_angle.get::<radian>(),
            y: y_angle.get::<radian>(),
            z: z_angle.get::<radian>(),
        }
    }

    #[inline]
    pub fn norm(self) -> f64 {
        sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Hamilton-conjugate. For a unit quaternion this is also the inverse.
    #[inline]
    pub fn conjugate(self) -> Quat {
        Quat::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Returns a unit-length copy. Falls back to identity on near-zero norm
    /// (matches C++ behavior — keeps AHRS from blowing up on bad input).
    #[inline]
    pub fn normalized(self) -> Quat {
        let n = self.norm();
        if n <= 1e-15 {
            Quat::IDENTITY
        } else {
            Quat::new(self.w / n, self.x / n, self.y / n, self.z / n)
        }
    }
}

// Scalar ops (must come before Mul<Quat> so they don't shadow it)
impl Mul<f64> for Quat {
    type Output = Quat;
    #[inline]
    fn mul(self, s: f64) -> Quat {
        Quat::new(self.w * s, self.x * s, self.y * s, self.z * s)
    }
}
impl Div<f64> for Quat {
    type Output = Quat;
    #[inline]
    fn div(self, s: f64) -> Quat {
        Quat::new(self.w / s, self.x / s, self.y / s, self.z / s)
    }
}
impl DivAssign<f64> for Quat {
    #[inline]
    fn div_assign(&mut self, s: f64) {
        self.w /= s;
        self.x /= s;
        self.y /= s;
        self.z /= s;
    }
}
impl Add for Quat {
    type Output = Quat;
    #[inline]
    fn add(self, q: Quat) -> Quat {
        Quat::new(self.w + q.w, self.x + q.x, self.y + q.y, self.z + q.z)
    }
}
impl Sub for Quat {
    type Output = Quat;
    #[inline]
    fn sub(self, q: Quat) -> Quat {
        Quat::new(self.w - q.w, self.x - q.x, self.y - q.y, self.z - q.z)
    }
}

/// Hamilton product `p ⊗ q`. Matches the C++ definition exactly so AHRS
/// behavior is preserved.
impl Mul<Quat> for Quat {
    type Output = Quat;
    #[inline]
    fn mul(self, q: Quat) -> Quat {
        let p = self;
        Quat::new(
            p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
            p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
            p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x,
            p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w,
        )
    }
}

// ─────────────────── Madgwick gradient as a 4-tuple (Grad4) ─────────────────
/// Distinct type from `Quat` so we never accidentally feed a gradient into a
/// Hamilton product. Scalar ops are intentionally limited to what the
/// correction step actually needs.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Grad4 {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Grad4 {
    #[inline]
    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }
    #[inline]
    pub fn norm(self) -> f64 {
        sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }
}

impl Mul<f64> for Grad4 {
    type Output = Grad4;
    #[inline]
    fn mul(self, s: f64) -> Grad4 {
        Grad4::new(self.w * s, self.x * s, self.y * s, self.z * s)
    }
}
impl Add for Grad4 {
    type Output = Grad4;
    #[inline]
    fn add(self, g: Grad4) -> Grad4 {
        Grad4::new(self.w + g.w, self.x + g.x, self.y + g.y, self.z + g.z)
    }
}
impl DivAssign<f64> for Grad4 {
    #[inline]
    fn div_assign(&mut self, s: f64) {
        self.w /= s;
        self.x /= s;
        self.y /= s;
        self.z /= s;
    }
}

// ───────────────────── axis-angle and Euler conversions ─────────────────────
#[inline]
pub fn axis_angle_to_quat(axis: Vec3, angle: Angle) -> Quat {
    let half = angle.get::<radian>() * 0.5;
    let s = sin(half);
    Quat::new(cos(half), axis.x * s, axis.y * s, axis.z * s)
}

#[inline]
pub fn roll(q: Quat) -> Angle {
    Angle::new::<radian>(-atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    ))
}

#[inline]
pub fn pitch(q: Quat) -> Angle {
    Angle::new::<radian>(atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    ))
}

#[inline]
pub fn yaw(q: Quat) -> Angle {
    // `asin` via libm
    Angle::new::<radian>(libm::asin(2.0 * (q.w * q.y - q.z * q.x)))
}

#[inline]
pub fn roll_to_quat(angle: Angle) -> Quat {
    axis_angle_to_quat(Vec3::new(0.0, 1.0, 0.0), angle)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-12, "expected {a} ≈ {b}");
    }

    #[test]
    fn identity_is_unit() {
        approx_eq(Quat::IDENTITY.norm(), 1.0);
    }

    #[test]
    fn hamilton_identity_left() {
        let q = Quat::new(0.5, 0.5, 0.5, 0.5);
        let r = Quat::IDENTITY * q;
        approx_eq(r.w, q.w);
        approx_eq(r.x, q.x);
        approx_eq(r.y, q.y);
        approx_eq(r.z, q.z);
    }

    #[test]
    fn hamilton_identity_right() {
        let q = Quat::new(0.5, 0.5, 0.5, 0.5);
        let r = q * Quat::IDENTITY;
        approx_eq(r.w, q.w);
        approx_eq(r.x, q.x);
        approx_eq(r.y, q.y);
        approx_eq(r.z, q.z);
    }

    #[test]
    fn conjugate_inverts_unit() {
        let q = Quat::new(0.5, 0.5, 0.5, 0.5);
        let r = q * q.conjugate();
        approx_eq(r.w, 1.0);
        approx_eq(r.x, 0.0);
        approx_eq(r.y, 0.0);
        approx_eq(r.z, 0.0);
    }

    #[test]
    fn cross_orthogonal() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(a.cross(b), Vec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn axis_angle_90deg_z() {
        // 90° about +Z = [cos45, 0, 0, sin45]
        let q = axis_angle_to_quat(
            Vec3::new(0.0, 0.0, 1.0),
            Angle::new::<radian>(core::f64::consts::FRAC_PI_2),
        );
        approx_eq(q.w, libm::cos(core::f64::consts::FRAC_PI_4));
        approx_eq(q.z, libm::sin(core::f64::consts::FRAC_PI_4));
    }
}
