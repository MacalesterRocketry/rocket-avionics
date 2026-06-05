//! Madgwick-style attitude/heading reference system, ported from `ahrs.cpp`.
//!
//! This module is split into a **pure math layer** (gradient computation, quat
//! propagation) that's host-testable, and a **runtime state struct** that the
//! sensor task reads from / writes to via an Embassy mutex. Keeping the math
//! separate from the I/O lets us regression-test against existing logXX.bin
//! files before flying.
//!
//! Status: SKELETON. Function signatures match the C++ API so the rest of the
//! firmware can be ported against this surface, but bodies are stubs.

#![allow(dead_code, unused_variables)]

use uom::si::angle::radian;
use uom::si::angular_velocity::degree_per_second;
use uom::si::f64::{Angle, AngularVelocity, Time};
use crate::config::G;
use crate::math::{AngularVelocityVector, Grad4, Quat, Vec3};

/// Body→earth quaternion rotation: `p = q ⊗ [0, v_b] ⊗ q*`
pub fn rotate_body_to_earth(q: Quat, v_b: Vec3) -> Vec3 {
    let r = q * v_b.to_quat(0.0) * q.conjugate();
    Vec3::new(r.x, r.y, r.z)
}

/// Earth→body: `p = q* ⊗ [0, v_e] ⊗ q`
pub fn rotate_earth_to_body(q: Quat, v_e: Vec3) -> Vec3 {
    let r = q.conjugate() * v_e.to_quat(0.0) * q;
    Vec3::new(r.x, r.y, r.z)
}

/// Δq from gyro rate `ω` over `dt`. Small-angle branch matches C++ for stability
/// near zero rotation rate (avoids divide-by-zero in axis-angle form).
pub fn delta_quat_from_gyro(omega: AngularVelocityVector, dt: Time) -> Quat {
    let wmag = omega.norm();
    let zero_threshold = AngularVelocity::new::<degree_per_second>(1e-12);
    if wmag < zero_threshold {
        let x = (0.5 * omega.x * dt).value;
        let y = (0.5 * omega.y * dt).value;
        let z = (0.5 * omega.z * dt).value;
        Quat::new(1.0, x, y, z)
    } else {
        let axis = omega / wmag;
        let theta = Angle::new::<radian>((wmag * dt).value);
        crate::math::axis_angle_to_quat(axis, theta)
    }
}

/// Madgwick magnetometer gradient term. TODO: port from C++ once accelerometer
/// gradient is validated against bench data — magnetometer β is currently 0.0
/// in the flight config, so this is a no-op on the hot path.
pub fn compute_magnetometer_gradient(_m_n: Vec3, _q: Quat) -> Grad4 {
    Grad4::default()
}

/// Madgwick accelerometer gradient term, equation (25)–(28) in the 2010 paper.
/// TODO: port the explicit C++ expressions and add a host-side test that
/// reproduces a known correction step from a logged sample.
pub fn compute_accelerometer_gradient(_a_n: Vec3, _q: Quat) -> Grad4 {
    Grad4::default()
}

/// Mutable AHRS runtime state. Lives behind an Embassy mutex; the sensor task
/// owns the write side and the control loop reads via getter functions.
#[derive(Debug, Clone, Copy)]
pub struct AhrsState {
    pub q: Quat,
    pub last_update_us: u64,
    pub acceleration_earth: Vec3,
    pub velocity_earth: Vec3,
    pub position_earth: Vec3,
    pub angular_velocity_body: Vec3,
}

impl Default for AhrsState {
    fn default() -> Self {
        Self {
            q: Quat::IDENTITY,
            last_update_us: 0,
            acceleration_earth: Vec3::ZERO,
            velocity_earth: Vec3::ZERO,
            position_earth: Vec3::ZERO,
            angular_velocity_body: Vec3::ZERO,
        }
    }
}

impl AhrsState {
    /// One AHRS step. `in_flight=true` disables gravity-based accel correction
    /// (in C++ this skips the Madgwick step entirely during burn/coast).
    /// TODO: port full update logic — see `ahrs.cpp::update_ahrs`.
    pub fn update(&mut self, _gyro: Vec3, _accel: Vec3, _mag: Vec3, _now_us: u64, _in_flight: bool) {
        // Stub. Will integrate:
        //   1. dt = (now - last) / 1e6, skip if dt ∉ (0, 0.1)
        //   2. q1 = q0 ⊗ Δq_gyro
        //   3. q2 = normalize(q1)
        //   4. q3 = madgwick correction if !in_flight
        //   5. q4 = normalize(q3)
        //   6. earth-frame accel = R(q4)·accel − [0,0,G]
        //   7. integrate velocity, position
    }

    pub fn zero_pos_vel(&mut self) {
        self.position_earth = Vec3::ZERO;
        self.velocity_earth = Vec3::ZERO;
    }
}
