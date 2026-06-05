//! Roll PID controller. Skeleton.
//!
//! Ported from `roll-controller.cpp`. The pure-math pieces (effectiveness
//! curve, PID step from quat error to fin deflection) live here as plain
//! functions so they can be unit-tested on host against the existing
//! Python simulation that produced the Kp/Ki/Kd values in `config.rs`.

#![allow(dead_code, unused_variables)]

use crate::config::TORQUE_PER_DEG_50MS;
use crate::math::{Quat, Vec3};
use uom::si::angle::degree;
use uom::si::f64::*;

/// Fin effectiveness (N·m per degree) at a given airspeed. Linear-in-deflection
/// approximation, quadratic-in-velocity. Matches `calculate_effectiveness`.
pub fn effectiveness(velocity_earth: Vec3) -> f64 {
    let v = velocity_earth.mag();
    TORQUE_PER_DEG_50MS * (v * v) / (50.0 * 50.0)
}

/// Stateful PID step. Caller must persist `integral` across calls (or pass an
/// owned `RollPid` struct — preferred over the C++ `static double integral`
/// trick, which would not be thread-safe across cores).
#[derive(Default, Debug, Clone, Copy)]
pub struct RollPid {
    pub integral: f64,
}

impl RollPid {
    /// Returns fin deflection (deg). Returns 0 when effectiveness is too low
    /// to authoritatively command — same guard as the C++ version.
    /// TODO: port full body once `crate::ahrs::AhrsState` is wired in.
    pub fn step(
        &mut self,
        _q_current: Quat,
        _q_target: Quat,
        _angular_velocity_body: Vec3,
        _velocity_earth: Vec3,
        _dt: f64,
    ) -> Angle {
        Angle::new::<degree>(0.0)
    }
}
