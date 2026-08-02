//! Roll PID controller. Skeleton.
//!
//! Ported from `roll-controller.cpp`. The pure-math pieces (effectiveness
//! curve, PID step from quat error to fin deflection) live here as plain
//! functions so they can be unit-tested on host against the existing
//! Python simulation that produced the Kp/Ki/Kd values in `config.rs`.

#![allow(dead_code, unused_variables)]

use embassy_time::Duration;
use crate::config::{MOMENT_OF_INERTIA, ROLL_PID_KD, ROLL_PID_KI, ROLL_PID_KP, TORQUE_PER_DEG_50MS};
use crate::math::{Deg, Quat, Vec3};
use crate::orientation::ahrs::AhrsState;

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
    initial_orientation: Quat,
}

impl RollPid {
    pub fn default() -> Self {
        Self {
            integral: 0.0,
            initial_orientation: Quat::IDENTITY,
        }
    }
    
    pub fn launch(&mut self, ahrs: AhrsState) {
        self.initial_orientation = ahrs.q;
    }
    
    /// Returns fin deflection (deg). Returns 0 when effectiveness is too low
    /// to authoritatively command — same guard as the C++ version.
    /// TODO: port full body once `crate::ahrs::AhrsState` is wired in.
    pub fn step(
        &mut self,
        desired_angle: Deg,
        ahrs: AhrsState,
        dt: Duration,
    ) -> Deg {
        0.0
    }
}
