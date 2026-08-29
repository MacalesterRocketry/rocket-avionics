//! Roll PID controller
//!
//! Ported from `roll-controller.cpp`. Takes an absolute target attitude plus the
//! current AHRS solution and returns a desired angular acceleration. It knows
//! nothing about fins, servos, or flight state: turning angular acceleration
//! into actuator commands is `fins.rs`'s job, and deciding what attitude to
//! target is the state machine's. That keeps this a pure control law, testable
//! on host against the Python simulation that produced the gains in `config.rs`.

use crate::config::{ROLL_PID_KD, ROLL_PID_KI, ROLL_PID_KP};
use crate::navigation::ahrs::AhrsState;
use crate::utils::duration_to_seconds;
use crate::utils::math::{AngularVec3, Quat};
use embassy_time::{Duration, Instant};

/// Sanity bounds on the timestep, guarding the integral against a garbage `dt`
/// — same reasoning as the guard in `AhrsState::update`. Covers both the first
/// tick after arming and a loop that stalled long enough to make the integral
/// jump.
const DT_MIN: Duration = Duration::from_micros(1);
const DT_MAX: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Copy)]
pub struct RollPid {
    integral: f64,
    last_time: Instant,
}

impl RollPid {
    pub fn new() -> Self {
        Self {
            integral: 0.0,
            last_time: Instant::now(),
        }
    }

    /// Clear the integral and restart the timebase. Call on arming, so a
    /// disarmed stretch can't dump a stale integral into the fins on the first
    /// armed tick.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_time = Instant::now();
    }

    /// One control step. `qtarget` is the absolute attitude to hold. The
    /// timestep is measured internally, so the caller's tick rate is the only
    /// thing that sets it.
    pub fn step(&mut self, qtarget: Quat, ahrs: &AhrsState) -> AngularVec3 {
        let now = Instant::now();
        let dt = now - self.last_time;
        self.last_time = now;

        let qcurrent: Quat = ahrs.get_orientation_earth();
        // Compute quaternion error
        let qerror: Quat = (qcurrent.conjugate() * qtarget).normalized();
        // TODO: Also, is this in earth-frame or body frame? If we were also controlling pitch and yaw, that wouldn't matter, but we need to be sure the axis is right.
        //  This is definitely earth frame, since it's based on sources that also control pitch and yaw. We need to transform it to body.

        AngularVec3 {
            pitch: 0.0,
            yaw: 0.0,
            // TODO: physically this acts about the y axis, per the `.y` reads
            //  in roll_pid, which disagrees with `AngularVec3`'s roll-is-z mapping.
            //  Both are pre-axis-cleanup conventions; not reconciling here.
            roll: self.roll_pid(qerror, ahrs, dt),
        }
    }

    fn roll_pid(&mut self, qerror: Quat, ahrs: &AhrsState, dt: Duration) -> f64 {
        // Convert to angular error
        let roll_error: f64 = -2.0 * qerror.z; // Z-component = roll error
        // TODO: I've found two possible errors from the simulation. First, we need to confirm that y is the right component; if it's not, we'll have problems like what we got.
        //  Second, qtarget may have the wrong sign of target going in. Not a big deal, and it might not actually be true outside of sim, but it's something to consider.

        // PID terms
        let ang_accel_p: f64 = ROLL_PID_KP * roll_error;
        if dt >= DT_MIN && dt <= DT_MAX {
            self.integral += roll_error * duration_to_seconds(dt);
        }
        let ang_accel_i: f64 = ROLL_PID_KI * self.integral;
        // Filtered rather than raw: at the 50 Hz actuation rate the raw gyro
        // would alias high-frequency noise into the control band, and no
        // downstream filter could undo it. See GYRO_LPF_HZ.
        let ang_accel_d: f64 = -ROLL_PID_KD * ahrs.get_angular_velocity_filtered().roll;
        // TODO: The integral is unbounded, so sustained fin saturation winds it
        //  up and leaves the fins pegged after the error reverses. It wants a
        //  clamp, but the bound is a tuning decision — pick it against sim or
        //  flight data rather than guessing here.

        let ang_accel_desired: f64 = ang_accel_p + ang_accel_i + ang_accel_d;

        // TODO: figure out logging to SD and defmt. This fires every control
        //  tick, so it wants to be off by default in flight.
        defmt::info!("roll pid: err={}, accel={}", roll_error, ang_accel_desired);
        ang_accel_desired
    }
}