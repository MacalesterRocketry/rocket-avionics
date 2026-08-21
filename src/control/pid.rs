//! Roll PID controller
//!
//! Ported from `roll-controller.cpp`. The pure-math pieces (effectiveness
//! curve, PID step from quat error to fin deflection) live here as plain
//! functions so they can be unit-tested on host against the existing
//! Python simulation that produced the Kp/Ki/Kd values in `config.rs`.

use crate::config::{ROLL_PID_KD, ROLL_PID_KI, ROLL_PID_KP};
use crate::navigation::ahrs::AhrsState;
use crate::utils::math::{Deg, Quat, calculate_roll_deg, duration_to_seconds, roll_deg_to_quat};
use embassy_time::{Duration, Instant};
use num_traits::abs;

/// Stateful PID step. Caller must persist `integral` across calls (or pass an
/// owned `RollPid` struct — preferred over the C++ `static double integral`
/// trick, which would not be thread-safe across cores).
#[derive(Debug, Clone, Copy)]
pub struct RollPid {
    pub integral: f64,
    initial_orientation: Quat,
    last_time: Instant,
}

impl RollPid {
    pub fn new() -> Self {
        Self {
            integral: 0.0,
            initial_orientation: Quat::IDENTITY,
            last_time: Instant::now(),
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
        target_angle: Deg,
        ahrs: AhrsState,
        dt: Duration, // TODO: Do I want to do it this way or calculate it?
    ) -> Deg {
        // Start
        let qcurrent: Quat = ahrs.get_orientation_earth(); // Current orientation (Q4)
        let current_angle: Deg = calculate_roll_deg(qcurrent); // Current roll angle in degrees (from Q4)
        // TODO: I'm pretty sure this current angle is just wrong. It should be in body frame, not earth frame.
        //  Edit: Looking at the data compared to the integrated gyro data, it's very close throughout most of the flight but differs greatly when the rocket tilts at apogee. As a result, I think it is earth-frame.
        //  But it's not used for anything, so it may not matter.

        // If it is angled, we need to adjust that tilt before storing the orientation)
        // TODO: Figure out what this comment means

        // BUILD ROLL QUATERNION
        // const Quat qroll = Quat{cos(roll_angle_rad/2), sin(roll_angle_rad/2), 0, 0}; // Roll by 90°
        // TODO: The old was x: sin(angle/2), but the new one has x: sin(angle). Should the Vec3 passed into axisAngleToQuat be {0.5, 0, 0}? Check this all with Tala.
        let qroll: Quat = roll_deg_to_quat(-target_angle); // Negative for body frame
        let qtarget: Quat = (self.initial_orientation * qroll).normalized(); // Target orientation; TODO: This probably has different pitch and yaw than what we actually want. Replace them with the current ones from qcurrent? Or just ignore them since we're only controlling roll?

        let mut chosen_deflection = 0.0;

        // CONTROL LOOP (Until target reached)
        let now = Instant::now();
        if abs(target_angle - current_angle) > 1.0 {
            let dt = now - self.last_time;
            let fin_deflection_angle = self.calculate_desired_angular_acceleration(&qtarget, &ahrs, dt);
            // TODO: figure out logging to SD and defmt
            // logRollControl(target_angle, current_angle, fin_deflection_angle);
            // #if DEBUG and DEBUG_PRINT_ROLL_CONTROL
            // Serial.print("Target Roll: ");
            // Serial.print(target_angle);
            // Serial.print("°, Current Roll: ");
            // Serial.print(current_angle);
            // Serial.print("°, Fin Deflection: ");
            // Serial.print(fin_deflection_angle);
            // Serial.println("°");
            // #endif

            // TODO: Command servos (or just set the angle somewhere and have the servos read it)
            // for (const ServoID servo : servos) {
            //     set_servo_angle(servo, fin_deflection_angle);
            // }
            chosen_deflection = fin_deflection_angle;
        }
        self.last_time = now;
        chosen_deflection
    }

    fn calculate_desired_angular_acceleration(&mut self, qtarget: &Quat, ahrs: &AhrsState, dt: Duration) -> Deg {
        let qcurrent = ahrs.get_orientation_earth();

        // ============================================
        // ERROR CALCULATION
        // ============================================

        // Compute quaternion error
        let qrollerror: Quat = (qcurrent.conjugate() * *qtarget).normalized();
        //const Quat qrollerror = (qtarget.conjugate() * qcurrent).normalized();
        // TODO: This is the geodesic path, not the total path, so it can get confused since moving away from the target might actually move it closer. Fix?
        // TODO: Also, is this in earth-frame or body frame? If we were also controlling pitch and yaw, that wouldn't matter, but we need to be sure the axis is right.
        //  This is definitely earth frame, since it's based on sources that also control pitch and yaw. We need to transform it to body.

        // Convert to angular error
        let eroll_y: f64 = -2.0 * qrollerror.y; // Y-component = roll error
        // double eroll_y_test = calculate_roll_deg(qrollerror);
        // TODO: I've found two possible errors from the simulation. First, we need to confirm that y is the right component; if it's not, we'll have problems like what we got.
        //  Second, qtarget may have the wrong sign of target going in. Not a big deal, and it might not actually be true outside of sim, but it's something to consider.

        // TORQUE PID CALCULATION

        // PID terms
        let ang_accel_p: f64 = ROLL_PID_KP * eroll_y;
        self.integral += eroll_y * duration_to_seconds(dt);
        let ang_accel_i: f64 = ROLL_PID_KI * self.integral;
        let ang_accel_d: f64 = -ROLL_PID_KD * ahrs.get_angular_velocity_body().y;

        // Total desired angular acceleration
        let ang_accel_desired: f64 = ang_accel_p + ang_accel_i + ang_accel_d;
        ang_accel_desired
    }
}
