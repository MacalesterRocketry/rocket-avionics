use crate::config::{ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD, MOMENT_OF_INERTIA, TORQUE_PER_DEG_50MS};
use crate::orientation::ahrs::Ahrs;

pub struct RollController {
    integral: f64,
}

impl RollController {
    pub fn new() -> Self {
        Self { integral: 0.0 }
    }

    pub fn update(&mut self, ahrs: &Ahrs, target_roll_deg: f64, dt: f64) -> f64 {
        let current_roll_deg = ahrs.get_roll_deg();
        let error = target_roll_deg - current_roll_deg;

        let p = ROLL_PID_KP * error;
        self.integral += error * dt;
        let i = ROLL_PID_KI * self.integral;
        let d = -ROLL_PID_KD * ahrs.angular_velocity.y; // Using Y as roll axis like C++

        let ang_accel_desired = p + i + d;
        let torque_desired = ang_accel_desired * MOMENT_OF_INERTIA;

        let v = ahrs.velocity.norm();
        let effectiveness = TORQUE_PER_DEG_50MS * (v * v) / (50.0 * 50.0);

        if effectiveness.abs() < 1e-6 {
            0.0
        } else {
            torque_desired / effectiveness
        }
    }
}
