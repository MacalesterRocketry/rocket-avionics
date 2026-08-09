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

use embassy_time::{Duration, Instant};
use libm::{asin, atan2, cos, sin, sqrt};
use crate::config::{AHRS_ACC_BETA, AHRS_MAG_BETA, G};
use crate::math::{deg_to_rad, rad_to_deg, Deg, Grad4, Quat, Rad, Vec3};

/// Mutable AHRS runtime state. Lives behind an Embassy mutex; the sensor task
/// owns the write side and the control loop reads via getter functions.
#[derive(Debug, Clone, Copy)]
pub struct AhrsState {
    pub q: Quat,
    pub last_update: Instant,
    pub acceleration_earth: Vec3,
    pub velocity_earth: Vec3,
    pub position_earth: Vec3,
    pub angular_velocity_body: Vec3,
    in_flight: bool,
}

impl Default for AhrsState {
    fn default() -> Self {
        Self {
            q: Quat::IDENTITY,
            last_update: Instant::now(),
            acceleration_earth: Vec3::ZERO,
            velocity_earth: Vec3::ZERO,
            position_earth: Vec3::ZERO,
            angular_velocity_body: Vec3::ZERO,
            in_flight: false,
        }
    }
}

impl AhrsState {
    pub fn update(&mut self, gyro: Vec3, accel: Vec3, mag: Vec3, now: Instant) {
        let dt = now - self.last_update;
        self.last_update = now;

        // usually like .005s, so if it's far greater, skip so we don't get huge jumps in orientation from bad timing
        if dt <= Duration::from_millis(0) || dt > Duration::from_millis(100) {
            return; // Invalid time step, skip update
        }

        //  COMPLETE Q0 → Q4
        // Step 0: Initial quaternion (q0) - previous orientation
        let q0 = self.q;

        // Step 2: Gyro propagation (q1) - integrate angular rates
        let delta_q = delta_quat_from_gyro(gyro, dt);
        let q1 = q0 * delta_q;

        // Step 3: Normalize (q2) - maintain quaternion unit length
        let q2 = q1.normalized();

        // Step 4: Madgwick sensor fusion correction (q3) - fuse with accelerometer and magnetometer
        // In flight, we just follow the gyroscope. Gravity doesn't affect it, so we need to ignore the accelerometer, and the magnetometer is unreliable.
        let q3 = if self.in_flight { q2 } else { madgwick_correction_step(q2, accel, mag, dt) };

        // Step 5: Final normalization (q4) - ensure valid quaternion
        let q4 = q3.normalized();

        // Update state with final quaternion, final state
        self.q = q4;

        // EARTH FRAME CONVERSIONS
        // Convert body-frame measurements to earth frame for control systems
        let _earth_accel = rotate_body_to_earth(q4, accel);
        let _earth_gyro = rotate_body_to_earth(q4, gyro);
        let _earth_mag = rotate_body_to_earth(q4, mag);

        self.acceleration_earth = _earth_accel - Vec3 {x: 0.0, y: 0.0, z: G}; // Remove gravity from vertical acceleration when on the ground
        let dt_seconds = duration_to_seconds(dt);
        self.velocity_earth += self.acceleration_earth * dt_seconds;
        self.position_earth += self.velocity_earth * dt_seconds;

        self.angular_velocity_body = gyro; // still in body frame, but we can use it for control

        // // TODO: figure out debug stuff for Rust
        // // CONTINUOUS ORIENTATION MONITORING/Active Tracking
        // #if DEBUG and DEBUG_PRINT_ORIENTATION
        // static unsigned long lastPrint = 0;
        // if (now_micros - lastPrint > 100000) {
        //     // Convert quaternion to Euler angles
        //     const double roll = calculate_roll_deg(self.q);
        //     const double pitch = calculate_pitch_deg(self.q);
        //     const double yaw = calculate_yaw_deg(self.q);
        //
        //     // Display orientation and earth-frame data
        //     Serial.println("ROCKET ORIENTATION");
        //     Serial.printf("Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\r\n",
        //                   self.q.w, self.q.x, self.q.y, self.q.z);
        //     Serial.printf("Euler: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\r\n",
        //                   roll, pitch, yaw);
        //     Serial.printf("Earth Accel: X=%.2f, Y=%.2f, Z=%.2f m/s²\r\n",
        //                   self.acceleration_earth.x, self.acceleration_earth.y, self.acceleration_earth.z);
        //     Serial.printf("Rocket Accel: X=%.2f, y=%.2f, Z=%.2f m/s²\r\n",
        //                   accel.x, accel.y, accel.z);
        //     Serial.printf("Earth Velocity: X=%.2f, Y=%.2f, Z=%.2f m/s\r\n",
        //                   self.velocity_earth.x, self.velocity_earth.y, self.velocity_earth.z);
        //     Serial.printf("Earth Position: X=%.2f, Y=%.2f, Z=%.2f m\r\n",
        //                   self.position_earth.x, self.position_earth.y, self.position_earth.z);
        //     Serial.println("==========================");
        //
        //     lastPrint = now_micros;
        // }
        // #endif
    }

    pub fn launch(&mut self) {
        self.in_flight = true;
    }

    pub fn landing(&mut self) {
        self.in_flight = false;
    }

    pub fn zero_pos_vel(&mut self) {
        self.position_earth = Vec3::ZERO;
        self.velocity_earth = Vec3::ZERO;
    }

    // TODO: should this function exist? Should it just be new()?
    pub fn start(&mut self) {
        self.last_update = Instant::now();
    }

    pub fn get_orientation_earth(&self) -> Quat { self.q }
    pub fn get_acceleration_earth(&self) -> Vec3 { self.acceleration_earth }
    pub fn get_velocity_earth(&self) -> Vec3 { self.velocity_earth }
    pub fn get_position_earth(&self) -> Vec3 { self.position_earth }
    pub fn get_angular_velocity_body(&self) -> Vec3 { self.angular_velocity_body }
}

pub fn rotate_body_to_earth(q: Quat, v_b: Vec3) -> Vec3 {
    // p = q ⊗ [0,v_b] ⊗ q*
    let res = q * v_b.to_quat(0.0) * q.conjugate();
    Vec3{ x: res.x, y: res.y, z: res.z } // last 3 are vector part
}

pub fn rotate_earth_to_body(q: Quat, v_e: Vec3) -> Vec3 {
    // p = q* ⊗ [0,v_e] ⊗ q
    let res = q.conjugate() * v_e.to_quat(0.0) * q;
    Vec3{ x: res.x, y: res.y, z: res.z } // last 3 are vector part
}

// small helper: axis-angle -> quaternion exact
pub fn axis_angle_rad_to_quat(axis: Vec3, angle: f64) -> Quat {
    let half = angle * 0.5;
    let s = sin(half);
    Quat{ w: cos(half), x: axis.x * s, y: axis.y * s, z: axis.z * s }
}

// build delta quaternion(propagation) from angular rate omega (rad/s) and dt
pub fn delta_quat_from_gyro(omega: Vec3, dt: Duration) -> Quat {
    let dt_s = duration_to_seconds(dt);
    let wmag = omega.norm();
    if wmag < 1e-12 {
        // tiny rotation -> small-angle approx: q ≈ [1, 0.5*ω*dt]
        Quat { w: 1.0, x: 0.5 * omega.x * dt_s, y: 0.5 * omega.y * dt_s, z: 0.5 * omega.z * dt_s }
    } else {
        let axis = omega / wmag;
        let theta = wmag * dt_s;
        axis_angle_rad_to_quat(axis, theta)
    }
}

// Madgwick correction step, I dont know how to get the library from adafruit so I just did it manually/mathmatically!

pub fn compute_magnetometer_gradient(m_n: Vec3, q: Quat) -> Grad4 {
    // For magnetometer part we should compute reference direction and its gradient.
    // compute Earth's magnetic field in body frame and gradient.
    // Compute h = q ⊗ m_n ⊗ q*  (magnetic field in earth frame)
    let h = rotate_body_to_earth(q, m_n);

    // Projection of h onto x-y plane of Earth (reference)
    // b = [0, bx, 0, bz] as in Madgwick
    let b = Vec3 {
        x: sqrt(h.x * h.x + h.y * h.y),
        y: 0.0,
        z: h.z
    };

    // compute the gradient of magnetometer using a simplified combined gradient:
    // Compute an approximate mag error vector between predicted and measured (in body frame!)
    let m_pred = rotate_earth_to_body(q, b); // predicted magnetic field in body frame (reference)
    let mag_err = Vec3 {
        x: m_pred.x / (m_pred.norm() + 1e-12) - m_n.x,
        y: m_pred.y / (m_pred.norm() + 1e-12) - m_n.y,
        z: m_pred.z / (m_pred.norm() + 1e-12) - m_n.z
    };

    // Build a simple magnetometer gradient approximation using cross products of predicted vs measured
    // This is less exact than full Madgwick derivation but should work well in practice as a correction term, simpler for us
    let g_mag_vec = Vec3 {
        x: 2.0 * (q.y * mag_err.z - q.z * mag_err.y + q.w * mag_err.x - q.x * mag_err.z),
        y: 2.0 * (q.z * mag_err.x - q.w * mag_err.z + q.x * mag_err.y - q.y * mag_err.x),
        z: 2.0 * (q.w * mag_err.y - q.x * mag_err.x + q.y * mag_err.z - q.z * mag_err.y)
    };
    // Convert this vector into 4-component approximate gradient (spread across all axes)
    Grad4 {
        w: 0.0,
        x: g_mag_vec.x,
        y: g_mag_vec.y,
        z: g_mag_vec.z
    }
}

pub fn compute_accelerometer_gradient(a_n: Vec3, q: Quat) -> Grad4 {
    // Compute objective function gradient (following Madgwick 2010 equations, check paper).
    // For full derivation see Madgwick's report; here we implement the standard gradient step.
    // variables
    let _2q1 = 2.0 * q.w; let _2q2 = 2.0 * q.x; let _2q3 = 2.0 * q.y; let _2q4 = 2.0 * q.z;
    let _4q2 = 4.0 * q.x; let _4q3 = 4.0 * q.y;

    // Gradient of f (accel) part (from Madgwick): need to use specific q# combinations
    // math behind the values= f = predicted_gravity - measured_gravity; predicted_gravity = [2(q2 q4 - q1 q3), 2(q1 q2 + q3 q4), q1^2 - q2^2 - q3^2 + q4^2]
    let f = Vec3 {
        x: 2.0 * (q.x * q.z - q.w * q.y) - a_n.x,
        y: 2.0 * (q.w * q.x + q.y * q.z) - a_n.y,
        z: q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z - a_n.z
    };

    // Jacobian J (3x4) lines combined into gradient g = J^T * f  (this expands to 4 components, as we need it to build a quaternion)
    // Madgwick 2010 paper's compact expression:
    Grad4 {
        w: 0.0,
        x: -_2q3 * f.x + _2q2 * f.y,
        y:  _2q4 * f.x + _2q1 * f.y - _4q2 * f.z,
        z: -_2q1 * f.x + _2q4 * f.y - _4q3 * f.z,
    }
}

// returns corrected sensor fused quaternion as quaternion q3 (unnormalized)
fn madgwick_correction_step(q_pred: Quat, // predicted quaternion (gyro-propagated & normalized) -> q2
                            acc: Vec3, // accelerometer (bias-corrected) in body frame (no normalizing required; we can normalize inside)
                            mag: Vec3, // magnetometer (bias-corrected & soft-iron corrected) in body frame
                            dt: Duration) -> Quat { // time step (s)
    let q = q_pred;

    let nm = mag.norm();
    let mut mag_grad = Grad4 {w: 0.0, x: 0.0, y: 0.0, z: 0.0};
    // Skip magnetometer correction if the measurement is too small
    if nm >= 1e-12 {
        let m_n = mag / nm; // normalized magnetometer measurement
        mag_grad = compute_magnetometer_gradient(m_n, q);
    }

    let na = acc.norm();
    let mut accel_grad = Grad4 {w: 0.0, x: 0.0, y: 0.0, z: 0.0};
    // Skip gravity-based accelerometer correction if the measurement doesn't seem like gravity (like during burn or coast)
    if na >= 0.5 * G && na <= 2.0 * G {
        let a_n = acc / na; // normalized accelerometer measurement
        accel_grad = compute_accelerometer_gradient(a_n, q);
    }

    let mut g_combined = accel_grad * AHRS_ACC_BETA + mag_grad * AHRS_MAG_BETA;

    // Normalize gradient
    let gn = g_combined.norm();
    if gn > 0.0 {
        g_combined /= gn;
    }

    // Integrate to get corrected quaternion (Simple Euler integration)
    q_pred - (g_combined * duration_to_seconds(dt)).into()
}

// Bias computation (mean across samples)
pub fn compute_bias_mean(samples: &[Vec3]) -> Vec3 {
    let mut s = Vec3 { x: 0.0, y: 0.0, z: 0.0 };
    if samples.is_empty() {
         return s;
    }
    for v in samples.iter() {
        s += *v;
    }
    s / samples.len() as f64
}

// Main Loop AHRS function Using the above utilities

pub fn calculate_pitch_rad(q: Quat) -> Rad {
    atan2(2.0 * (q.w * q.x + q.y * q.z),
          1.0 - 2.0 * (q.x * q.x + q.y * q.y))
}

pub fn calculate_yaw_rad(q: Quat) -> Rad {
    asin(2.0 * (q.w * q.y - q.z * q.x))
}

pub fn calculate_roll_rad(q: Quat) -> Rad {
    -atan2(2.0 * (q.w * q.z + q.x * q.y),
           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
}

pub fn calculate_roll_deg(q: Quat) -> Deg {
    rad_to_deg(calculate_roll_rad(q))
}

pub fn calculate_pitch_deg(q: Quat) -> Deg {
    rad_to_deg(calculate_pitch_rad(q))
}

pub fn calculate_yaw_deg(q: Quat) -> Deg {
    rad_to_deg(calculate_yaw_rad(q))
}

pub fn yaw_rad_to_quat(roll: Rad) -> Quat {
    axis_angle_rad_to_quat(Vec3 {x: 1.0, y: 0.0, z: 0.0}, roll)
}

pub fn pitch_rad_to_quat(yaw: Rad) -> Quat {
    axis_angle_rad_to_quat(Vec3 {x: 0.0, y: 0.0, z: 1.0}, yaw)
}

pub fn roll_rad_to_quat(pitch: Rad) -> Quat {
    axis_angle_rad_to_quat(Vec3 {x: 0.0, y: 1.0, z: 0.0}, pitch)
}

pub fn yaw_deg_to_quat(yaw: Deg) -> Quat {
    yaw_rad_to_quat(deg_to_rad(yaw))
}

pub fn pitch_deg_to_quat(pitch: Deg) -> Quat {
    pitch_rad_to_quat(deg_to_rad(pitch))
}

pub fn roll_deg_to_quat(roll: Deg) -> Quat {
    roll_rad_to_quat(deg_to_rad(roll))
}

fn duration_to_seconds(dt: Duration) -> f64 {
    dt.as_nanos() as f64 * 1e-9 // convert to seconds
}
