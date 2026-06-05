//! Servo control. Skeleton.
//!
//! The C++ code uses Arduino's `Servo` library, which on the RP2040 maps each
//! servo to a PWM slice. RP2350 has the same PWM IP block (8 slices × 2
//! channels), and embassy-rp exposes them via `embassy_rp::pwm::Pwm`.
//!
//! Plan: one `Pwm` instance per servo pin, configured for 50 Hz period
//! (20 ms), with the duty cycle mapped from `SERVO_MICROS_MIN..=SERVO_MICROS_MAX`
//! to PWM ticks. `set_servo_angle()` is a synchronous function that just
//! writes the new compare value — no `.await` needed.

#![allow(dead_code, unused_variables)]

use uom::si::angle::degree;
use uom::si::f64::Angle;
use crate::math::clamp;
use crate::config::{
    SERVO_DEGREE_RANGE, SERVO_MICROS_MAX, SERVO_MICROS_MIN, SERVO_NEUTRAL_ANGLE,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServoId { XPlus, XMinus, YPlus, YMinus }

pub const ALL_SERVOS: [ServoId; 4] =
    [ServoId::XPlus, ServoId::XMinus, ServoId::YPlus, ServoId::YMinus];

/// Map a 0.0..=1.0 progress to a pulse width in microseconds.
#[inline]
fn calc_servo_micros(progress: f64) -> u32 {
    SERVO_MICROS_MIN + ((SERVO_MICROS_MAX - SERVO_MICROS_MIN) as f64 * progress) as u32
}

/// Convert an angle (deg, signed, around the fin-neutral point) into a pulse
/// width in microseconds. Pure math — no I/O — so it's host-testable.
pub fn angle_to_micros(angle_from_neutral: Angle) -> u32 {
    let neutral = Angle::new::<degree>(SERVO_NEUTRAL_ANGLE);
    let range = Angle::new::<degree>(SERVO_DEGREE_RANGE);
    let offset: Angle = -angle_from_neutral + neutral + (range / 2.0);
    let clamped = clamp(offset, neutral, range + neutral);
    let progress = clamped.get::<degree>() / SERVO_DEGREE_RANGE;
    calc_servo_micros(progress)
}

/// Set a servo to a deflection angle. TODO: write to the corresponding PWM.
pub fn set_servo_angle(_servo: ServoId, _angle_from_neutral: Angle) {
    // TODO: look up PWM channel from ServoId and call `pwm.set_duty_cycle()`.
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn neutral_maps_to_midpoint() {
        // At angle = SERVO_NEUTRAL_ANGLE the offset = degree_range/2, progress = 0.5,
        // pulse = midpoint of [SERVO_MICROS_MIN, SERVO_MICROS_MAX].
        let us = angle_to_micros(Angle::new::<degree>(SERVO_NEUTRAL_ANGLE));
        let mid = (SERVO_MICROS_MIN + SERVO_MICROS_MAX) / 2;
        assert!((us as i64 - mid as i64).abs() <= 1);
    }
}
