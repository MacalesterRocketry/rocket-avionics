use embassy_rp::pwm::{Config, Pwm};
use crate::config::{SERVO_MICROS_MIN, SERVO_MICROS_MAX, SERVO_NEUTRAL_ANGLE, SERVO_DEGREE_RANGE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE};
use crate::utils::clamp;

pub struct Servo<'d> {
    pwm: Pwm<'d>,
}

impl<'d> Servo<'d> {
    pub fn new(mut pwm: Pwm<'d>) -> Self {
        let mut config = Config::default();
        config.divider = 64.into();
        config.top = 39062; 
        pwm.set_config(&config);
        Self { pwm }
    }

    pub fn set_angle(&mut self, angle_degrees_from_neutral: f64) {
        let offset_angle = -angle_degrees_from_neutral + SERVO_NEUTRAL_ANGLE + (SERVO_DEGREE_RANGE / 2.0);
        let clamped_angle = clamp(offset_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        let progress = clamped_angle / SERVO_DEGREE_RANGE; // 0.0 to 1.0
        
        let micros = SERVO_MICROS_MIN as f64 + (SERVO_MICROS_MAX - SERVO_MICROS_MIN) as f64 * progress;
        
        let duty = (micros / 20000.0 * 39062.0) as u16;
        
        let mut config = Config::default();
        config.divider = 64.into();
        config.top = 39062;
        config.compare_a = duty;
        self.pwm.set_config(&config);
    }
}
