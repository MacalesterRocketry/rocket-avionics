//! Servo control.
//!
//! Each RP2350 PWM slice drives two channels, so the four fin servos occupy
//! two slices. Which channel a servo lands on is fixed by its GPIO number, and
//! that mapping is declared once in `config::board`. By the time a `Servos`
//! reaches this module the slices are already split into independent [`Fin`]s,
//! so nothing here knows or cares about slices or A/B channels — a fin is a
//! named field, and writing one never disturbs its slice-mate.
//!
//! Each [`Fin`] owns its trim and applies it on every write, so there is no
//! path that commands a servo without its calibration.
//!
//! Setting an angle is a plain register write, so none of this is `async`.

use defmt::{error, info, Debug2Format};
use embassy_rp::pwm;
use embassy_rp::pwm::PwmOutput;
use embassy_time::{Duration, Ticker};
use embedded_hal::pwm::SetDutyCycle;

use crate::config::board::{ServoConfig, Servos, SYS_CLK_HZ};
use crate::config::{
    SERVO_DEGREE_RANGE, SERVO_MAX_ANGLE, SERVO_MICROS_MAX, SERVO_MICROS_MIN, SERVO_MIN_ANGLE,
    SERVO_TRIM,
};
use crate::{mark_init_failed, Subsystem, mark_init_complete};
use crate::math::{clamp, Deg};

/// Standard hobby-servo frame rate.
const SERVO_PWM_HZ: u32 = 50;
/// One PWM frame in microseconds — the denominator every pulse width is
/// expressed against.
const SERVO_PERIOD_MICROS: u16 = (1_000_000 / SERVO_PWM_HZ) as u16;

/// Slice clock divider. Picked together with [`PWM_TOP`] so that a frame is an
/// exact whole number of counts; the asserts below enforce that.
const PWM_DIV_INT: u8 = 50;
/// Counter top. The slice counts `0..=PWM_TOP`, so one frame is `PWM_TOP + 1`
/// counts of the divided clock.
///
/// At 150 MHz the divider yields a 3 MHz counter clock, and 3 MHz / 50 Hz =
/// 60_000 counts per frame — so the top is 59_999, and the frame is exactly
/// 20 ms with no rounding error.
const PWM_TOP: u16 = (SYS_CLK_HZ / PWM_DIV_INT as u32 / SERVO_PWM_HZ - 1) as u16;

// A frame must be a whole number of counts, or every pulse width is skewed by
// the rounding error. That count must also fit the 16-bit counter.
const _: () = assert!(
    SYS_CLK_HZ % (PWM_DIV_INT as u32 * SERVO_PWM_HZ) == 0,
    "servo PWM frame is not a whole number of counts; adjust PWM_DIV_INT"
);
const _: () = assert!(
    SYS_CLK_HZ / PWM_DIV_INT as u32 / SERVO_PWM_HZ <= u16::MAX as u32 + 1,
    "servo PWM counter top overflows u16; raise PWM_DIV_INT"
);

/// Pulse width per degree of fin deflection: 10 µs/deg over a 100° sweep.
const MICROS_PER_DEG: f64 = (SERVO_MICROS_MAX - SERVO_MICROS_MIN) as f64 / SERVO_DEGREE_RANGE;
/// Pulse width at zero deflection and zero trim.
const MICROS_CENTER: f64 = (SERVO_MICROS_MIN + SERVO_MICROS_MAX) as f64 / 2.0;

/// Convert a deflection angle (deg, signed, 0 = in line with the fin) plus that
/// fin's `trim` into a pulse width in microseconds.
///
/// Limited both to the fin's mechanical limits (the bounds for the servo's travel)
/// and the servo's electronic endpoints, so trim removes a bit of range from one end.
pub const fn angle_to_micros(angle_deg_from_neutral: Deg, trim: Deg) -> u16 {
    let commanded = clamp(angle_deg_from_neutral, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    let micros = MICROS_CENTER - (commanded - trim) * MICROS_PER_DEG;
    clamp(micros, SERVO_MICROS_MIN as f64, SERVO_MICROS_MAX as f64) as u16
}

/// PWM slice configuration shared by every servo.
fn servo_pwm_config() -> pwm::Config {
    let mut config = pwm::Config::default();
    config.top = PWM_TOP;
    config.divider = PWM_DIV_INT.into();
    config
}

/// One fin's servo, carrying the trim that calibrates it.
///
/// The `PwmOutput` is private on purpose: every write goes through
/// [`set_angle`](Fin::set_angle), so there is no way to command a fin and
/// forget its trim.
pub struct Fin {
    output: PwmOutput<'static>,
    trim: Deg,
}

impl Fin {
    /// Built by the generated `ServoConfig::into_servos`. Not useful elsewhere:
    /// a `PwmOutput` can only be obtained by claiming the servo pins, which
    /// happens exactly once.
    pub const fn new(output: PwmOutput<'static>, trim: Deg) -> Self {
        Self { output, trim }
    }

    /// This fin's mechanical zero offset in degrees
    pub const fn trim(&self) -> Deg {
        self.trim
    }

    /// Deflect this fin. 0° is in line with the fin; positive and negative
    /// angles rotate it opposite ways. Trim is always applied.
    pub fn set_angle(&mut self, angle_deg_from_neutral: Deg) -> Result<(), pwm::PwmError> {
        self.output.set_duty_cycle_fraction(
            angle_to_micros(angle_deg_from_neutral, self.trim),
            SERVO_PERIOD_MICROS,
        )
    }
}

/// Claim the servo pins and bring every fin to neutral.
pub fn init_servos(servo_config: ServoConfig) -> Result<Servos, pwm::PwmError> {
    let mut servos = servo_config.into_servos(&servo_pwm_config(), SERVO_TRIM);
    for fin in servos.iter_mut() {
        fin.set_angle(0.0)?;
    }
    Ok(servos)
}

pub async fn servos_loop(servo_config: ServoConfig) {
    info!("initializing servos");
    let servos = init_servos(servo_config);
    if servos.is_err() {
        error!("Error initializing servos: {:?}", Debug2Format(&servos.err().unwrap()));
        mark_init_failed(Subsystem::SERVOS);
    } else {
        mark_init_complete(Subsystem::SERVOS);
    }
    info!("servos initialized");

    let mut ticker = Ticker::every(Duration::from_hz(20));
    loop {
        ticker.next().await;
        // TODO: Figure out what tick rate I actually want and how to communicate it. Maybe just have an atomic or Watch for the servo angles?
        //  Or maybe move the final angular acceleration -> servo angle calculation over to this file and have a Watch for desired pitch, roll, yaw accel?
        //  Or move PID here?
        //  This definitely doesn't need to be faster than 50Hz, since that's the PWM speed.
    }
}

// TODO: figure out how testing needs to work
#[cfg(test)]
mod tests {
    use super::*;

    const MID: u16 = (SERVO_MICROS_MIN + SERVO_MICROS_MAX) / 2;

    #[test]
    fn untrimmed_center_is_the_pulse_midpoint() {
        assert_eq!(angle_to_micros(0.0, 0.0), MID);
    }

    #[test]
    fn untrimmed_travel_spans_the_full_pulse_range() {
        // Negative deflection lengthens the pulse, positive shortens it.
        assert_eq!(angle_to_micros(SERVO_MIN_ANGLE, 0.0), SERVO_MICROS_MAX);
        assert_eq!(angle_to_micros(SERVO_MAX_ANGLE, 0.0), SERVO_MICROS_MIN);
    }

    /// Regression: the previous normalized-progress formula computed a negative
    /// intermediate near full positive deflection, and `f64 as u16` saturates
    /// to 0 rather than wrapping — so +47°..+50° all produced 1000 µs. Every
    /// degree of authority must actually move the pulse.
    #[test]
    fn every_degree_of_travel_moves_the_pulse() {
        let mut prev = angle_to_micros(SERVO_MIN_ANGLE, 0.0);
        let mut deg = SERVO_MIN_ANGLE as i32 + 1;
        while deg <= SERVO_MAX_ANGLE as i32 {
            let us = angle_to_micros(deg as Deg, 0.0);
            assert!(us < prev, "pulse did not move at {deg}°: {prev} -> {us}");
            prev = us;
            deg += 1;
        }
    }

    #[test]
    fn trim_shifts_the_whole_pulse_window() {
        assert_eq!(angle_to_micros(0.0, -3.0), MID - 30);
        assert_eq!(angle_to_micros(0.0, 2.5), MID + 25);
        // A trim is just an offset on the command, not a change of scale.
        assert_eq!(angle_to_micros(10.0, -3.0), angle_to_micros(13.0, 0.0));
    }

    /// Per the chosen policy, trim never pushes a pulse outside the servo's
    /// endpoints — it costs deflection at one end instead.
    #[test]
    fn trim_clamps_rather_than_exceeding_servo_endpoints() {
        assert_eq!(angle_to_micros(47.0, -3.0), SERVO_MICROS_MIN);
        assert_eq!(angle_to_micros(SERVO_MAX_ANGLE, -3.0), SERVO_MICROS_MIN);
        // ...while the opposite end keeps its full travel.
        assert_eq!(angle_to_micros(SERVO_MIN_ANGLE, -3.0), SERVO_MICROS_MAX - 30);
    }

    #[test]
    fn commands_beyond_mechanical_authority_clamp() {
        assert_eq!(angle_to_micros(180.0, 0.0), angle_to_micros(SERVO_MAX_ANGLE, 0.0));
        assert_eq!(angle_to_micros(-180.0, 0.0), angle_to_micros(SERVO_MIN_ANGLE, 0.0));
    }
}