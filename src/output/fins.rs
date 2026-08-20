use defmt::{error, info, Debug2Format};
use embassy_rp::pwm;
use embassy_time::{Duration, Ticker};
use crate::config::board::{ServoConfig, Servos};
use crate::config::MOMENT_OF_INERTIA;
use crate::{mark_init_failed, Subsystem, mark_init_complete};
use crate::math::Deg;
use crate::orientation::ahrs::AhrsState;
use crate::output::roll_controller;
use crate::output::servo::init_servos;

// TODO: Here uom would probably be great.
pub fn angular_accel_to_fin_deflection_angle(ahrs: &AhrsState, ang_accel_desired: f64) -> Deg {
    let torque_desired: f64 = ang_accel_desired * MOMENT_OF_INERTIA; // τ = I * α

    // adjusted: Look up effectiveness AT ZERO deflection
    let effectiveness_zero: f64 = roll_controller::effectiveness(ahrs.get_velocity_earth());

    // Compute fin deflection using linear approximation
    if effectiveness_zero <= 1e-9 {
        // If effectiveness is too low, we can't control, so return zero deflection
        return 0.0
    }
    let fin_deflection_angle: f64 = torque_desired / effectiveness_zero;
    fin_deflection_angle
}

pub async fn fins_loop(servo_config: ServoConfig) {
    info!("initializing servos");
    let mut fins = match init_servos(servo_config) {
        Ok(servos) => {
            info!("servos initialized");
            mark_init_complete(Subsystem::CONTROL);
            servos
        }
        Err(e) => {
            error!("Error initializing servos: {:?}", Debug2Format(&e));
            mark_init_failed(Subsystem::CONTROL);
            return;
        }
    };
    info!("servos initialized");

    let mut ticker = Ticker::every(Duration::from_hz(crate::output::servo::SERVO_PWM_HZ as u64));
    loop {
        ticker.next().await;
        // if let Some(sp) = setpoints.try_get() {
        //     // TODO: write to each fin
        // }
        // TODO: Figure out what tick rate I actually want and how to communicate it. Maybe just have an atomic or Watch for the servo angles?
        //  Or maybe move the final angular acceleration -> servo angle calculation over to this file and have a Watch for desired pitch, roll, yaw accel?
        //  Or move PID here?
        //  This definitely doesn't need to be faster than 50Hz, since that's the PWM speed.
    }
}
