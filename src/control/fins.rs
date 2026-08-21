use crate::config::board::{ServoConfig, Servos};
use crate::config::{MOMENT_OF_INERTIA, TORQUE_PER_DEG_50MS};
use crate::control::servo;
use crate::control::servo::init_servos;
use crate::navigation::ahrs::AhrsState;
use crate::utils::math::{Deg, Vec3};
use crate::{Subsystem, mark_init_complete, mark_init_failed};
use defmt::{Debug2Format, error, info};
use embassy_time::{Duration, Ticker};

/// Effectiveness is the slope of the deflection vs torque curve at zero deflection, which is what we want for the linear approximation. We can adjust it later if we want to get fancy and account for nonlinearity at higher deflections.
/// Using deflection in degrees, so effectiveness is in Nm/deg
pub fn effectiveness(velocity_earth: Vec3) -> f64 {
    let v = velocity_earth.mag();
    TORQUE_PER_DEG_50MS * (v * v) / (50.0 * 50.0)
}

// TODO: Here uom would probably be great.
pub fn angular_accel_to_fin_deflection_angle(velocity_earth: Vec3, ang_accel_desired: f64) -> Deg {
    let torque_desired: f64 = ang_accel_desired * MOMENT_OF_INERTIA; // τ = I * α

    // adjusted: Look up effectiveness AT ZERO deflection
    let effectiveness_zero: f64 = effectiveness(velocity_earth);

    // Compute fin deflection using linear approximation
    if effectiveness_zero <= 1e-9 {
        // If effectiveness is too low, we can't control, so return zero deflection
        return 0.0;
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

    let mut ticker = Ticker::every(Duration::from_hz(servo::SERVO_PWM_HZ as u64));
    loop {
        ticker.next().await;

        // angular_accel_to_fin_deflection_angle(ahrs, ang_accel_desired);
        // if let Some(sp) = setpoints.try_get() {
        //     // TODO: write to each fin
        // }
        // TODO: Figure out what tick rate I actually want and how to communicate it. Maybe just have an atomic or Watch for the servo angles?
        //  Or maybe move the final angular acceleration -> servo angle calculation over to this file and have a Watch for desired pitch, roll, yaw accel?
        //  Or move PID here?
        //  This definitely doesn't need to be faster than 50Hz, since that's the PWM speed.
    }
}
