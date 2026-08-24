//! Attitude control.
//!
//! One task at the servo frame rate owns the whole chain: read the latest AHRS
//! solution and setpoint, run the PID, mix to fin deflections, write the servos.
//!
//! It deliberately does *not* run at the sensor rate. The PWM frame is 20 ms and
//! the servos themselves respond at roughly 10 Hz, so 50 Hz already oversamples
//! the actuator by ~10x and a faster compensator buys no closed-loop
//! performance. Splitting PID and servo writes into separate tasks would be
//! worse still: it adds up to another full period of actuation delay, and at a
//! ~5 Hz bandwidth 20 ms is already ~36° of phase. Keeping it in one task also
//! means the PID error and the effectiveness lookup come from the same AHRS
//! snapshot rather than two different instants.
//!
//! The three layers stay separate as modules, not as tasks: [`pid`] is the
//! control law, [`fins`] the effector model, [`servo`] the hardware.

pub mod fins;
pub mod pid;
pub mod servo;

use crate::config::board::ServoConfig;
use crate::control::pid::RollPid;
use crate::control::servo::init_servos;
use crate::navigation::ahrs::AHRS_STATE;
use crate::state::{CONTROL_SETPOINT, ControlSetpoint};
use crate::{mark_init_complete, mark_init_failed, mark_runtime_error, Subsystem, has_runtime_error, clear_runtime_error};
use defmt::{Debug2Format, error, info, warn};
use embassy_time::{Duration, Ticker};

pub async fn control_loop(servo_config: ServoConfig) {
    info!("initializing servos");
    let mut servos = match init_servos(servo_config) {
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

    let Some(mut ahrs_rx) = AHRS_STATE.receiver() else {
        error!("Failed to get AHRS receiver; have too many receivers been initialized?");
        mark_init_failed(Subsystem::CONTROL);
        return;
    };
    let Some(mut setpoint_rx) = CONTROL_SETPOINT.receiver() else {
        error!("Failed to get control setpoint receiver; have too many receivers been initialized?");
        mark_init_failed(Subsystem::CONTROL);
        return;
    };

    let mut pid = RollPid::new();
    let mut armed = false;

    let mut ticker = Ticker::every(Duration::from_hz(servo::SERVO_PWM_HZ as u64));
    loop {
        ticker.next().await;

        // `try_get` rather than `changed`: the ticker decides when to actuate,
        // and the setpoint is whatever is current at that moment. Treat a
        // missing setpoint or AHRS solution as disarmed — before the first
        // sensor sample there is nothing to control against.
        let setpoint = setpoint_rx.try_get().unwrap_or(ControlSetpoint::Disarmed);
        let ahrs = ahrs_rx.try_get();

        let command = match (setpoint, ahrs) {
            (ControlSetpoint::Attitude(qtarget), Some(ahrs)) => {
                if !armed {
                    // Drop any integral accumulated before this arming.
                    pid.reset();
                    armed = true;
                    info!("control armed");
                }
                Some((pid.step(qtarget, &ahrs), ahrs.get_velocity_earth()))
            }
            _ => {
                if armed {
                    armed = false;
                    info!("control disarmed");
                }
                None
            }
        };

        let result = match command {
            Some((ang_accel, velocity_earth)) => {
                fins::apply(&mut servos, velocity_earth, ang_accel)
            }
            None => servos.center_all(),
        };
        if let Err(e) = result {
            warn!("servo write failed: {:?}", Debug2Format(&e));
            mark_runtime_error(Subsystem::CONTROL);
        } else if has_runtime_error(Subsystem::CONTROL) {
            info!("servo write succeeded, clearing runtime error");
            clear_runtime_error(Subsystem::CONTROL);
        }
    }
}