//! Fin effector model.
//!
//! Converts a desired angular acceleration into per-servo deflections. This is
//! the layer that knows torque comes from aerodynamic surfaces — swap this
//! module out for thrust-vectoring or a reaction wheel and neither `pid.rs` nor
//! the state machine changes, because both sides of it speak
//! [`AngularVec3`] and volts-free servo angles.
//!
//! Which fin contributes what to each axis lives in [`FIN_MIX`], not here, so a
//! different fin count or arrangement is a config edit.

use crate::config::board::Servos;
use crate::config::{FIN_MIX, MOMENT_OF_INERTIA, TORQUE_PER_DEG_50MS};
use crate::utils::math::{AngularVec3, Deg, Vec3};
use embassy_rp::pwm;

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

/// Deflect every fin to serve `ang_accel`.
///
/// Each fin's deflection is the sum of its per-axis contributions from
/// [`FIN_MIX`]. Only roll is populated today, so this reduces to the same angle
/// on every fin, since roll authority is the same regardless of fin placement
/// around the longitudinal axis of the rocket.
///
/// Servos are zipped against the mix table rather than addressed by name; both
/// orderings come from one macro expansion, so they cannot drift apart.
pub fn apply(
    servos: &mut Servos,
    velocity_earth: Vec3,
    ang_accel: AngularVec3,
) -> Result<(), pwm::PwmError> {
    // TODO: FIN_MIX is the forward map (deflection → torque), so summing its
    //  columns like this is really using its transpose. For a symmetric fin set
    //  the rows are orthogonal and the transpose is proportional to the
    //  pseudo-inverse, so this is correct — and for roll-only it is exact. An
    //  asymmetric arrangement would need the actual pseudo-inverse, or it gets
    //  silent cross-axis coupling.
    // TODO: Summed multi-axis demands can exceed the servo travel limits, and
    //  ServoOutput clamps per-servo, which distorts whichever axis mattered
    //  most. Wants real control allocation once pitch/yaw are live.
    for (servo, mix) in servos.iter_mut().zip(FIN_MIX.iter()) {
        let demand = ang_accel.roll * mix.roll
            + ang_accel.pitch * mix.pitch
            + ang_accel.yaw * mix.yaw;
        servo.set_angle(angular_accel_to_fin_deflection_angle(velocity_earth, demand))?;
    }
    Ok(())
}