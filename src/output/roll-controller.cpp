#include "output/roll-controller.h"
#include "config.h"

#include "orientation/ahrs.h"
#include "output/servo.h"
#include "utils.h"
#include "output/sdcard.h"

// Effectiveness is the slope of the deflection vs torque curve at zero deflection, which is what we want for the linear approximation. We can adjust it later if we want to get fancy and account for nonlinearity at higher deflections.
// Using deflection in degrees, so effectiveness is in Nm/deg
double calculate_effectiveness(const Vec3 &velocity) {
  return 1.0 / 360; // Placeholder value
}

Deg calculate_deflection_pid(const Quat& qtarget, const double dt) {
  static double integral = 0;
  const Quat qcurrent = get_orientation();

  // ============================================
  // ERROR CALCULATION
  // ============================================

  // Compute quaternion error
  const Quat qrollerror = (qcurrent.conjugate() * qtarget).normalized();

  // Convert to angular error
  const double eroll_y = 2 * qrollerror.y; // Y-component = roll error TODO: Are we sure it's that? I would have assumed it would need to be Euler before we can assume y is roll.
  // double eroll_y_test = calculate_roll_deg(qrollerror);

  // TORQUE PID CALCULATION

  // PID terms
  const double ang_accel_p = ROLL_PID_Kp * eroll_y;
  integral += eroll_y * dt;
  const double ang_accel_i = ROLL_PID_Ki * integral;
  const double ang_accel_d = -ROLL_PID_Kd * get_angular_velocity().y;

  // Total desired angular acceleration
  const double ang_accel_desired = ang_accel_p + ang_accel_i + ang_accel_d;
  const double torque_desired = ang_accel_desired * MOMENT_OF_INERTIA; // τ = I * α

  // adjusted: Look up effectiveness AT ZERO deflection
  const double effectiveness_zero = calculate_effectiveness(get_velocity());

  // Compute fin deflection using linear approximation
  const double fin_deflection_angle = torque_desired / effectiveness_zero;

  return fin_deflection_angle;
}

void update_roll(const Deg target_angle, const Quat& base_orientation) {
  // Start
  const Quat qcurrent = get_orientation(); // Current orientation (Q4)
  const Deg current_angle = calculate_roll_deg(qcurrent); // Current roll angle in degrees (from Q4)

  // If it is angled, we need to adjust that tilt before storing the orientation)
  // TODO: Figure out what this comment means

  // BUILD ROLL QUATERNION
  // const Quat qroll = Quat{cos(roll_angle_rad/2), sin(roll_angle_rad/2), 0, 0}; // Roll by 90°
  // TODO: The old was x: sin(angle/2), but the new one has x: sin(angle). Should the Vec3 passed into axisAngleToQuat be {0.5, 0, 0}? Check this all with Tala.
  const Quat qroll = roll_deg_to_quat(-target_angle); // Negative for body frame
  const Quat qtarget = (base_orientation * qroll).normalized(); // Target orientation; TODO: This probably has different pitch and yaw than what we actually want. Replace them with the current ones from qcurrent? Or just ignore them since we're only controlling roll?

  // CONTROL LOOP (Until target reached)
  static uint64_t last_time = micros64();
  if (abs(target_angle - current_angle) > 1) {
    const uint64_t now_micros = micros64();
    const double dt = (now_micros - last_time) / 1000000.0;
    const double fin_deflection_angle = calculate_deflection_pid(qtarget, dt);
    logRollControl(target_angle, current_angle, fin_deflection_angle);
#if DEBUG and DEBUG_PRINT_ROLL_CONTROL
    Serial.print("Target Roll: ");
    Serial.print(target_angle);
    Serial.print("°, Current Roll: ");
    Serial.print(current_angle);
    Serial.print("°, Fin Deflection: ");
    Serial.print(fin_deflection_angle);
    Serial.println("°");
#endif

    // Command servos
    for (const ServoID servo : servos) {
      set_servo_angle(servo, fin_deflection_angle);
    }

    last_time = now_micros;
  }
}
