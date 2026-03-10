#include "states.h"
#include "orientation/ahrs.h"
#include "output/roll-controller.h"
#include "output/servo.h"
#include "utils.h"

#include "millis64.h"

// Effectiveness is the slope of the deflection vs torque curve at zero deflection, which is what we want for the linear approximation. We can adjust it later if we want to get fancy and account for nonlinearity at higher deflections.
// Using deflection in degrees, so effectiveness is in Nm/deg
double calculate_effectiveness(const Vec3 &velocity, double attack_angle, double deflection_angle) {
  return 1; // Placeholder value
}

void calculate_and_execute_torque(const Vec3 &gyro, const Quat &qtarget, double &integral, const double dt) {
  // ============================================
  // GET CURRENT STATE
  // ============================================

  Quat qcurrent = get_orientation();         // Updated Q4 from AHRS

  // ============================================
  // ERROR CALCULATION
  // ============================================

  // Compute quaternion error
  Quat qrollerror = (qcurrent.conjugate() * qtarget).normalized();

  // Convert to angular error
  double eroll_x = 2 * qrollerror.x;        // X-component = roll error

  // TORQUE PID CALCULATION

  // PID terms
  double torque_p = Kp * eroll_x;
  integral += eroll_x * dt;
  double torque_i = Ki * integral;
  double torque_d = -Kd * gyro.x;

  // Total desired torque
  // TODO: Do we need to incorporate moment of inertia into this?
  double torque_desired = torque_p + torque_i + torque_d;
  torque_desired = clamp(torque_desired, -SERVO_MAX_TORQUE, SERVO_MAX_TORQUE);

  // CFD MAPPING
  // Mach and aoa
  Vec3 velocity = get_velocity();
  double attack_angle = calculate_angle_of_attack(); // TODO: Can we just assume this is 0 and the fins keep it straight? If not, how do we calculate it? Maybe the angle between the velocity vector and the orientation.

  // adjusted: Look up effectiveness AT ZERO deflection
  const double effectiveness_zero = calculate_effectiveness(velocity, attack_angle, 0);

  // Compute fin deflection using linear approximation
  const double fin_deflection_angle = torque_desired / effectiveness_zero;

  // Command servos
  for (Servo &servo : servos) {
    set_servo_angle(servo, fin_deflection_angle);
  }

#if DEBUG
  Deg current_roll = calculate_roll_deg(qcurrent);
  Serial.print("Current Roll: ");
  Serial.println(current_roll);
#endif
}

void pid(const Deg target_angle, const Deg current_angle, const Vec3 &gyroRaw) {
  // Start
  const Quat qcurrent = get_orientation(); // Current orientation (Q4)
  // If it is angled, we need to adjust that tilt before storing the orientation)

  Serial.println("Stabilized at Q4, ready for target_angles"); //would be good to confirm where we are

  // BUILD ROLL QUATERNION

  // Convert target_angle to body-frame rotation since q4 is earth frame
  const double roll_angle_rad = degToRad(-target_angle);  // Negative for body frame
  const Quat qroll = Quat{cos(roll_angle_rad/2), sin(roll_angle_rad/2), 0, 0}; // Roll by 90°


  // COMPUTE TARGET ORIENTATION

  // Apply roll to stabilized orientation
  const Quat qtarget = (qcurrent * qroll).normalized(); // Target orientation; TODO: This is based on the current orientation, not the initial, so it will always try to roll. Figure out how to fix.

  // CONTROL LOOP (Until target reached)
  double integral = 0; // TODO: Should this be static and global?
  uint64_t last_time = micros64();
  while (abs(target_angle - current_angle) > 1) { // TODO: convert to if so it's non-blocking
    const uint64_t now_micros = micros64();
    const double dt = (now_micros - last_time) / 1000000.0;
    calculate_and_execute_torque(gyroRaw, qtarget, integral, dt);
    last_time = now_micros;
  }
}
