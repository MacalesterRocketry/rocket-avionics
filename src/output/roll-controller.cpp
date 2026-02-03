#include "states.h"
#include "orientation/ahrs.h"
#include "output/roll-controller.h"
#include "output/servo.h"
#include "utils.h"

#include "millis64.h"

void calculate_and_execute_torque(const Vec3 &gyro, const Quat &qtarget, double &integral, const double dt) {
  // ============================================
  // GET CURRENT STATE
  // ============================================

  Quat qcurrent = get_current_orientation();         // Updated Q4 from AHRS

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
  double torque_desired = torque_p + torque_i + torque_d;
  torque_desired = clamp(torque_desired, -max_torque, max_torque);

  // CFD MAPPING
  // Get flight conditions
  double mach = calculate_mach_number();
  double attack_angle = calculate_angle_of_attack();

  // Look up effectiveness from CFD table
  const double effectiveness = cfd_lookup(mach, attack_angle, "dτ/dδ"); // TODO: figure out what dτ/dδ is supposed to be
                                                                        //  Note: δ was prior name for fin_deflection_angle

  // Compute fin deflection
  const double fin_deflection_angle = torque_desired / effectiveness;

  // Command servos
  for (Servo &servo : servos) {
    set_servo_angle(servo, fin_deflection_angle);
  }

  // DISPLAY CURRENT ROLL
  // Extract roll angle from qcurrent
  double current_roll = extract_roll_angle(qcurrent);
  Serial.println("Current roll: " + degrees(current_roll) + "°");


}

void pid(double target_angle, double current_angle, const Vec3 &gyroRaw) {
  // Start
  Quat qcurrent = get_current_orientation(); // Current orientation (Q4)
  // If it is angled, we need to adjust that tilt vefore storing the orientation)

  Serial.println("Stabilized at Q4, ready for target_angles"); //would be good to confirm where we are

  // BUILD ROLL QUATERNION

  // Convert target_angle to body-frame rotation since q4 is earth frame
  double roll_angle_rad = -target_angle * PI / 180;  // Negative for body frame
  Quat qroll = Quat{cos(roll_angle_rad/2), sin(roll_angle_rad/2), 0, 0}; // Roll by 90°


  // COMPUTE TARGET ORIENTATION

  // Apply roll to stabilized orientation
  Quat qtarget = (qcurrent * qroll).normalized(); // Target orientation; TODO: figure out how to input (use initial orientation somehow?)

  // CONTROL LOOP (Until target reached)
  double integral = 0; // TODO: Should this be static and global?
  uint64_t last_time = micros64();
  while (abs(target_angle - current_angle) > 1) { // TODO: convert to if so it's non-blocking
    uint64_t now = micros64();
    uint64_t dt = now - last_time; // TODO: micros or seconds?
    calculate_and_execute_torque(gyroRaw, qtarget, integral, dt);
    last_time = now;
    // LOOP TIMING

    wait(10);  // 100Hz control loop
  }

  // Continue stabilization loop indefinitely
  while (true) {
    double integral = 0.0;
    // TODO: This was set up to ignore the I term, presumably for small tweaks. Are we sure we want that? If so, how can we figure out when to use it? Or just gradually scale down I?
    calculate_and_execute_torque(gyroRaw, qtarget, integral, 0.0);
    // LOOP TIMING
    wait(10);  // 100Hz control loop
  }
}
