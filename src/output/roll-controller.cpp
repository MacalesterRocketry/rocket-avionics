#include "output/roll-controller.h"
#include "config.h"

#include "orientation/ahrs.h"
#include "output/servo.h"
#include "utils.h"
#include "output/sdcard.h"


bool leftTurnSignalPinOn = false;
bool rightTurnSignalPinOn = false;
void signalLeftTurn() {
  // multiplexed, so left is TURN_SIGNAL_LEFT_PIN HIGH and TURN_SIGNAL_RIGHT_PIN LOW
  if (!leftTurnSignalPinOn) {
    leftTurnSignalPinOn = true;
    digitalWriteFast(TURN_SIGNAL_LEFT_PIN, HIGH);
  }
  if (rightTurnSignalPinOn) {
    rightTurnSignalPinOn = false;
    digitalWriteFast(TURN_SIGNAL_RIGHT_PIN, LOW);
  }
}

void signalRightTurn() {
  // multiplexed, so right is TURN_SIGNAL_RIGHT_PIN HIGH and TURN_SIGNAL_LEFT_PIN LOW
  if (!rightTurnSignalPinOn) {
    rightTurnSignalPinOn = true;
    digitalWriteFast(TURN_SIGNAL_RIGHT_PIN, HIGH);
  }
  if (leftTurnSignalPinOn) {
    leftTurnSignalPinOn = false;
    digitalWriteFast(TURN_SIGNAL_LEFT_PIN, LOW);
  }
}

void clearTurnSignal() {
  if (leftTurnSignalPinOn) {
    leftTurnSignalPinOn = false;
    digitalWriteFast(TURN_SIGNAL_LEFT_PIN, LOW);
  }
  if (rightTurnSignalPinOn) {
    rightTurnSignalPinOn = false;
    digitalWriteFast(TURN_SIGNAL_RIGHT_PIN, LOW);
  }
}


// Effectiveness is the slope of the deflection vs torque curve at zero deflection, which is what we want for the linear approximation. We can adjust it later if we want to get fancy and account for nonlinearity at higher deflections.
// Using deflection in degrees, so effectiveness is in Nm/deg
double calculate_effectiveness(const Vec3 &velocity) {
  const double v = velocity.mag();
  const double predicted_torque = TORQUE_PER_DEG_50MS * (v * v) / (50.0 * 50.0); // Scale the effectiveness based on velocity squared, normalized to 50 m/s
  return predicted_torque; // Nm/deg
}

Deg calculate_deflection_pid(const Quat& qtarget, const double dt) {
  static double integral = 0;
  const Quat qcurrent = get_orientation_earth();

  // ============================================
  // ERROR CALCULATION
  // ============================================

  // Compute quaternion error
  const Quat qrollerror = (qcurrent.conjugate() * qtarget).normalized();
  //const Quat qrollerror = (qtarget.conjugate() * qcurrent).normalized();
  // TODO: This is the geodesic path, not the total path, so it can get confused since moving away from the target might actually move it closer. Fix?
  // TODO: Also, is this in earth-frame or body frame? If we were also controlling pitch and yaw, that wouldn't matter, but we need to be sure the axis is right.
  //  This is definitely earth frame, since it's based on sources that also control pitch and yaw. We need to transform it to body.

  // Convert to angular error
  const double eroll_y = -2.0 * qrollerror.y; // Y-component = roll error
  // double eroll_y_test = calculate_roll_deg(qrollerror);
  // TODO: I've found two possible errors from the simulation. First, we need to confirm that y is the right component; if it's not, we'll have problems like what we got.
  //  Second, qtarget may have the wrong sign of target going in. Not a big deal, and it might not actually be true outside of sim, but it's something to consider.

  // TORQUE PID CALCULATION

  // PID terms
  const double ang_accel_p = ROLL_PID_Kp * eroll_y;
  integral += eroll_y * dt;
  const double ang_accel_i = ROLL_PID_Ki * integral;
  const double ang_accel_d = -ROLL_PID_Kd * get_angular_velocity_body().y;

  // Total desired angular acceleration
  const double ang_accel_desired = ang_accel_p + ang_accel_i + ang_accel_d;
  const double torque_desired = ang_accel_desired * MOMENT_OF_INERTIA; // τ = I * α

  // adjusted: Look up effectiveness AT ZERO deflection
  const double effectiveness_zero = calculate_effectiveness(get_velocity_earth());

  // Compute fin deflection using linear approximation
  if (effectiveness_zero <= 1e-9) {
    // If effectiveness is too low, we can't control, so return zero deflection
    return 0;
  }
  const double fin_deflection_angle = torque_desired / effectiveness_zero;

  return fin_deflection_angle;
}

void update_roll(const Deg target_angle, const Quat& base_orientation) {
  // Start
  const Quat qcurrent = get_orientation_earth(); // Current orientation (Q4)
  const Deg current_angle = calculate_roll_deg(qcurrent); // Current roll angle in degrees (from Q4)
  // TODO: I'm pretty sure this current angle is just wrong. It should be in body frame, not earth frame.
  //  Edit: Looking at the data compared to the integrated gyro data, it's very close throughout most of the flight but differs greatly when the rocket tilts at apogee. As a result, I think it is earth-frame.
  //  But it's not used for anything, so it may not matter.

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
    if (fin_deflection_angle > 0) {
      signalRightTurn();
    } else {
      signalLeftTurn();
    }
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
  } else {
    // Clear turn signals when we're close enough to the target
    clearTurnSignal();
  }
}
