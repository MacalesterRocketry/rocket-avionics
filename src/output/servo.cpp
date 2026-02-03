#include "servo.h"

#include <Arduino.h>
#include <Servo.h>
#include "flags.h"

#include "millis64.h"
#include "utils.h"

int32_t calc_servo_micros_position(const double_t progress) { // progress: 0.0 to 1.0
  return SERVO_MICROS_MIN + static_cast<int32_t>((SERVO_MICROS_MAX - SERVO_MICROS_MIN) * progress);
}

void sweep_servo(Servo &servo, const double_t duration_seconds, const bool backwards=false) {
  const uint64_t duration_us = static_cast<uint64_t>(duration_seconds * 1e6);
  const uint64_t startTime = micros64();

  while (true) {
    const uint64_t elapsed = micros64() - startTime;

    // Calculate the progress ratio (0.0 to 1.0), where 1.0 means the sweep is complete
    const double progress = static_cast<double>(elapsed) / duration_us;

    if (progress >= 1.0) {
      if (backwards)
        servo.writeMicroseconds(SERVO_MICROS_MIN);
      else
        servo.writeMicroseconds(SERVO_MICROS_MAX);
      break;
    }

    const int32_t current_pos = backwards ?
      calc_servo_micros_position(1.0 - progress) :
      calc_servo_micros_position(progress);

    servo.writeMicroseconds(current_pos);
  }
}

void init_servos() {
  servoXPlus.attach(SERVO_XPLUS_PIN);
  servoXMinus.attach(SERVO_XMINUS_PIN);
  servoYPlus.attach(SERVO_YPLUS_PIN);
  servoYMinus.attach(SERVO_YMINUS_PIN);
}

// So 0° is in line with the fin, positive angles rotate it one way, and negative angles rotate it the other way.
// TODO: Figure out which direction it's rotated and how that affects PID.
void set_servo_angle(Servo& servo, const double_t angle_degrees_from_neutral) {
  constexpr double max_deflection = SERVO_DEGREE_RANGE / 2.0;
  constexpr double neutral_angle = max_deflection; // can calibrate if need be
  const double_t clamped_angle = clamp(angle_degrees_from_neutral, -max_deflection, max_deflection);
  const double_t progress = (clamped_angle + neutral_angle) / SERVO_DEGREE_RANGE; // 0.0 to 1.0
  const int32_t micros_position = calc_servo_micros_position(progress);
  servo.writeMicroseconds(micros_position);
}

void test_servo(Servo& servo) {
  constexpr double_t duration = 2.0; // seconds
  sweep_servo(servo, duration, false);
  sweep_servo(servo, duration, true);
  servo.write(90); // return to center
}

void servo_test_loop() {
  // Sweep X+ servo
  test_servo(servoXPlus);
  // Sweep X- servo
  test_servo(servoXMinus);
  // Sweep Y+ servo
  test_servo(servoYPlus);
  // Sweep Y- servo
  test_servo(servoYMinus);
}