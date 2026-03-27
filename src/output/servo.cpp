#include "output/servo.h"
#include "config.h"

#include "utils.h"

#include <Servo.h>

static Servo servoXPlus;
static Servo servoXMinus;
static Servo servoYPlus;
static Servo servoYMinus;

int32_t calc_servo_micros_position(const double_t progress) {
  // progress: 0.0 to 1.0
  return SERVO_MICROS_MIN + static_cast<int32_t>((SERVO_MICROS_MAX - SERVO_MICROS_MIN) * progress);
}

Servo *get_servo(const ServoID id) {
  switch (id) {
    case SERVO_XPLUS:
      return &servoXPlus;
    case SERVO_XMINUS:
      return &servoXMinus;
    case SERVO_YPLUS:
      return &servoYPlus;
    case SERVO_YMINUS:
      return &servoYMinus;
    default:
      return &servoXMinus; // default to something so it compiles, but this should never happen
  }
}

void sweep_servo(const ServoID servo, const double_t duration_seconds, const bool backwards = false) {
  const uint64_t duration_us = static_cast<uint64_t>(duration_seconds * 1e6);
  const uint64_t startTime = micros64();

  while (true) {
    const uint64_t elapsed = micros64() - startTime;

    // Calculate the progress ratio (0.0 to 1.0), where 1.0 means the sweep is complete
    const double progress = static_cast<double>(elapsed) / duration_us;

    if (progress >= 1.0) {
      if (backwards)
        set_servo_angle(servo, -(SERVO_DEGREE_RANGE / 2.0)); // return to min angle
      else
        set_servo_angle(servo, SERVO_DEGREE_RANGE / 2.0); // return to max angle
      break;
    }

    double angle_coefficient = progress - 0.5;
    if (backwards) angle_coefficient = -angle_coefficient;
    set_servo_angle(servo, SERVO_DEGREE_RANGE * angle_coefficient);
  }
}

void init_servos() {
  servoXPlus.attach(SERVO_XPLUS_PIN);
  servoXMinus.attach(SERVO_XMINUS_PIN);
  servoYPlus.attach(SERVO_YPLUS_PIN);
  servoYMinus.attach(SERVO_YMINUS_PIN);
  for (const ServoID servo : servos) {
    set_servo_angle(servo, 0); // start at neutral position
  }
}

// So 0° is in line with the fin, positive angles rotate it one way, and negative angles rotate it the other way.
// TODO: Figure out which direction it's rotated and how that affects PID.
void set_servo_angle(const ServoID servo, const double_t angle_degrees_from_neutral) {
  constexpr double max_deflection = SERVO_DEGREE_RANGE / 2.0;
  constexpr double neutral_angle = max_deflection; // can calibrate if need be
  const double_t clamped_angle = clamp(angle_degrees_from_neutral, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE) + SERVO_NEUTRAL_ANGLE;
  const double_t progress = (clamped_angle + neutral_angle) / SERVO_DEGREE_RANGE; // 0.0 to 1.0
  const int32_t micros_position = calc_servo_micros_position(progress);
  get_servo(servo)->writeMicroseconds(micros_position);
}

void test_servo(const ServoID servo) {
  constexpr double_t duration = 2.0; // seconds
  sweep_servo(servo, duration, false);
  sweep_servo(servo, duration, true);
  set_servo_angle(servo, 0); // return to center
}

void servo_test_loop() {
  // Sweep X+ servo
  test_servo(SERVO_XPLUS);
  // Sweep X- servo
  test_servo(SERVO_XMINUS);
  // Sweep Y+ servo
  test_servo(SERVO_YPLUS);
  // Sweep Y- servo
  test_servo(SERVO_YMINUS);
}