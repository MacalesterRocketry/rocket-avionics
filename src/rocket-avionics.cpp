#include <Arduino.h>

#include "flags.h"

#if SERVO_TESTING
#include <Servo.h>

#include "millis64.h"
#include "utils.h"

Servo servoXPlus;
Servo servoXMinus;
Servo servoYPlus;
Servo servoYMinus;

void setup() {
  servoXPlus.attach(SERVO_XPLUS_PIN);
  servoXMinus.attach(SERVO_XMINUS_PIN);
  servoYPlus.attach(SERVO_YPLUS_PIN);
  servoYMinus.attach(SERVO_YMINUS_PIN);
}

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

void test_servo(Servo& servo) {
  constexpr double_t duration = 2.0; // seconds
  sweep_servo(servo, duration, false);
  sweep_servo(servo, duration, true);
  servo.write(90); // return to center
}

void loop() {
  // Sweep X+ servo
  test_servo(servoXPlus);
  // Sweep X- servo
  test_servo(servoXMinus);
  // Sweep Y+ servo
  test_servo(servoYPlus);
  // Sweep Y- servo
  test_servo(servoYMinus);
}

#else // !SERVO_TESTING
#include "states.h"
#include "utils.h"
#include "output/sdcard.h"
#include "orientation/sensors.h"
#if USE_GPS
#include "orientation/gps.h"
#endif

void setup() {
  setState(STATE_STARTING);
  initIndicators();

#if DEBUG
  Serial.begin(115200);
  while (!Serial)
    handleState();
#endif

  wait(500);

  Wire.setClock(400000); // set i2c clock to 400kHz (fast mode)

  pinMode(EJECT_BUTTON, INPUT_PULLUP);
  initSDCard();
  initSensors();
#if USE_GPS
  initGPS();
#endif

  setState(STATE_READY_TO_LAUNCH);
}

void loop() {
  handleState();
}
#endif // SERVO_TESTING