#include <Arduino.h>

#include "flags.h"

#if SERVO_TESTING
#include <Servo.h>

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

void sweep_servo(Servo &servo, const int32_t DEG_INCREMENT, const uint16_t DELAY_MS) {
  int32_t pos = 0;
  while (pos >= 0 && pos <= 180) { // 180 is the max value, regardless of physical servo limits
    servo.write(pos);
    delay(DELAY_MS);
    pos += DEG_INCREMENT;
  }
}

void test_servo(Servo& servo, const int32_t DEG_INCREMENT, const uint16_t DELAY_MS) {
  sweep_servo(servo, DEG_INCREMENT, DELAY_MS);
  sweep_servo(servo, -DEG_INCREMENT, DELAY_MS);
  servo.write(90); // return to center
}

void loop() {
  constexpr int16_t DEG_INCREMENT = 1;
  constexpr uint16_t DELAY_MS = 15;
  // Sweep X+ servo
  test_servo(servoXPlus, DEG_INCREMENT, DELAY_MS);
  // Sweep X- servo
  test_servo(servoXMinus, DEG_INCREMENT, DELAY_MS);
  // Sweep Y+ servo
  test_servo(servoYPlus, DEG_INCREMENT, DELAY_MS);
  // Sweep Y- servo
  test_servo(servoYMinus, DEG_INCREMENT, DELAY_MS);
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