#include "config.h"
#include "orientation/ahrs.h"

#if SERVO_TESTING
#include "output/servo.h"
void setup() {
  init_servos();
  Serial.begin(115200);
}

void loop() {
  servo_test_loop();
  // read angle from serial input and set servo to that angle
  // if (Serial.available() > 0) {
  //   String input = Serial.readStringUntil('\n');
  //   double angle = input.toDouble();
  //   Serial.print("Setting servo angle to: ");
  //   Serial.println(angle);
  //   set_servo_angle(SERVO_XMINUS, angle);
  // }
}
#else // !SERVO_TESTING
#include <Wire.h>

#include "states.h"
#include "utils.h"
#include "orientation/sensors.h"
#if USE_GPS
#include "orientation/gps.h"
#endif
#include "output/sdcard.h"
#include "output/servo.h"

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
#if DEBUG
  Serial.println("SD card initialized.");
#endif
  initSensors();
#if DEBUG
  Serial.println("Sensors initialized.");
#endif
#if USE_GPS
  initGPS();
#if DEBUG
  Serial.println("GPS initialized.");
#endif
#endif
  init_servos();
#if DEBUG
  Serial.println("Servos initialized.");
#endif
  start_ahrs();
#if DEBUG
  Serial.println("AHRS started.");
#endif

  setState(STATE_READY_TO_LAUNCH);
}

void loop() {
  handleState();
}
#endif // SERVO_TESTING