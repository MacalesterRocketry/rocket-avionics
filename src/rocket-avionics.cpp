#include <Arduino.h>

#include "flags.h"

#if SERVO_TESTING
#include "output/servo.h"
void setup() {
  init_servos();
}

void loop() {
  servo_test_loop();
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