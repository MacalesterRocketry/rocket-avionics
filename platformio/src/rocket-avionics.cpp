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

#if PROFILING
uint64_t durationList[PROFILING_SAMPLES];
uint16_t durationIndex = 0;
uint64_t previous_micros = 0;
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

#if PROFILING
  if (previous_micros != 0) {
    durationList[durationIndex] = micros64() - previous_micros;
    durationIndex = (durationIndex + 1) % PROFILING_SAMPLES;
  }

  if (durationIndex == 0) { // Print average duration every PROFILING_SAMPLES iterations
    uint64_t totalDuration = 0;
    uint64_t maxDuration = 0;
    uint64_t minDuration = UINT64_MAX;
    for (const uint16_t duration : durationList) {
      totalDuration += duration;
      if (duration > maxDuration) {
        maxDuration = duration;
      }
      if (duration < minDuration) {
        minDuration = duration;
      }
    }
    const uint64_t averageDuration = totalDuration / PROFILING_SAMPLES;
    Serial.print("Average loop duration over last ");
    Serial.print(PROFILING_SAMPLES);
    Serial.print(" samples: ");
    Serial.print(averageDuration);
    Serial.println(" microseconds");
    Serial.print("Max loop duration: ");
    Serial.print(maxDuration);
    Serial.println(" microseconds");
    Serial.print("Min loop duration: ");
    Serial.print(minDuration);
    Serial.println(" microseconds");
    Serial.println("==========================");
  }
  previous_micros = micros64();
#endif
}
#endif