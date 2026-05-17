#include "states.h"
#include "config.h"

#include "orientation/ahrs.h"
#include "orientation/sensors.h"
#if USE_GPS
#include "orientation/gps.h"
#endif
#include "output/roll-controller.h"
#include "output/sdcard.h"

#include "Adafruit_NeoPixel.h"

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, NEOPIXEL_PIN);

uint32_t lastTimeBuzzerChanged = 0;
bool buzzerOn = false;

SystemState systemState;

uint64_t ignitionTime = 0;

void initIndicators() {
  pixel.begin();
  pixel.setBrightness(30);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWriteFast(BUZZER_PIN, LOW);
#if USE_TURN_SIGNALS
  pinMode(TURN_SIGNAL_LEFT_PIN, OUTPUT);
  pinMode(TURN_SIGNAL_RIGHT_PIN, OUTPUT);
#endif
}

void wait(const int milliseconds) {
  const unsigned long startTime = millis();
  while (millis() - startTime < milliseconds) {
#if USE_GPS
    readGPS();
#endif
    handleState();
  }
}

SensorReadings getSensorData() {
#if USE_GPS
  readGPS();
#endif
  return readSensors();
}

void indicateState(SystemState state) {
  switch (state) {
    case STATE_READY_TO_LAUNCH: {
#if USE_GPS
      if (hasGPSFix()) {
        pixel.setPixelColor(0, pixel.Color(0, 255, 0));
        runBuzzer(0.2, 16);
      } else {
        pixel.setPixelColor(0, pixel.Color(255, 255, 0));
        runBuzzer(0.2, 8);
      }
#else
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      runBuzzer(0.2, 16);
#endif
      break;
    }
    case STATE_ASCENT: {
      // purple and long beeps
      pixel.setPixelColor(0, pixel.Color(255, 0, 255));
#if USE_GPS
      if (hasGPSFix()) {
        runBuzzer(1, 16);
      } else {
        runBuzzer(1, 8);
      }
#else
      runBuzzer(1, 16);
#endif
      break;
    }
    case STATE_FILE_CLOSED: {
      pixel.setPixelColor(0, pixel.Color(0, 255, 255));
      runBuzzer(0.2, 4);
      break;
    }
    case STATE_STARTING: {
      pixel.setPixelColor(0, pixel.Color(0, 0, 255));
      runBuzzer(0.2, 2);
      break;
    }
    case STATE_WARNING: {
      pixel.setPixelColor(0, pixel.Color(255, 120, 0));
      runBuzzer(0.1, 0.5);
      break;
    }
    case STATE_ERROR: {
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
      runBuzzer(0.1, 0.1);
      break;
    }
    case STATE_IRRELEVANT:
      break;
  }
  pixel.show();
}

Deg rollProgram() {
  const double timeInFlight = (micros64() - ignitionTime) / 1000000.0;
  // at 2 seconds, roll 90°, then at 4 seconds roll to -90°, then at 6 seconds roll back to 0°
  if (timeInFlight >= 0 && timeInFlight < 3) {
    return 0.0;
  }
  if (timeInFlight >= 3 && timeInFlight < 6) {
    return 90.0;
  }
  return 0.0;
}

void handleState() { // operations and transition functions
  indicateState(systemState);

  // global operations and transition functions
  // TODO: Also close file on full SD card and low battery
  if (digitalRead(EJECT_BUTTON) == LOW && fileOpen()) {
    ejectSDCard();
    return setState(STATE_FILE_CLOSED);
  }

  switch (systemState) {
    case STATE_STARTING: {
      break;
    }
    case STATE_READY_TO_LAUNCH: {
      // hasLaunched needs to be called before logData
      // Both functions clear any high-G interrupts, but hasLaunched needs to read them first
      // Only true when using interrupts, which we're not right now, but I'm leaving it.
      if (hasLaunched()) {
        ignitionTime = micros64();
        // TODO: I didn't get a log for this. Troubleshoot.
        logEvent(systemState, STATE_ASCENT, EVENT_LAUNCH_DETECTED);
#if DEBUG
        Serial.println("Launch detected!");
#endif
        return setState(STATE_ASCENT);
      }

      const SensorReadings sensorData = getSensorData();
      update_ahrs(sensorData.lsm.gyro, sensorData.lsm.accel, sensorData.lis3.mag, false);
      logAHRS(get_orientation_earth(), get_acceleration_earth(), get_velocity_earth(), get_position_earth());
      zero_pos_vel(); // On the ground, it's expected to be stationary, so we can zero our position and velocity estimates to correct for any drift during pre-launch.
      // TODO: Is that actually a good idea?
      break;
    }
    case STATE_ASCENT: {
      const SensorReadings sensorData = getSensorData();
      Vec3 accel = sensorData.lsm.accel;
      if (accel.mag() >= ACCELEROMETER_SWITCH_THRESHOLD) { // If the low-G accelerometer is saturated, switch to high-G readings for AHRS
        accel = sensorData.adxl.highg_accel;
      }
      update_ahrs(sensorData.lsm.gyro, accel, sensorData.lis3.mag, true);
      logAHRS(get_orientation_earth(), get_acceleration_earth(), get_velocity_earth(), get_position_earth());

      // Actuate roll control surfaces based on current orientation and target angle
      static const Quat base_orientation = get_orientation_earth(); // Set the base orientation at launch
      update_roll(rollProgram(), base_orientation);

      // TODO: Transition function should probably be some threshold for chute deploy
      //  bar+gyro+acc all crazy within 0.1s of each other?
      break;
    }
    case STATE_FILE_CLOSED: {
      break;
    }
    case STATE_WARNING: { // TODO: Warning should be a flag in Ready to Launch, not a state
      break;
    }
    case STATE_ERROR: {
      break;
    }
    case STATE_IRRELEVANT:
      break;
  }
  pixel.show();
}

void setState(const SystemState state) {
#if DEBUG
  Serial.print("State change: ");
  Serial.print(systemState);
  Serial.print(" -> ");
  Serial.println(state);
#endif
  systemState = state;
  handleState();
  // If called in handleState, this could theoretically cause a bug where it detects a state change,
  // handles it, then switches back to finish off the previous state.
  // To avoid this, use `return setState(state)` when changing states within handleState.
}

/*
 * Function to run the buzzer for a specified duration, every specified
 * number of seconds.
 * @param duration Duration in seconds
 * @param secondsBetween Seconds between each beep (between the last beep ended and this one starts)
 */
void runBuzzer(float secondsDuration, float secondsBetween) {
  if (!buzzerOn && millis() - lastTimeBuzzerChanged > secondsBetween * 1000) {
    lastTimeBuzzerChanged = millis();
    digitalWriteFast(BUZZER_PIN, HIGH);
    buzzerOn = true;
  } else if (buzzerOn && millis() - lastTimeBuzzerChanged > secondsDuration * 1000) {
    lastTimeBuzzerChanged = millis();
    digitalWriteFast(BUZZER_PIN, LOW);
    buzzerOn = false;
  }
}

void error(const String& message, const bool fatal) {
#if DEBUG
  Serial.println(message);
#endif
  if (fatal) {
    ejectSDCard();
    setState(STATE_ERROR);
    while (true) {
      yield();
    }
  }
  setState(STATE_WARNING);
}
