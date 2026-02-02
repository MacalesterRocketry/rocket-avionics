#include "states.h"

#include "Adafruit_NeoPixel.h"
#include "orientation/ahrs.h"
#include "orientation/sensors.h"

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, NEOPIXEL_PIN);

unsigned long lastTimeBuzzerChanged = 0;
bool buzzerOn = false;

SystemState systemState;

void initIndicators() {
  pixel.begin();
  pixel.setBrightness(30);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
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

SensorReadings getSensorData() { // TODO: return a struct from this, which can then be used for updateAHRS
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
  }
  pixel.show();
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
    case STATE_READY_TO_LAUNCH: {
      // hasLaunched needs to be called before logData
      // Both functions clear any high-G interrupts, but hasLaunched needs to read them first
      // Only true when using interrupts, which we're not right now, but I'm leaving it.
      if (hasLaunched()) {
        logEvent(systemState, STATE_ASCENT, EVENT_LAUNCH_DETECTED);
        start_ahrs();
#if DEBUG
        Serial.println("Launch detected!");
#endif
        return setState(STATE_ASCENT);
      }

      if (fileOpen()) {
        getSensorData();
      } else {
        error("Data file closed unexpectedly", false);
        logEvent(systemState, STATE_FILE_CLOSED, EVENT_OTHER);
        return setState(STATE_FILE_CLOSED); // TODO: log events here?
      }
      break;
    }
    case STATE_ASCENT: {
      const SensorReadings sensorData = getSensorData();
      update_ahrs(sensorData.lsm.gyro, sensorData.adxl.highg_accel, sensorData.lis3.mag);
      Quat orientation = get_current_orientation();
      logQuaternion(orientation);
      // TODO: Use orientation for PID stuff
      // TODO: Log orientation to SD card

      // TODO: Transition function should probably be some threshold for chute deploy
      // bar+gyro+acc all crazy within 0.1s of each other?
      // if (fileOpen()) {
      //   getSensorData();
      // } else {
      //   error("Data file closed unexpectedly", false);
      //   logEvent(systemState, STATE_FILE_CLOSED, EVENT_OTHER); // TODO: Make this an error flag so it stays in the same state
      //   return setState(STATE_FILE_CLOSED);
      // }
      break;
    }
    case STATE_FILE_CLOSED: {
      break;
    }
    case STATE_STARTING: {
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

void setState(SystemState state) {
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
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOn = true;
  } else if (buzzerOn && millis() - lastTimeBuzzerChanged > secondsDuration * 1000) {
    lastTimeBuzzerChanged = millis();
    digitalWrite(BUZZER_PIN, LOW);
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
