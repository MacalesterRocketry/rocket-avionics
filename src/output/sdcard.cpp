#include "sdcard.h"

#include "../states.h"
#include "../utils.h"

SdFat SD;
SdFile dataFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50), &SPI1); // 50MHz is fast, good for RP2040

// Buffer control
constexpr uint32_t SYNC_INTERVAL_MS = 1000; // Sync to SD card every 1 second
uint32_t lastSyncTime = 0;

void initSDCard() {
#if DEBUG
  Serial.println("Initializing SD card...");
#endif

  // Retry mechanism for SD card initialization
  while (!SD.begin(config)) {
    error("Initialization failed! Retrying...", false);
    wait(1000); // Wait for a second before retrying
  }
#if DEBUG
  Serial.println("Initialization successful.");
#endif

  // create a new file name, incrementing if it's already used
  char filename[15];
  strcpy(filename, "log00.bin"); // TODO: Use datetime
  for (uint8_t i = 0; i < 100; i++) {
    filename[3] = i / 10 + '0';
    filename[4] = i % 10 + '0';
    if (!SD.exists(filename)) {
      break;
    }
  }

  if (!dataFile.open(filename, O_RDWR | O_CREAT | O_TRUNC)) {
    error("Failed to open log file on SD card");
    return;
  }

  // Write the Header once at the start
  FileHeader header;
  dataFile.write((uint8_t*)&header, sizeof(header));

#if DEBUG
  Serial.print("Logging to: ");
  Serial.println(filename);
#endif
}

void ejectSDCard() {
  dataFile.close();
  lastSyncTime = millis(); // not necessary, but just in case of further changes that might expect it
  setState(STATE_FILE_CLOSED);
}

bool fileOpen() {
  return dataFile && dataFile.isOpen();
}

void logPacket(const PacketType type, const void* data, const size_t size) {
  if (!fileOpen()) {
    error("Data file closed unexpectedly", false);
    // logEvent(systemState, STATE_FILE_CLOSED, EVENT_OTHER); // TODO: Make this an error flag so it stays in the same state
    // setState(STATE_FILE_CLOSED);
    return;
  }
  // Packet layout: 1B type + 4B micros + 4B millis + N bytes data (depends on type)

  PacketHeader header{};
  header.type = type;
  header.timestamp_millis = millis();
  header.timestamp_micros = micros();

  dataFile.write(&header, sizeof(header));
  dataFile.write(data, size);

  if (millis() - lastSyncTime > SYNC_INTERVAL_MS) {
    sdSync();
  }
}

void sdSync() {
  dataFile.sync(); // TODO: Figure out a way to do this without blocking
  lastSyncTime = millis();
  logEvent(STATE_IRRELEVANT, STATE_IRRELEVANT, EVENT_SD_SYNC);
}

void logIMU(const float ax, const float ay, const float az, const float gx, const float gy, const float gz,
            const float temp) {
  PayloadIMU data = {ax, ay, az, gx, gy, gz, temp};
  logPacket(PACKET_IMU, &data, sizeof(data));
}

void logHighG(const float ax, const float ay, const float az) {
  PayloadHighG data = {ax, ay, az};
  logPacket(PACKET_HIGHG, &data, sizeof(data));
}

void logGPS(const uint8_t hours, const uint8_t minutes, const uint8_t seconds, const uint32_t milliseconds,
            const int32_t latitude, const int32_t longitude,
            const float speed, const float angle,
            const float altitude,
            const uint8_t satellites, const uint8_t fixquality) {
  const uint8_t deciseconds = milliseconds / 100; // convert to deciseconds for storage
  PayloadGPS data = {
    hours,
    minutes,
    seconds,
    deciseconds, // milliseconds are only ever 0, 200, 400, 600, 800 at 5Hz, so we can save space
    latitude, // Decimal degrees * 10,000,000 (standard int notation)
    longitude,
    speed, // knots
    angle, // degrees
    altitude, // meters
    satellites,
    fixquality // 0 = Invalid, 1 = GPS, 2 = DGPS
  };
  logPacket(PACKET_GPS, &data, sizeof(data));
}

void logMagnetometer(const float mx, const float my, const float mz) {
  PayloadMagnetometer data = {mx, my, mz};
  logPacket(PACKET_MAG, &data, sizeof(data));
}

void logBarometer(const float pressure, const float altitude, const float temperature) {
  PayloadBarometer data = {pressure, altitude, temperature};
  logPacket(PACKET_BARO, &data, sizeof(data));
}

void logEvent(const SystemState oldState, const SystemState newState, const EventType reason) {
  PayloadEvent data = {oldState, newState, reason};
  logPacket(PACKET_EVENT, &data, sizeof(data));

  // TODO: Maybe not? AHRS needs all the performance it can get right at ignition. Maybe put it on another core?
  // dataFile.sync();
}

void logStatus(const uint8_t currentState, const float batteryVoltage, const uint8_t sensorsDetected) {
  PayloadStatus data = {currentState, batteryVoltage, sensorsDetected};
  logPacket(PACKET_STATUS, &data, sizeof(data));
}

void logQuaternion(const Quat& orientation) {
  const PayloadQuaternion data = {
    static_cast<float>(orientation.w),
    static_cast<float>(orientation.x),
    static_cast<float>(orientation.y),
    static_cast<float>(orientation.z)
  };
  logPacket(PACKET_QUATERNION, &data, sizeof(data));
}