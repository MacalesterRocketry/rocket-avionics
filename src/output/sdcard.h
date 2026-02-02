#ifndef ROCKET_AVIONICS_SD_CARD_H
#define ROCKET_AVIONICS_SD_CARD_H

#include <Arduino.h>
#include <SdFat.h>

#include "../flags.h"
#include "../utils.h"

void initSDCard();
void ejectSDCard();
bool fileOpen();
void sdSync();
void logPacket(PacketType type, const void* data, size_t size);
void logIMU(float ax, float ay, float az, float gx, float gy, float gz,
            float temp);
void logHighG(float ax, float ay, float az);
void logGPS(uint8_t hours, uint8_t minutes, uint8_t seconds, uint32_t milliseconds,
            int32_t latitude, int32_t longitude,
            float speed, float angle,
            float altitude,
            uint8_t satellites, uint8_t fixquality);
void logMagnetometer(float mx, float my, float mz);
void logBarometer(float pressure, float altitude, float temperature);
void logEvent(SystemState oldState, SystemState newState, EventType reason);
void logStatus(uint8_t currentState, float batteryVoltage, uint8_t sensorsDetected);
void logQuaternion(const Quat& orientation);
void logDatetime(uint16_t year, uint8_t month, uint8_t day,
                 uint8_t hours, uint8_t minutes, uint8_t seconds);

#pragma pack(push, 1)
struct FileHeader {
  uint8_t version = 2;
  uint8_t endian = 0; // little-endian
};

struct PacketHeader {
  uint8_t type;
  uint32_t timestamp_micros;
  uint32_t timestamp_millis; // micros wraps around after just over an hour, so we're using millis for the actual time and micros for deltas
};

// IMU: Accelerometer + Gyroscope + Temperature
struct PayloadIMU {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float temperature;
};

struct PayloadHighG {
  float accX, accY, accZ;
};

// GPS
struct PayloadGPS {
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint8_t deciseconds; // milliseconds are only ever 0, 200, 400, 600, 800 at 5Hz, so we can save space
  int32_t latitude;  // Decimal degrees * 10,000,000 (standard int notation)
  int32_t longitude;
  float speed; // knots
  float angle; // degrees
  float altitude;  // meters
  uint8_t satellites;
  uint8_t fixquality; // 0 = Invalid, 1 = GPS, 2 = DGPS
};

// Magnetometer
struct PayloadMagnetometer {
  float magX, magY, magZ;
};

// Barometer
struct PayloadBarometer {
  float pressure;    // Pascals
  float temperature; // Celsius
  float altitude;    // Meters
};

// Event: For Launch Detection, Apogee, etc.
struct PayloadEvent {
  SystemState oldState;
  SystemState newState;
  EventType reasonCode;
};

struct PayloadStatus {
  uint8_t rocketState;
  float batteryVoltage; // Volts
  uint8_t sensorsDetected; // Bitfield of detected sensors
};

struct PayloadQuaternion {
  float w; // Our code uses doubles, but we don't need that precision for logs
  float x;
  float y;
  float z;
};

struct PayloadDatetime {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
};

#pragma pack(pop)

#endif