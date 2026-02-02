#include "gps.h"

#include "Adafruit_GPS.h"
#include "../output/sdcard.h"
#include "../states.h"
#include "pico/stdlib.h"

struct repeating_timer gps_timer;

#define GPSSerial Serial1 // It can't be SoftwareSerial because we're using an interrupt to read data quickly
Adafruit_GPS GPS(&GPSSerial);

bool GPSFix = false;

void setHasGPSFix(const bool hasFix) {
  GPSFix = hasFix;
}

bool hasGPSFix() {
  return GPSFix;
}

// ISR for GPS char reading every 1ms
bool GPS_Timer_Callback(struct repeating_timer *t) {
  // Read all available bytes from the serial port into the library's buffer
  // This moves data from the small hardware UART buffer to the GPS library's memory
  while (GPS.available()) {
    GPS.read();
  }
  return true;
}

void initGPS() {
#if DEBUG
  Serial.println("Starting GPS initialization...");
#endif

  GPS.begin(9600);
#if DEBUG
  Serial.println("Connected to GPS at 9600 baud");
#endif
  wait(200);

  // Enable RMC + GGA - RMC for basic data, GGA for altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); wait(200);
  // Update GPS position at 5Hz (the maximum)
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ); wait(200);
  // Since the GPS has a max of 5Hz, setting NMEA rate higher just causes duplicates and wastes bandwidth
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); wait(200);
  // Disable antenna status messages
  GPS.sendCommand(PGCMD_NOANTENNA);
  wait(1000);

  // read a char from the GPS serial bus every 1ms (1000us) to ensure we don't miss any data
  add_repeating_timer_us(-1000, GPS_Timer_Callback, nullptr, &gps_timer);

#if DEBUG
  Serial.println("GPS initialization complete");
#endif

  logDatetime(GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
}

void readGPS() {
  // Process all available NMEA sentences
  while (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) { // this also sets the newNMEAreceived() flag to false
      setHasGPSFix(false);
      return; // we can fail to parse a sentence in which case we should just wait for another
    }
    printGPSData();
  }
  if (GPS.fix) {
    setHasGPSFix(true);
  } else {
    setHasGPSFix(false);
  }
}

void printGPSData() {
  #if DEBUG and DEBUG_PRINT_SENSORS
  Serial.printf("GPS time: %02d:%02d:%02d\n", GPS.hour, GPS.minute, GPS.seconds);
  Serial.printf("Fix: %d, quality: %d, satellites: %d\n", GPS.fix, GPS.fixquality, GPS.satellites);
  Serial.printf("Location: %.4f %c, %.4f %c\n", GPS.latitude, GPS.lat, GPS.longitude, GPS.lon);
  Serial.printf("Speed (knots): %.2f, angle: %.2f, altitude: %.2f\n", GPS.speed, GPS.angle, GPS.altitude);
  #endif

  logGPS(GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds,
         GPS.latitude_fixed, GPS.longitude_fixed,
         GPS.speed, GPS.angle,
         GPS.altitude,
         GPS.satellites, GPS.fixquality);
}
