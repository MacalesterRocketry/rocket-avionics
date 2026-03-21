#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_ADXL375.h>
#include <BMP3XX.h>

#include "config.h"
#if USE_GPS
#include "gps.h"
#endif
#include "sdcard.h"
#include "states.h"

void initSensors();

void initLowGAccelerometer();
void initMagnetometer();
void initHighGAccelerometer();
void initBarometer();

LSMReading readLSM();
LIS3Reading readLIS3();
ADXLReading readADXL();
BMPReading readBMP();
SensorReadings readSensors();

bool hasLaunched();
#endif