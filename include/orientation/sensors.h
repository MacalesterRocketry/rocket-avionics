#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"
#if USE_GPS
#endif
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