#ifndef GPS_H
#define GPS_H

#include "config.h"

void initGPS();
void readGPS();
void printGPSData();
void setHasGPSFix(bool hasFix);
bool hasGPSFix();

#endif