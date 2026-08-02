#ifndef GPS_H
#define GPS_H

void initGPS();
void readGPS();
void printGPSData();
void setHasGPSFix(bool hasFix);
bool hasGPSFix();

#endif