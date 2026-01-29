#pragma once

#define DEBUG 0
#define DEBUG_PRINT_SENSORS 0

#define PID_TUNING 0
#define SYNC_INTERVAL_MS 5000
#define SERVO_TESTING 0

#define USE_GPS 1

// Thresholds
#define LAUNCH_ACCEL_THRESHOLD_G 4.0
#define SERVO_DEGREE_RANGE 100.0 // degrees of the physical servo. For example, our smaller servo does 100°, and our larger one does 180°.
#define SERVO_MICROS_MIN 1000 // microseconds
#define SERVO_MICROS_MAX 2000 // microseconds

// Pin definitions
#define HARDWARE_VERSION 2

#if HARDWARE_VERSION == 2
#define SD_CS_PIN 23
#define BUZZER_PIN A2
#define NEOPIXEL_PIN 4
#define EJECT_BUTTON 24

#define ADXL_INT1_PIN 5
#define ADXL_INT2_PIN 6
#define LSM_INT1_PIN 9
#define LSM_INT2_PIN 10
#define LIS3_INT1_PIN 11
#define LIS3_INT2_PIN 12
#define BMP_INT_PIN 13

#define SERVO_XPLUS_PIN 25
#define SERVO_XMINUS_PIN 14
#define SERVO_YPLUS_PIN 15
#define SERVO_YMINUS_PIN 8

#define BATTERY_VOLTAGE_PIN A3

#elif HARDWARE_VERSION == 1
#define SD_CS_PIN 23
#define BUZZER_PIN A2
#define EJECT_BUTTON PIN_BUTTON
#define NEOPIXEL_PIN PIN_NEOPIXEL
#endif // HARDWARE_VERSION