#pragma once

// TODO: On some boots, the loop is extremely slow (like 5 seconds), resulting in failures of launch detection and servo movement. Troubleshoot this.

#define DEBUG 0
#define DEBUG_PRINT_SENSORS 0
#define DEBUG_PRINT_ORIENTATION 0
#define DEBUG_PRINT_ROLL_CONTROL 0

#define PID_TUNING 0
#define SERVO_TESTING 0

#define PROFILING 0
#define PROFILING_SAMPLES 100

#define SYNC_INTERVAL_MS 5000

#define USE_GPS 1
#define USE_TURN_SIGNALS 1

// Calibration
#define G 9.80579 // North Branch gravity in m/s²

#define MAG_BIAS_X (-35.63)
#define MAG_BIAS_Y (19.96)
#define MAG_BIAS_Z (-33.43)

#define HIGHG_BIAS_X (9.8)
#define HIGHG_BIAS_Y (7.85)
#define HIGHG_BIAS_Z (9.8 - G)
#define HIGHG_TRIM_X (-5) // In counts, not m/s²; TODO: This is a better way to calibrate, so use it
#define HIGHG_TRIM_Y (-4)
#define HIGHG_TRIM_Z (0)

#define LOWG_BIAS_X (0.05)
#define LOWG_BIAS_Y (0.03)
#define LOWG_BIAS_Z (10.03 - G)

#define GYRO_BIAS_X (-0.01)
#define GYRO_BIAS_Y (0.0)
#define GYRO_BIAS_Z (0.0)
// #define GYRO_BIAS_X (0.029822) // TODO: test
// #define GYRO_BIAS_Y (-0.003555)
// #define GYRO_BIAS_Z (0.028698)

// AHRS Constants
// Beta constants indicate how much to trust the different sensor corrections.
// Higher beta means more trust in that sensor and faster convergence, but also more susceptibility to noise.
#define AHRS_ACC_BETA (0.1) // accelerometer beta constant for Madgwick filter
#define AHRS_MAG_BETA (0.0) // magnetometer beta constant for Madgwick filter

// PID constants
#define MOMENT_OF_INERTIA 0.00685 // MoI in kg m²
#define ROLL_PID_Kp 0.4 // proportional constant
#define ROLL_PID_Ki 1 // integral constant
#define ROLL_PID_Kd 2 // derivative constant

// Thresholds
#define LAUNCH_ACCEL_THRESHOLD_G 4.0
#define SERVO_DEGREE_RANGE 100.0 // degrees of the physical servo. For example, our smaller servo does 100°
#define SERVO_MAX_ANGLE 50.0 // degrees, maximum angle the servo can move to when in the fin
#define SERVO_MIN_ANGLE (-50.0) // degrees, minimum angle the servo can move to when in the fin
#define SERVO_NEUTRAL_ANGLE (-7.0) // degrees, the angle at which the servo is neutral (pointing straight back)
#define SERVO_MICROS_MIN 1000 // microseconds
#define SERVO_MICROS_MAX 2000 // microseconds
#define SERVO_MAX_TORQUE 0.51 // Nm, according to datasheet (5.2 Kgf cm at 7.4V, which is 0.51 Nm)

#define ACCELEROMETER_SWITCH_THRESHOLD (15.9 * G) // Low-G accelerometer saturates at 16G

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
#define BATTERY_VOLTAGE_R1 100000.0 // 100kOhm voltage divider resistors for the battery voltage measurements
#define BATTERY_VOLTAGE_R2 100000.0

#define TURN_SIGNAL_LEFT_PIN A0
#define TURN_SIGNAL_RIGHT_PIN A1

#elif HARDWARE_VERSION == 1
#define SD_CS_PIN 23
#define BUZZER_PIN A2
#define EJECT_BUTTON PIN_BUTTON
#define NEOPIXEL_PIN PIN_NEOPIXEL
#endif // HARDWARE_VERSION