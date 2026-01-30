#ifndef ROCKET_AVIONICS_ROLL_CONTROLLER_H
#define ROCKET_AVIONICS_ROLL_CONTROLLER_H

#include "../orientation/sensors.h"

// Constants
#define MAX_SERVO_ANGLE 100 // max rotation of servo
#define MIN_SERVO_ANGLE 0 // min rotation of servo
#define Kp 0.4 // proportional constant
#define Ki 1 // integral constant
#define Kd 2 // derivative constant

// double calculate_servo_angle(double airspeed, double current_rocket_angle, double target_rocket_angle);

#endif