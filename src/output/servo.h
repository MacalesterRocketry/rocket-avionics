#ifndef ROCKET_AVIONICS_SERVO_H
#define ROCKET_AVIONICS_SERVO_H

#include <Servo.h>

static Servo servoXPlus;
static Servo servoXMinus;
static Servo servoYPlus;
static Servo servoYMinus;

void init_servos();
void servo_test_loop();
void set_servo_angle(Servo& servo, double_t angle_degrees);

#endif //ROCKET_AVIONICS_SERVO_H