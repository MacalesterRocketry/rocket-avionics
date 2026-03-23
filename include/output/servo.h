#ifndef ROCKET_AVIONICS_SERVO_H
#define ROCKET_AVIONICS_SERVO_H

#include <Servo.h>

enum ServoID {
  SERVO_XPLUS,
  SERVO_XMINUS,
  SERVO_YPLUS,
  SERVO_YMINUS
};

static ServoID servos[] = {SERVO_XPLUS, SERVO_XMINUS, SERVO_YPLUS, SERVO_YMINUS};

void init_servos();
void servo_test_loop();
void set_servo_angle(ServoID servo, double_t angle_degrees_from_neutral);

#endif //ROCKET_AVIONICS_SERVO_H