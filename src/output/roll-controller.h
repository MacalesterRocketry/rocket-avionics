#ifndef ROCKET_AVIONICS_ROLL_CONTROLLER_H
#define ROCKET_AVIONICS_ROLL_CONTROLLER_H

#include "../orientation/sensors.h"

void update_roll(Deg target_angle, const Quat& base_orientation);

#endif