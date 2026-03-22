#ifndef ROCKET_AVIONICS_AHRS_H
#define ROCKET_AVIONICS_AHRS_H

#include "utils.h"

void update_ahrs(const Vec3& gyro, const Vec3& accel, const Vec3& mag);
void start_ahrs();
Quat get_orientation();
Vec3 get_acceleration();
Vec3 get_velocity();
Vec3 get_position();
Vec3 get_angular_velocity();

Rad calculate_roll_rad(const Quat& q);
Rad calculate_pitch_rad(const Quat& q);
Rad calculate_yaw_rad(const Quat& q);
Deg calculate_roll_deg(const Quat& q);
Deg calculate_pitch_deg(const Quat& q);
Deg calculate_yaw_deg(const Quat& q);

Quat roll_deg_to_quat(Deg roll);
Quat yaw_deg_to_quat(Deg yaw);
Quat pitch_deg_to_quat(Deg pitch);

#endif //ROCKET_AVIONICS_AHRS_H