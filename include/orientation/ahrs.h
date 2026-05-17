#ifndef ROCKET_AVIONICS_AHRS_H
#define ROCKET_AVIONICS_AHRS_H

#include "utils.h"

void update_ahrs(const Vec3& gyro, const Vec3& accel, const Vec3& mag, bool in_flight);
void start_ahrs();
Quat get_orientation_earth();
Vec3 get_acceleration_earth();
Vec3 get_velocity_earth();
Vec3 get_position_earth();
Vec3 get_angular_velocity_body();

void zero_pos_vel();

Rad calculate_roll_rad(const Quat& q);
Rad calculate_pitch_rad(const Quat& q);
Rad calculate_yaw_rad(const Quat& q);
Deg calculate_roll_deg(const Quat& q);
Deg calculate_pitch_deg(const Quat& q);
Deg calculate_yaw_deg(const Quat& q);

Quat yaw_rad_to_quat(Rad roll);
Quat pitch_rad_to_quat(Rad yaw);
Quat roll_rad_to_quat(Rad pitch);
Quat pitch_deg_to_quat(Deg pitch);
Quat yaw_deg_to_quat(Deg yaw);
Quat roll_deg_to_quat(Deg roll);

#endif //ROCKET_AVIONICS_AHRS_H