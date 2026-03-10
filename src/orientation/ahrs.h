#ifndef ROCKET_AVIONICS_AHRS_H
#define ROCKET_AVIONICS_AHRS_H

// AHRS LOOP
// steps: Single-file AHRS loop: q0 -> bias-corrected sensors -> gyro propagate (q1) ->
// normalise (q2) -> Madgwick correction (q3) -> normalise (q4 final orientation)
// Replace the placeholder sensor reads functions with our IMU reads. DOUBLE CHECK I HAVE THE RIGHT ONES!!

#include "../utils.h"

void update_ahrs(const Vec3& gyro, const Vec3& accel, const Vec3& mag);
void start_ahrs();
Quat get_orientation();
Vec3 get_acceleration();
Vec3 get_velocity();
Vec3 get_position();

Rad calculate_roll_rad(const Quat& q);
Rad calculate_pitch_rad(const Quat& q);
Rad calculate_yaw_rad(const Quat& q);
Deg calculate_roll_deg(const Quat& q);
Deg calculate_pitch_deg(const Quat& q);
Deg calculate_yaw_deg(const Quat& q);

#endif //ROCKET_AVIONICS_AHRS_H