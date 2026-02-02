#ifndef ROCKET_AVIONICS_AHRS_H
#define ROCKET_AVIONICS_AHRS_H

// AHRS LOOP
// steps: Single-file AHRS loop: q0 -> bias-corrected sensors -> gyro propagate (q1) ->
// normalise (q2) -> Madgwick correction (q3) -> normalise (q4 final orientation)
// Replace the placeholder sensor reads functions with our IMU reads. DOUBLE CHECK I HAVE THE RIGHT ONES!!

#include "../utils.h"

void update_ahrs(const Vec3& gyro, const Vec3& accel, const Vec3& mag);
void start_ahrs();
Quat get_current_orientation();

#endif //ROCKET_AVIONICS_AHRS_H