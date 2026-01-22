#ifndef ROCKET_AVIONICS_AHRS_H
#define ROCKET_AVIONICS_AHRS_H

// AHRS LOOP
// steps: Single-file AHRS loop: q0 -> bias-corrected sensors -> gyro propagate (q1) ->
// normalise (q2) -> Madgwick correction (q3) -> normalise (q4 final orientation)
// Replace the placeholder sensor reads functions with our IMU reads. DOUBLE CHECK I HAVE THE RIGHT ONES!!

#include "../utils.h"

void updateAHRS(const Vec3& gyroRaw, const Vec3& accelRaw, const Vec3& magRaw);
void initAHRS();
Quat getCurrentOrientation();

//Getting
const Vec3 getGyroCorrected();
const Vec3 getAccelCorrected();
const Vec3 getMagCorrected();

#endif //ROCKET_AVIONICS_AHRS_H