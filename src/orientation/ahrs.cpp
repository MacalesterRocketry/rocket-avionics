// AHRS LOOP
// steps: Single-file AHRS loop: q0 -> bias-corrected sensors -> gyro propagate (q1) ->
// normalise (q2) -> Madgwick correction (q3) -> normalise (q4 final orientation)
// Replace the placeholder sensor reads functions with our IMU reads. DOUBLE CHECK I HAVE THE RIGHT ONES!!

#include <Arduino.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

#include <array>
#include <vector>
#include <cmath>

#include "millis64.h"
#include "../utils.h"

// rotate a body-vector v_b into earth frame using quaternion q (body->earth)
Vec3 rotateBodyToEarth(const Quat& q, const Vec3& v_b) {
  // p = q ⊗ [0,v_b] ⊗ q*
  const Quat p{0, v_b.x, v_b.y, v_b.z};
  const Quat t = q * p;
  const Quat res = t * q.conjugate();
  return Vec3{res.x, res.y, res.z}; // last 3 are vector part
}

// small helper: axis-angle -> quaternion exact
Quat axisAngleToQuat(const Vec3& axis, const double angle) {
  const double half = angle * 0.5;
  const double s = std::sin(half);
  return Quat{std::cos(half), axis.x * s, axis.y * s, axis.z * s};
}

// build delta quaternion(propagation) from angular rate omega (rad/s) and dt
Quat deltaQuatFromGyro(const Vec3& omega, const double dt) {
  const double wmag = omega.norm3();
  if (wmag < 1e-12) {
    // tiny rotation -> small-angle approx: q ≈ [1, 0.5*ω*dt]
    return Quat{1.0, 0.5 * omega.x * dt, 0.5 * omega.y * dt, 0.5 * omega.z * dt};
  } else {
    const Vec3 axis = omega / wmag;
    const double theta = wmag * dt;
    return axisAngleToQuat(axis, theta);
  }
}

// Madgwick correction step, I dont know how to get the library from adafruit so I just did it manually/mathmatically!

// returns corrected sensor fused quaternion as quaternion q3 (unnormalised)
Quat madgwickCorrectionStep(const Quat& q_pred, // predicted quaternion (gyro-propagated & normalised) -> q2
                            const Vec3& a, // accelerometer (bias-corrected) in body frame (no normalising required; we can normalise inside)
                            const Vec3& m, // magnetometer (bias-corrected & soft-iron corrected) in body frame
                            const Vec3& gyro,
                            double beta, // algorithm gain (0.0 = no correction, larger faster)
                            double dt) { // time step (s)
// normalise accelerometer measurement
  Vec3 a_n = a;
  const double na = a_n.norm3();
  if (na < 1e-12)
    return q_pred; // can't correct without accel
  a_n /= na;

  // normalise magnetometer measurement
  Vec3 m_n = m;
  const double nm = m_n.norm3();
  if (nm < 1e-12)
    return q_pred;
  m_n /= nm;

  const Quat q = q_pred;

  // Reference direction of Earth's magnetic field (in body frame) --- Madgwick derivation uses this.
  // Compute objective function gradient (following Madgwick 2010 equations, check paper).
  // For full derivation see Madgwick's report; here we implement the standard gradient step.
  // variables
  const double _2q1 = 2.0 * q.w, _2q2 = 2.0 * q.x, _2q3 = 2.0 * q.y, _2q4 = 2.0 * q.z;
  const double _4q1 = 4.0 * q.w, _4q2 = 4.0 * q.x, _4q3 = 4.0 * q.y;
  const double _8q2 = 8.0 * q.x, _8q3 = 8.0 * q.y;
  const double _4q4 = 4.0 * q.z;

  // Gradient of f (accel) part (from Madgwick): need to use specific q# combinations
  // math behind the values= f = predicted_gravity - measured_gravity; predicted_gravity = [2(q2 q4 - q1 q3), 2(q1 q2 + q3 q4), q1^2 - q2^2 - q3^2 + q4^2]
  Vec3 f = {
    2.0 * (q.x * q.z - q.w * q.y) - a_n.x,
    2.0 * (q.w * q.x + q.y * q.z) - a_n.y,
    q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z - a_n.z
  };

  // Jacobian J (3x4) lines combined into gradient g = J^T * f  (this expands to 4 components, as we need it to build a quaternion)
  // Madgwick 2010 paper's compact expression:
  Vec3 g_accel_vec;
  const Grad4 grad = {
    _4q1 * f.z - _2q3 * f.x + _2q2 * f.y,
    _4q2 * f.z + _2q4 * f.x + _2q1 * f.y - _8q2 * f.z,
    _4q3 * f.z - _2q1 * f.x + _2q4 * f.y - _8q3 * f.z,
    _4q4 * f.z + _2q2 * f.x + _2q3 * f.y
  };

  // For magnetometer part we should compute reference direction and its gradient.
  // compute Earth's magnetic field in body frame and gradient.
  // Compute h = q ⊗ m_n ⊗ q*  (magnetic field in earth/body frame)
  const Quat p_m = m_n.toQuat(0);
  const Quat t = q_pred * p_m;
  const Quat hq = t * q_pred.conjugate();
  const Vec3 h{hq.x, hq.y, hq.z};

  // Projection of h onto x-y plane of Earth (reference)
  Vec3 b; // b = [bx, 0, bz] as in Madgwick
  b.x = std::sqrt(h.x * h.x + h.y * h.y);
  b.y = 0.0;
  b.z = h.z;

  // compute the gradient of magnetometer using a simplified combined gradient:
  // Compute an approximate mag error vector between predicted and measured (in body frame!)
  const Vec3 m_pred = h; // already in body frame from above calculation
  const Vec3 mag_err = {
    m_pred.x / (m_pred.norm3() + 1e-12) - m_n.x,
    m_pred.y / (m_pred.norm3() + 1e-12) - m_n.y,
    m_pred.z / (m_pred.norm3() + 1e-12) - m_n.z
  };

  // Build a simple magnetometer gradient approximation using cross products of predicted vs measured
  // This is less exact than full Madgwick derivation but should work well in practice as a correction term, simpler for us
  Vec3 g_mag_vec = {
    2.0 * (q.y * mag_err.z - q.z * mag_err.y + q.w * mag_err.x - q.x * mag_err.z),
    2.0 * (q.z * mag_err.x - q.w * mag_err.z + q.x * mag_err.y - q.y * mag_err.x),
    2.0 * (q.w * mag_err.y - q.x * mag_err.x + q.y * mag_err.z - q.z * mag_err.y)
  };
  // Convert this vector into 4-component approximate gradient (spread across all axes)
  const Grad4 magGrad = {
    0.0,
    g_mag_vec.x,
    g_mag_vec.y,
    g_mag_vec.z
  };

  // Combine gradients (accel-heavy + mag small contribution)
  Grad4 g_combined = magGrad + grad;

  // Normalise gradient
  const double gn = g_combined.norm();
  if (gn > 0.0) {
    g_combined /= gn;
  } else {
    // if mag fails
    g_combined = grad;
    double gn2 = g_combined.norm();
    if (gn2 > 0)
      g_combined /= gn2;
  }

  // Compute quaternion rate from gyroscope (already bias-corrected), math is q_dot = 0.5 * q ⊗ [0,ω]
  const Quat omega_q = gyro.toQuat(0.0);
  Quat q_dot = q_pred * omega_q;
  q_dot *= 0.5;

  // Apply gradient descent corrective step scaled by beta: q_dot = q_dot - beta * gradient
  q_dot.w -= beta * g_combined.w;
  q_dot.x -= beta * g_combined.x;
  q_dot.y -= beta * g_combined.y;
  q_dot.z -= beta * g_combined.z;

  // Integrate to get corrected quaternion (Simple Euler integration)
  Quat q_corr = q_pred + q_dot * dt;

  return q_corr;
}

// Bias computation (mean across samples)
Vec3 computeBiasMean(const std::vector<Vec3>& samples) {
  Vec3 s{0, 0, 0};
  if (samples.empty()) return s;
  for (const auto& v : samples) {
    s += v;
  }
  return s / samples.size();
}

// Main Loop AHRS function Using the above utilities

// AHRS Current State
struct AHRSState {
  Quat q = {1.0, 0.0, 0.0, 0.0}; // Current orientation quaternion
  Vec3 gyroBias = {0.0, 0.0, 0.0};
  Vec3 accelBias = {0.0, 0.0, 0.0};
  Vec3 magBias = {0.0, 0.0, 0.0};
  double beta = 0.1; // Madgwick filter gain
  uint64_t lastUpdate = 0;
  bool isCalibrated = false;
  uint64_t calibrationStart = 0;
  std::vector<Vec3> gyroSamples, accelSamples, magSamples;
};
static AHRSState state;

Vec3 calc_gyro_corrected(const Vec3& gyroRaw) {
  return Vec3{
    gyroRaw.x - state.gyroBias.x,
    gyroRaw.y - state.gyroBias.y,
    gyroRaw.z - state.gyroBias.z
  };
}
Vec3 calc_accel_corrected(const Vec3& accelRaw) {
  return Vec3{
    accelRaw.x - state.accelBias.x,
    accelRaw.y - state.accelBias.y,
    accelRaw.z - state.accelBias.z
  };
}
Vec3 calc_mag_corrected(const Vec3& magRaw) {
  return Vec3{
    magRaw.x - state.magBias.x,
    magRaw.y - state.magBias.y,
    magRaw.z - state.magBias.z
  };
}

// TODO: Improve, add to states.cpp
bool calibrate_ahrs(const Vec3& gyroRaw, const Vec3& accelRaw, const Vec3& magRaw) {
  const uint64_t now = micros64();
  // Auto-calibration phase (first 3 seconds or longer if needed)
  if (!state.isCalibrated) {
    if (state.calibrationStart == 0) {
      state.calibrationStart = now;
      Serial.println("Auto-calibration started - keep IMU stationary...");
    }
    // Collect calibration samples for 3 seconds; we can adjust this duration as needed??
    if (now - state.calibrationStart < 3000000) {
      state.gyroSamples.push_back(gyroRaw);
      state.accelSamples.push_back(accelRaw);
      state.magSamples.push_back(magRaw);

      // Display calibration progress
      static unsigned long lastCalibPrint = 0;
      if (now - lastCalibPrint > 500000) {
        float progress = (now - state.calibrationStart) / 3000000.0 * 100.0;
        Serial.printf("Calibrating... %.0f%%\n", progress);
        lastCalibPrint = now;
      }
      return true;
    } else {
      // Calibration complete from the steps above - now compute biases
      state.gyroBias = computeBiasMean(state.gyroSamples);
      state.accelBias = computeBiasMean(state.accelSamples);
      state.magBias = computeBiasMean(state.magSamples);

      // Adjustment for rocket launch position (Z-up coordinate system/nose up)
      state.accelBias.z += 9.81; // TODO: Figure out if this should be y

      state.isCalibrated = true;
      state.gyroSamples.clear();
      state.accelSamples.clear();
      state.magSamples.clear();

      Serial.println("Calibration complete!");
      Serial.printf("Gyro bias: %.4f, %.4f, %.4f rad/s\n",
                    state.gyroBias.x, state.gyroBias.y, state.gyroBias.z);
      Serial.printf("Accel bias: %.4f, %.4f, %.4f m/s²\n",
                    state.accelBias.x, state.accelBias.y, state.accelBias.z);
    }
  }
  return false;
}

void updateAHRS(const Vec3& gyroRaw, const Vec3& accelRaw, const Vec3& magRaw) {
  // Used to Calculate delta time
  const uint64_t now = micros64();
  double dt = (now - state.lastUpdate) / 1000000.0;
  state.lastUpdate = now;

  if (dt <= 0 || dt > 0.1) {
    dt = 0.01;
  }

  if (calibrate_ahrs(gyroRaw, accelRaw, magRaw)) return;

  //  COMPLETE Q0 → Q4
  // Step 0: Initial quaternion (q0) - previous orientation
  const Quat q0 = state.q;

  // Step 1: Bias correction and sensor adjustment
  const Vec3 gyroCorrected = calc_gyro_corrected(gyroRaw);
  const Vec3 accelCorrected = calc_accel_corrected(accelRaw);
  const Vec3 magCorrected = calc_mag_corrected(magRaw);

  // Step 2: Gyro propagation (q1) - integrate angular rates
  const Quat delta_q = deltaQuatFromGyro(gyroCorrected, dt);
  const Quat q1 = q0 * delta_q;

  // Step 3: Normalise (q2) - maintain quaternion unit length
  Quat q2 = q1;
  q2 = q2.normalized();

  // Step 4: Madgwick sensor fusion correction (q3) - fuse with accelerometer and magnetometer
  const Quat q3 = madgwickCorrectionStep(q2, accelCorrected, magCorrected, gyroCorrected, state.beta, dt);

  // Step 5: Final normalisation (q4) - ensure valid quaternion
  Quat q4 = q3;
  q4 = q4.normalized();

  // Update state with final quaternion, final state
  state.q = q4;

  // EARTH FRAME CONVERSIONS
  // Convert body-frame measurements to earth frame for control systems
  const Vec3 earthAccel = rotateBodyToEarth(q4, accelCorrected);
  const Vec3 earthGyro = rotateBodyToEarth(q4, gyroCorrected);
  const Vec3 earthMag = rotateBodyToEarth(q4, magCorrected);

  // CONTINUOUS ORIENTATION MONITORING/Active Tracking
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 100000) {
    // Convert quaternion to Euler angles
    const double roll = atan2(2.0 * (q4.w * q4.x + q4.y * q4.z),
                              1.0 - 2.0 * (q4.x * q4.x + q4.y * q4.y));
    const double pitch = asin(2.0 * (q4.w * q4.y - q4.z * q4.x));
    const double yaw = atan2(2.0 * (q4.w * q4.z + q4.x * q4.y),
                             1.0 - 2.0 * (q4.y * q4.y + q4.z * q4.z));

    // Display orientation and earth-frame data
    Serial.println("ROCKET ORIENTATION");
    Serial.printf("Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
                  q4.w, q4.x, q4.y, q4.z);
    Serial.printf("Euler: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n",
                  roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
    Serial.printf("Earth Accel: X=%.2f, Y=%.2f, Z=%.2f m/s²\n",
                  earthAccel.x, earthAccel.y, earthAccel.z);
    Serial.printf("Earth Gyro: X=%.2f, Y=%.2f, Z=%.2f rad/s\n",
                  earthGyro.x, earthGyro.y, earthGyro.z);
    Serial.println("==========================");

    lastPrint = now;
  }
}


void initAHRS() {
  state.lastUpdate = micros64();
  state.gyroSamples.clear();
  state.accelSamples.clear();
  state.magSamples.clear();
}

Quat getCurrentOrientation() {
  return state.q;
}
