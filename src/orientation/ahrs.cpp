#include "ahrs.h"

#include <vector>
#include <cmath>

#include "millis64.h"
#include "utils.h"

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
  const double s = sin(half);
  return Quat{cos(half), axis.x * s, axis.y * s, axis.z * s};
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
  double beta = 0.1; // Madgwick filter gain
  uint64_t lastUpdate = 0;
  Vec3 acceleration = {0, 0, 0};
  Vec3 velocity = {0, 0, 0};
  Vec3 position = {0, 0, 0};
  Vec3 angular_velocity = {0, 0, 0};
};
static AHRSState state;

void update_ahrs(const Vec3& gyro, const Vec3& accel, const Vec3& mag) {
  // Used to Calculate delta time
  const uint64_t now_micros = micros64();
  const double dt = (now_micros - state.lastUpdate) / 1000000.0;
  state.lastUpdate = now_micros;

  if (dt <= 0) {
    return; // Invalid time step, skip update
  }

  //  COMPLETE Q0 → Q4
  // Step 0: Initial quaternion (q0) - previous orientation
  const Quat q0 = state.q;

  // Step 2: Gyro propagation (q1) - integrate angular rates
  const Quat delta_q = deltaQuatFromGyro(gyro, dt);
  const Quat q1 = q0 * delta_q;

  // Step 3: Normalise (q2) - maintain quaternion unit length
  Quat q2 = q1;
  q2 = q2.normalized();

  // Step 4: Madgwick sensor fusion correction (q3) - fuse with accelerometer and magnetometer
  const Quat q3 = madgwickCorrectionStep(q2, accel, mag, gyro, state.beta, dt);

  // Step 5: Final normalisation (q4) - ensure valid quaternion
  Quat q4 = q3;
  q4 = q4.normalized();

  // Update state with final quaternion, final state
  state.q = q4;

  // EARTH FRAME CONVERSIONS
  // Convert body-frame measurements to earth frame for control systems
  const Vec3 earthAccel = rotateBodyToEarth(q4, accel);
  const Vec3 earthGyro = rotateBodyToEarth(q4, gyro);
  const Vec3 earthMag = rotateBodyToEarth(q4, mag);

  state.acceleration = earthAccel;
  state.velocity += earthAccel * dt;
  state.position += state.velocity * dt;

  state.angular_velocity = gyro; // still in body frame, but we can use it for control

  // CONTINUOUS ORIENTATION MONITORING/Active Tracking
#if DEBUG and DEBUG_PRINT_ORIENTATION
  static unsigned long lastPrint = 0;
  if (now_micros - lastPrint > 100000) {
    // Convert quaternion to Euler angles
    const double roll = calculate_roll_deg(state.q);
    const double pitch = calculate_pitch_deg(state.q);
    const double yaw = calculate_yaw_deg(state.q);

    // Display orientation and earth-frame data
    Serial.println("ROCKET ORIENTATION");
    Serial.printf("Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
                  state.q.w, state.q.x, state.q.y, state.q.z);
    Serial.printf("Euler: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n",
                  roll, pitch, yaw);
    Serial.printf("Earth Accel: X=%.2f, Y=%.2f, Z=%.2f m/s²\n",
                  state.acceleration.x, state.acceleration.y, state.acceleration.z);
    Serial.printf("Earth Velocity: X=%.2f, Y=%.2f, Z=%.2f m/s\n",)
                  state.velocity.x, state.velocity.y, state.velocity.z);
    Serial.printf("Earth Position: X=%.2f, Y=%.2f, Z=%.2f m\n",)
                  state.position.x, state.position.y, state.position.z);
    Serial.println("==========================");

    lastPrint = now_micros;
  }
#endif
}
void start_ahrs() {
  state.lastUpdate = micros64();
}
Quat get_orientation() {
  return state.q;
}

Vec3 get_acceleration() {
  return state.acceleration;
}
Vec3 get_velocity() {
  return state.velocity;
}
Vec3 get_position() {
  return state.position;
}
Vec3 get_angular_velocity() {
  return state.angular_velocity;
}

Rad calculate_roll_rad(const Quat& q) {
  return atan2(2.0 * (q.w * q.x + q.y * q.z),
               1.0 - 2.0 * (q.x * q.x + q.y * q.y));
}

Rad calculate_pitch_rad(const Quat& q) {
  return asin(2.0 * (q.w * q.y - q.z * q.x));
}

Rad calculate_yaw_rad(const Quat& q) {
  return atan2(2.0 * (q.w * q.z + q.x * q.y),
               1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Deg calculate_roll_deg(const Quat& q) {
  return radToDeg(calculate_roll_rad(q));
}

Deg calculate_pitch_deg(const Quat& q) {
  return radToDeg(calculate_pitch_rad(q));
}

Deg calculate_yaw_deg(const Quat& q) {
  return radToDeg(calculate_yaw_rad(q));
}

Quat roll_deg_to_quat(const Deg roll) {
  const double roll_rad = degToRad(roll);
  return axisAngleToQuat({1, 0, 0}, roll_rad);
}

Quat yaw_deg_to_quat(const Deg yaw) {
  const double yaw_rad = degToRad(yaw);
  return axisAngleToQuat({0, 0, 1}, yaw_rad);
}

Quat pitch_deg_to_quat(const Deg pitch) {
  const double pitch_rad = degToRad(pitch);
  return axisAngleToQuat({0, 1, 0}, pitch_rad);
}