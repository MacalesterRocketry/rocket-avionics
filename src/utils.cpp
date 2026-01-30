#include "utils.h"

#include <array>

// 3D Vector
[[nodiscard]] double Vec3::dot(const Vec3 &v) const {
  return x*v.x + y*v.y + z*v.z;
}
[[nodiscard]] Vec3 Vec3::cross(const Vec3 &v) const {
  return { y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x };
}

[[nodiscard]] double Vec3::norm3() const {
  return std::sqrt(x*x + y*y + z*z);
}

[[nodiscard]] double Vec3::mag() const { return norm3(); }

[[nodiscard]] Quat Vec3::toQuat(double w) const {
  return Quat{w, x, y, z};
}

// Quaternion Containers
[[nodiscard]] Quat Quat::conjugate() const {
  return {w,-x,-y,-z};
}

Quat Quat::normalized() const {
  const double n = norm();
  if(n <= 1e-15) { return {1,0,0,0}; }
  return {w/n, x/n, y/n, z/n};
}

// convert quaternion using rotation matrix (body -> earth if q maps body->earth)
[[nodiscard]] std::array<std::array<double,3>,3> Quat::toMatrix() const {
  std::array<std::array<double,3>,3> R{};
  R[0][0] = 1.0 - 2.0*(y*y + z*z);
  R[0][1] = 2.0*(x*y - w*z);
  R[0][2] = 2.0*(x*z + w*y);
  R[1][0] = 2.0*(x*y + w*z);
  R[1][1] = 1.0 - 2.0*(x*x + z*z);
  R[1][2] = 2.0*(y*z - w*x);
  R[2][0] = 2.0*(x*z - w*y);
  R[2][1] = 2.0*(y*z + w*x);
  R[2][2] = 1.0 - 2.0*(x*x + y*y);
  return R;
}


// Limit values for gain stability(sensor values and noise handling)
static double clamp(const double num, const double low, const double high) {
  return num < low ? low : (num > high ? high : num);
}
