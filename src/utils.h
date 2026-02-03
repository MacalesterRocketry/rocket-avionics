#ifndef ROCKET_AVIONICS_UTILS_H
#define ROCKET_AVIONICS_UTILS_H

#include <Arduino.h>
#include "flags.h"

// Component Identifications
struct Vec3;
struct Quat;

double clamp(double num, double low, double high);

// 3D Vector
typedef struct Vec3 {
  double x, y, z;

  Vec3 operator*(const double s) const {
    return { x*s, y*s, z*s };
  }
  Vec3& operator*=(const double s) {
    x *= s; y *= s; z *= s;
    return *this;
  }

  Vec3 operator/(const double s) const {
    return { x/s, y/s, z/s };
  }
  Vec3& operator/=(const double s) {
    x /= s; y /= s; z /= s;
    return *this;
  }

  Vec3 operator+(const Vec3 &v) const {
    return { x+v.x, y+v.y, z+v.z };
  }
  Vec3& operator+=(const Vec3 &v) {
    x += v.x; y += v.y; z += v.z;
    return *this;
  }

  Vec3 operator-(const Vec3 &v) const {
    return { x-v.x, y-v.y, z-v.z };
  }
  Vec3& operator-=(const Vec3 &v) {
    x -= v.x; y -= v.y; z -= v.z;
    return *this;
  }

  [[nodiscard]] double dot(const Vec3 &v) const;
  [[nodiscard]] Vec3 cross(const Vec3 &v) const;

  [[nodiscard]] double norm3() const;

  [[nodiscard]] double mag() const;

  [[nodiscard]] Quat toQuat(double w) const;
} Vec3;

// 4D Vector Base (for Quat and Grad4)
template<typename T, typename Derived>
struct Vec4Base {
  T w, x, y, z;
  Vec4Base() = default;
  constexpr Vec4Base(T w_, T x_, T y_, T z_)
    : w(w_), x(x_), y(y_), z(z_) {}

  inline Derived& derived() {
    return static_cast<Derived&>(*this);
  }

  [[nodiscard]] inline const Derived& derived() const {
    return static_cast<const Derived&>(*this);
  }

  // Scalar product
  Derived operator*(const T s) const {
    return Derived{ w*s, x*s, y*s, z*s };
  }
  Derived& operator*=(const T s) {
    w *= s; x *= s; y *= s; z *= s;
    return derived();
  }

  Derived operator/(const T s) const {
    return Derived{ w/s, x/s, y/s, z/s };
  }
  Derived& operator/=(const T s) {
    w /= s; x /= s; y /= s; z /= s;
    return derived();
  }

  Derived operator+(const Vec4Base &q) const {
    return Derived{ w+q.w, x+q.x, y+q.y, z+q.z };
  }
  Derived& operator+=(const Vec4Base &q) {
    w += q.w; x += q.x; y += q.y; z += q.z;
    return derived();
  }

  Derived operator-(const Vec4Base &q) const {
    return Derived{ w-q.w, x-q.x, y-q.y, z-q.z };
  }
  Derived& operator-=(const Vec4Base &q) {
    w -= q.w; x -= q.x; y -= q.y; z -= q.z;
    return derived();
  }

  [[nodiscard]] T norm4() const { return std::sqrt(w*w + x*x + y*y + z*z); }
  [[nodiscard]] T norm() const { return norm4(); }
  [[nodiscard]] T mag() const { return norm4(); }
};

// Quaternion Containers
typedef struct Quat : Vec4Base<double, Quat> {
  using Vec4Base::Vec4Base;
  // Bring scalar operators from Vec4Base into Quat scope
  // so they aren't hidden by the Hamilton products below.
  using Vec4Base::operator*;
  using Vec4Base::operator*=;
  using Vec4Base::operator/;
  using Vec4Base::operator/=;

  // Hamilton product p ⊗ q
  Quat operator*(const Quat& q) const {
    return {
      w*q.w - x*q.x - y*q.y - z*q.z,
      w*q.x + x*q.w + y*q.z - z*q.y,
      w*q.y - x*q.z + y*q.w + z*q.x,
      w*q.z + x*q.y - y*q.x + z*q.w
    };
  }
  Quat& operator*=(const Quat& q) {
    const Quat p = *this;
    w = p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z;
    x = p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y;
    y = p.w*q.y - p.x*q.z + p.y*q.w + p.z*q.x;
    z = p.w*q.z + p.x*q.y - p.y*q.x + p.z*q.w;
    return *this;
  }

  [[nodiscard]] Quat conjugate() const;

  void normalise();

  [[nodiscard]] std::array<std::array<double,3>,3> toMatrix() const;
  Quat() = default;
} Quat;

struct Grad4 : Vec4Base<double, Grad4> {
  using Vec4Base::Vec4Base;
};

struct LSMReading {
  Vec3 accel; // m/s²
  Vec3 gyro; // rad/s
  double temperature; // °C
};

struct LIS3Reading {
  Vec3 mag; // uT
};

struct ADXLReading {
  Vec3 highg_accel; // m/s²
};

struct BMPReading {
  double pressure; // Pascals
  double temperature; // °C
  double altitude; // meters
};

struct SensorReadings {
  LSMReading lsm;
  LIS3Reading lis3;
  ADXLReading adxl;
  BMPReading bmp;
};


// Logging
enum PacketType : uint8_t {
  PACKET_IMU      = 0x10, // IMU (low-G Accelerometer + gyroscope + temperature)
  PACKET_HIGHG    = 0x11, // High-G Accelerometer
  PACKET_MAG      = 0x12, // Magnetometer
  PACKET_BARO     = 0x13, // Barometer (pressure, altitude, temperature)
  PACKET_GPS      = 0x20, // GPS (includes time, position, etc.)
  PACKET_DATETIME = 0x21, // Date and Time from GPS
  PACKET_EVENT    = 0x30, // Discrete Events (Launch, Apogee)
  PACKET_STATUS   = 0x40,  // Battery, etc.
  PACKET_QUATERNION = 0x50 // Quaternion data
};

enum EventType : uint8_t {
  EVENT_LAUNCH_DETECTED = 1,
  EVENT_APOGEE_DETECTED = 2,
  EVENT_BURNOUT_DETECTED = 3,
  EVENT_SD_SYNC = 4,
  EVENT_OTHER = 255
};

enum SystemState : uint8_t {
  STATE_STARTING,
  STATE_READY_TO_LAUNCH,
  STATE_ASCENT,
  STATE_ERROR,
  STATE_WARNING,
  STATE_FILE_CLOSED,
  STATE_IRRELEVANT = 255
};

enum Error : uint16_t { // using a bitmask so we can store multiple co-occurrent errors
  WARNING_SDCARD_CLOSED = 1 << 0,
  WARNING_NO_LSM = 1 << 1,
  WARNING_NO_ADXL = 1 << 2,
  WARNING_NO_LIS3 = 1 << 3,
  WARNING_NO_BMP = 1 << 4,
  ERROR_SDCARD_INIT = 1 << 5,

  WARNING_OTHER = 1 << 14,
  ERROR_OTHER = 1 << 15
};

#endif