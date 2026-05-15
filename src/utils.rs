use defmt::Format;
use nalgebra::{Quaternion, Vector3, UnitQuaternion};

pub type Vec3 = Vector3<f64>;
pub type Quat = Quaternion<f64>;
pub type UnitQuat = UnitQuaternion<f64>;

pub struct SensorReadings {
    pub lsm: LSMReading,
    pub lis3: LIS3Reading,
    pub adxl: ADXLReading,
    pub bmp: BMPReading,
}

pub struct LSMReading {
    pub accel: Vec3, // m/s²
    pub gyro: Vec3,  // rad/s
    pub temperature: f64, // °C
}

pub struct LIS3Reading {
    pub mag: Vec3, // uT
}

pub struct ADXLReading {
    pub highg_accel: Vec3, // m/s²
}

pub struct BMPReading {
    pub pressure: f64,    // Pascals
    pub temperature: f64, // °C
    pub altitude: f64,    // meters
}

#[derive(Copy, Clone, PartialEq, Format)]
pub enum SystemState {
    Starting,
    ReadyToLaunch,
    Ascent,
    Error,
    Warning,
    FileClosed,
}

pub fn clamp(num: f64, low: f64, high: f64) -> f64 {
    if num < low {
        low
    } else if num > high {
        high
    } else {
        num
    }
}

pub fn rad_to_deg(radians: f64) -> f64 {
    radians * 180.0 / core::f64::consts::PI
}

pub fn deg_to_rad(degrees: f64) -> f64 {
    degrees * core::f64::consts::PI / 180.0
}
