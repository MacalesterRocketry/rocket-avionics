//! Shared sensor + state types. Direct port of the struct/enum declarations
//! at the bottom of the C++ `utils.h`.

use crate::math::Vec3;
use bitflags::bitflags;

#[derive(Default, Debug, Clone, Copy)]
pub struct LsmReading {
    /// Body-frame linear acceleration (m/s²), bias-subtracted.
    pub accel: Vec3,
    /// Body-frame angular rate (rad/s), bias-subtracted.
    pub gyro: Vec3,
    /// Die temperature (°C).
    pub temperature: f64,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct Lis3Reading {
    /// Body-frame magnetic field (µT), hard-iron corrected.
    pub mag: Vec3,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct AdxlReading {
    /// Body-frame high-G acceleration (m/s²), bias-subtracted.
    pub highg_accel: Vec3,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct BmpReading {
    pub pressure: f64,    // Pa
    pub temperature: f64, // °C
    pub altitude: f64,    // m
}

#[derive(Default, Debug, Clone, Copy)]
pub struct SensorReadings {
    pub lsm: LsmReading,
    pub lis3: Lis3Reading,
    pub adxl: AdxlReading,
    pub bmp: BmpReading,
}

// ────────────────────────── state machine / events ──────────────────────────
/// Discriminants MUST match the C++ enum: decoder.py reads these raw bytes.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemState {
    Starting = 0,
    ReadyToLaunch = 1,
    Ascent = 2,
    Error = 3,
    Warning = 4,
    FileClosed = 5,
    Irrelevant = 255,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventType {
    LaunchDetected = 1,
    ApogeeDetected = 2,
    BurnoutDetected = 3,
    SdSync = 4,
    Other = 255,
}

bitflags! {
    /// Bitmask of co-occurring warnings/errors, matching C++ `enum Error : uint16_t`.
    /// Logged via `logStatus`, so the wire layout must stay stable.
    #[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ErrorFlags: u16 {
        const SDCARD_CLOSED = 1 << 0;
        const NO_LSM       = 1 << 1;
        const NO_ADXL      = 1 << 2;
        const NO_LIS3      = 1 << 3;
        const NO_BMP       = 1 << 4;
        const SDCARD_INIT  = 1 << 5;

        const WARNING_OTHER = 1 << 14;
        const ERROR_OTHER   = 1 << 15;
    }
}
