//! Shared sensor + state types. Direct port of the struct/enum declarations
//! at the bottom of the C++ `utils.h`.

use crate::math::Vec3;
use bitflags::bitflags;
use embassy_rp::pio;
use embassy_rp::pio_programs::ws2812::{PioWs2812, RgbColorOrder};
use embassy_time::Instant;
use crate::config::board::NUM_LEDS;
use crate::orientation::ahrs::AhrsState;
use crate::output::roll_controller::RollPid;

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
    pub accel: Vec3,
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

impl SensorReadings {
    /// Best-available acceleration for AHRS input: the low-G accel, unless
    /// it's saturated, in which case fall back to the high-G accelerometer.
    /// Mirrors the switch in `states.cpp`'s `STATE_ASCENT` handler.
    pub fn merged_accel(&self) -> Vec3 {
        let lowg = self.lsm.accel;
        if lowg.mag() >= crate::config::ACCELEROMETER_SWITCH_THRESHOLD {
            self.adxl.accel
        } else {
            lowg
        }
    }

    /// True once the high-G accelerometer has recorded a launch-magnitude
    /// acceleration spike. Mirrors `hasLaunched()` in the C++ source (the
    /// interrupt-based detector is wired but currently bypassed in favor of
    /// a plain magnitude check).
    pub fn has_launched(&self) -> bool {
        self.adxl.accel.mag() >= crate::config::LAUNCH_ACCEL_THRESHOLD_G * crate::config::G
    }
}

// ────────────────────────── state machine / events ──────────────────────────
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
