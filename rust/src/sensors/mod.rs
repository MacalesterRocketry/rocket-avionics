//! Sensor I/O. Skeleton.
//!
//! Each chip lives in its own submodule. The public surface here mirrors the
//! C++ `orientation/sensors.h` — `read_lsm()`, `read_lis3()`, `read_adxl()`,
//! `read_bmp()`, plus a combined `read_all()`. In the Rust port these are
//! `async fn`s driven by `embassy_rp::i2c::I2c` and SPI; the I²C bus is shared
//! via `embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice` so all four
//! chips can hang off the same Wire instance like in the C++ original.

#![allow(dead_code, unused_variables)]

pub mod adxl375;
pub mod bmp390;
pub mod lis3mdl;
pub mod lsm6dsox;

use crate::types::SensorReadings;

/// Read all four sensors. Returns biased + axis-corrected readings.
/// TODO: implement once individual driver wrappers are filled in.
pub async fn read_all() -> SensorReadings {
    SensorReadings::default()
}

/// Launch detector — magnitude check on the high-G accel. Mirrors the simple
/// "magnitude ≥ threshold" check from `hasLaunched()` in sensors.cpp (the
/// interrupt-based detector is wired but currently bypassed).
pub fn has_launched(highg_accel_mag_mps2: f64) -> bool {
    const G: f64 = crate::config::G;
    highg_accel_mag_mps2 >= (crate::config::LAUNCH_ACCEL_THRESHOLD_G * G)
}
