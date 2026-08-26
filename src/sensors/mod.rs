//! Sensor I/O.
//!
//! Each chip lives in its own submodule as a thin driver wrapper (init + read,
//! no cross-chip knowledge). `Sensors<I2C>` owns one instance of each and
//! mirrors the C++ `initSensors()` / `readSensors()` pair via `init_all()` /
//! `read_all()`. Once more than one chip is wired in, the I²C bus should be
//! shared via `embassy_embedded_hal::shared_bus`, same as the C++ Wire
//! instance — see the TODO on `Sensors`.

#![allow(dead_code, unused_variables)]

pub mod adxl375;
pub mod bmp390;
pub mod lis3mdl;
pub mod lsm6dsox;

use crate::utils::math::{AngularVec3, Vec3};

/// Owns every sensor driver instance sharing the I²C bus.
/// TODO: add lsm/lis3/bmp fields once their drivers are wired in.
pub struct Sensors<I2C: embedded_hal::i2c::I2c> {
    pub adxl: adxl375::Adxl<I2C>,
}

#[derive(Debug)]
pub enum InitError {
    Adxl(adxl375::Error),
}

/// Bring up every sensor on the shared I²C bus. Mirrors `initSensors()`.
pub fn init_all<I2C: embedded_hal::i2c::I2c>(i2c: I2C) -> Result<Sensors<I2C>, InitError> {
    let adxl = adxl375::Adxl::init(i2c).map_err(InitError::Adxl)?;
    Ok(Sensors { adxl })
}

impl<I2C: embedded_hal::i2c::I2c> Sensors<I2C> {
    /// Read all four sensors. Returns biased + axis-corrected readings.
    /// TODO: lsm/lis3/bmp still return defaults until their drivers land.
    /// TODO: Nothing here checks that a reading is actually *new*. If the loop
    ///   ever outruns a sensor's output data rate — or a sensor's ODR gets
    ///   lowered, which the aliasing TODOs in the driver modules argue for — the
    ///   same sample gets returned twice and AHRS integrates it as if time had
    ///   passed, which corrupts orientation and doubly so velocity/position.
    ///   The sensors all expose a data-ready bit (LSM6DSOX `STATUS_REG`, LIS3MDL
    ///   `STATUS_REG`, BMP390 `STATUS`), and their INT pins are already wired
    ///   through `InterruptConfig`, so the fix is to gate each read on
    ///   fresh-data and report staleness rather than silently duplicating.
    ///   Sensors run at different rates, so this is per-sensor, not per-tick.
    pub async fn read_all(&mut self) -> SensorReadings {
        let adxl = self.adxl.read().unwrap_or_else(|_| {
            defmt::warn!("ADXL375 read failed; using zeroed high-G reading for this tick");
            Default::default()
        });
        SensorReadings { adxl, ..Default::default() }
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct LsmReading {
    /// Body-frame linear acceleration (m/s²), bias-subtracted.
    pub accel: Vec3,
    /// Body-frame angular rate (rad/s), bias-subtracted.
    pub gyro: AngularVec3,
    /// Die temperature (°C).
    pub temperature: f64,
}

impl LsmReading {
    pub fn has_accel_saturated(&self) -> bool {
        self.accel.mag() >= crate::config::ACCELEROMETER_SWITCH_THRESHOLD
    }
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
        // TODO: Should probably detect if the low-G is offline and swap in the high-G
        if self.lsm.has_accel_saturated() {
            self.adxl.accel
        } else {
            self.lsm.accel
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

pub(crate) fn transform_sensor_axes(raw: Vec3) -> Vec3 {
    Vec3 {
        x: -raw.x,
        y: -raw.z,
        z: -raw.y,
    }
}
