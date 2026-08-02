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

use crate::types::SensorReadings;

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
    pub async fn read_all(&mut self) -> SensorReadings {
        let adxl = self.adxl.read().unwrap_or_else(|_| {
            defmt::warn!("ADXL375 read failed; using zeroed high-G reading for this tick");
            Default::default()
        });
        SensorReadings { adxl, ..Default::default() }
    }
}
