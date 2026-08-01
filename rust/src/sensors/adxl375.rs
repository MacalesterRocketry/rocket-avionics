//! ADXL375 ±200 g accelerometer (I²C). Wraps the `adxl3xx` driver crate.
//!
//! Scale factor: 49 mg/LSB on ADXL375 (per datasheet table 1).

use adxl3xx::{AdxlBusI2c, Adxl375 as Adxl3xxDriver};
use embedded_hal::i2c::I2c as I2cBus;

use crate::config::{HIGHG_BIAS_X, HIGHG_BIAS_Y, HIGHG_BIAS_Z};
use crate::math::Vec3;
use crate::types::AdxlReading;

#[derive(Debug)]
pub enum Error {
    /// Device ID mismatch or register I/O failure during setup.
    Init,
    /// Axis-offset calibration failed.
    Calibration,
    /// Axis read failed.
    Read,
}

pub struct Adxl<I2C: I2cBus> {
    driver: Adxl3xxDriver<AdxlBusI2c<I2C>>,
}

impl<I2C: I2cBus> Adxl<I2C> {
    /// Bring up the high-G accelerometer: validate device ID, reset to
    /// datasheet defaults (800 Hz, FIFO stream), and run axis-offset
    /// calibration. Mirrors `initHighGAccelerometer()` in the C++ source,
    /// minus the activity-interrupt wiring — launch detection currently
    /// polls magnitude instead (see `SensorReadings::has_launched`).
    pub fn init(i2c: I2C) -> Result<Self, Error> {
        let bus = AdxlBusI2c { i2c, addr: adxl3xx::reg::ADXL_ADDR };
        let mut driver = Adxl3xxDriver::new(bus).map_err(|_| Error::Init)?;

        driver.init_defaults().map_err(|_| Error::Init)?;
        // TODO: figure out if we want to auto calibrate on launchpad somehow
        // driver.calibrate_axis_offsets().map_err(|_| Error::Calibration)?;

        Ok(Self { driver })
    }

    /// Read XYZ, bias-corrected to m/s².
    pub fn read(&mut self) -> Result<AdxlReading, Error> {
        let raw: Vec3 = self.driver.read_axis().map_err(|_| Error::Read)?.into();
        Ok(AdxlReading {
            accel: Vec3::new(
                raw.x - HIGHG_BIAS_X,
                raw.y - HIGHG_BIAS_Y,
                raw.z - HIGHG_BIAS_Z,
            ),
        })
    }
}