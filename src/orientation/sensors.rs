use crate::utils::{LSMReading, LIS3Reading, ADXLReading, BMPReading, SensorReadings, Vec3};
use embedded_hal_async::i2c::I2c as AsyncI2c;

pub struct Sensors<I2C: AsyncI2c> {
    i2c: I2C,
}

impl<I2C: AsyncI2c> Sensors<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    pub async fn init(&mut self) -> Result<(), ()> {
        // Initialize LSM6DSOX
        // Initialize LIS3MDL
        // Initialize BMP390
        // Initialize ADXL375 (if on I2C)
        Ok(())
    }

    pub async fn read_all(&mut self) -> SensorReadings {
        // Mock readings for now
        SensorReadings {
            lsm: LSMReading {
                accel: Vec3::new(0.0, 0.0, 9.81),
                gyro: Vec3::new(0.0, 0.0, 0.0),
                temperature: 25.0,
            },
            lis3: LIS3Reading {
                mag: Vec3::new(0.0, 0.0, 0.0),
            },
            adxl: ADXLReading {
                highg_accel: Vec3::new(0.0, 0.0, 9.81),
            },
            bmp: BMPReading {
                pressure: 101325.0,
                temperature: 25.0,
                altitude: 0.0,
            },
        }
    }
}
