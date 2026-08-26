//! LSM6DSOX: gyro + low-G accel (I²C). Skeleton.
//!
//! Target config (matches C++):
//!   - accel range  ±16 g
//!   - accel ODR    1.66 kHz
//!   - gyro  range  ±2000 °/s
//!   - gyro  ODR    1.66 kHz
//!
//! TODO: Don't inherit that 1.66 kHz gyro ODR from the C++ — the loop reads this
//!   at ~400 Hz, so sampling a 1.66 kHz stream decimates it 4x with no filter in
//!   between and folds everything from 200–830 Hz down into 0–200 Hz. That
//!   happens at the read, before AHRS sees anything, so no software filter can
//!   recover it. Instead set the gyro ODR to 416 Hz (the ladder is
//!   12.5/26/52/104/208/416/833/1660/3330/6660) and enable the on-chip LPF1.
//!   The on-chip filter runs ahead of the sensor's own decimation, so it does
//!   what software provably cannot. GYRO_LPF_HZ then only has to cover
//!   416 Hz → 50 Hz. Confirm the register details against ST's datasheet.
//! TODO: Same question for the accel ODR — pick it against the actual read rate
//!   rather than carrying over the C++ value.
//!
//! Driver strategy: use the `lsm6dsox` crate behind the `sensors` Cargo
//! feature. If its API turns out to be too sync-flavored, fall back to a
//! 200-line register-level driver — the LSM6DSOX register map is well-
//! documented in ST's datasheet and we only need a handful of writes.
//! TODO: Rotate the result to the correct orientation, since it sits in an orientation
//!  where -y is what we want Z to be.
use accelerometer::Accelerometer;
use defmt::*;
use embassy_embedded_hal::shared_bus::I2cDeviceError;
use embassy_time::Delay;
use embedded_hal::i2c::I2c;
use lsm6dsox::*;

fn init_lsm6dsox<I2C: I2c>(i2c: I2C) -> Result<(), Error<I2C::Error>> {
    // TODO: Figure out what address I actually need to use
    let mut lsm = Lsm6dsox::new(i2c, SlaveAddress::Low, Delay);

    lsm.setup()?;
    lsm.set_accel_sample_rate(DataRate::Freq416Hz)?; // TODO: tune based on loop speed
    lsm.set_accel_scale(AccelerometerScale::Accel16g)?;
    lsm.enable_interrupts(true)?;
    lsm.map_interrupt(InterruptSource::EmbeddedFunctions, InterruptLine::INT1, true)?;
    if let Ok(reading) = lsm.accel_norm() {
        info!("Acceleration: {:?}", Debug2Format(&reading));
    }
    Ok(())
}