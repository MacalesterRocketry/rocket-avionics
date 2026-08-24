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
