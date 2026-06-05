//! LSM6DSOX: gyro + low-G accel (I²C). Skeleton.
//!
//! Target config (matches C++):
//!   - accel range  ±16 g
//!   - accel ODR    1.66 kHz
//!   - gyro  range  ±2000 °/s
//!   - gyro  ODR    1.66 kHz
//!
//! Driver strategy: use the `lsm6dsox` crate behind the `sensors` Cargo
//! feature. If its API turns out to be too sync-flavored, fall back to a
//! 200-line register-level driver — the LSM6DSOX register map is well-
//! documented in ST's datasheet and we only need a handful of writes.
