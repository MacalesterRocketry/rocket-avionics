//! ADXL375 ±200 g accelerometer (I²C). Skeleton.
//!
//! No mature Rust crate exists for this part as of late 2025. The register
//! map is a superset of ADXL343, so the plan is to write a thin async driver
//! supporting:
//!   - data-rate set (DATARATE_800_HZ)
//!   - activity threshold + interrupt mapping to INT1
//!   - getEvent equivalent: read XYZ, convert from LSB×scale → m/s²
//!
//! Scale factor: 49 mg/LSB on ADXL375 (per datasheet table 1) — verify before
//! trusting any biases the C++ source has hard-coded.
