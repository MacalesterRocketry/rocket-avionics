//! Orientation-adjacent modules: GPS lives here in the C++ source, so we keep
//! the same layout for review continuity even though the AHRS itself is now
//! at crate root (`crate::ahrs`).

pub mod gps;
pub mod ahrs;
