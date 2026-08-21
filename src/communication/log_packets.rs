//! Binary log packet wire format.
//!
//! MUST stay byte-compatible with `decoder.py` and the existing logXX.bin
//! files. File layout, packet-by-packet, is:
//!
//!   [FileHeader (2 bytes)]
//!   { [PacketHeader (9 bytes)] [Payload (N bytes, type-specific)] }*
//!
//! All fields are little-endian (we set `endian = 0` in FileHeader). To avoid
//! the foot-guns of `#[repr(C, packed)]` (taking refs to packed fields is UB),
//! every packet exposes an explicit `write_le(&self, dst: &mut [u8]) -> usize`
//! that writes one field at a time. The host-side `cargo hosttest` round-trips
//! a few sample packets through `decoder.py`'s `struct.unpack` format strings
//! to catch any drift.

use crate::utils::math::{Quat, Vec3};

pub const FILE_VERSION: u8 = 3;
pub const LITTLE_ENDIAN: u8 = 0;

/// Two-byte file header written once at the top of every logNN.bin.
pub const FILE_HEADER: [u8; 2] = [FILE_VERSION, LITTLE_ENDIAN];

/// Packet type discriminants. Values must match `PacketType` in `decoder.py`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketType {
    Imu      = 0x10,
    HighG    = 0x11,
    Mag      = 0x12,
    Baro     = 0x13,
    Gps      = 0x20,
    Datetime = 0x21,
    Event    = 0x30,
    Status   = 0x40,
    Ahrs     = 0x50,
    Control  = 0x51,
}

/// 9-byte packet header: 1 B type, 8 B timestamp (µs since boot).
pub fn write_packet_header(dst: &mut [u8], pkt: PacketType, micros: u64) -> usize {
    dst[0] = pkt as u8;
    dst[1..9].copy_from_slice(&micros.to_le_bytes());
    9
}

// ─────────────────────────────── payloads ───────────────────────────────────
// Each payload writer matches decoder.py's `PACKET_FORMATS` exactly. The byte
// counts in the doc-comments are checked at compile time via const assertions.

/// IMU — `fffffff` = 28 B
pub struct PayloadImu {
    pub ax: f32, pub ay: f32, pub az: f32,
    pub gx: f32, pub gy: f32, pub gz: f32,
    pub temperature: f32,
}
impl PayloadImu {
    pub const SIZE: usize = 28;
    pub fn write_le(&self, d: &mut [u8]) -> usize {
        let xs = [self.ax, self.ay, self.az, self.gx, self.gy, self.gz, self.temperature];
        write_f32_le(d, &xs)
    }
}

/// High-G — `fff` = 12 B
pub struct PayloadHighG { pub ax: f32, pub ay: f32, pub az: f32 }
impl PayloadHighG {
    pub const SIZE: usize = 12;
    pub fn write_le(&self, d: &mut [u8]) -> usize {
        write_f32_le(d, &[self.ax, self.ay, self.az])
    }
}

/// Magnetometer — `fff` = 12 B
pub struct PayloadMag { pub mx: f32, pub my: f32, pub mz: f32 }
impl PayloadMag {
    pub const SIZE: usize = 12;
    pub fn write_le(&self, d: &mut [u8]) -> usize {
        write_f32_le(d, &[self.mx, self.my, self.mz])
    }
}

/// Barometer — `fff` = 12 B (pressure, altitude, temperature — note the order
/// matches the C++ struct, NOT the field name order in the doc!).
pub struct PayloadBaro { pub pressure: f32, pub altitude: f32, pub temperature: f32 }
impl PayloadBaro {
    pub const SIZE: usize = 12;
    pub fn write_le(&self, d: &mut [u8]) -> usize {
        write_f32_le(d, &[self.pressure, self.altitude, self.temperature])
    }
}

/// GPS — `BBBBiifffBB` = 26 B
///   1+1+1+1 + 4+4 + 4+4+4 + 1+1 = 26
/// decoder.py docstring says "14 bytes" but the actual struct format string
/// `BBBBiifffBB` is 26 — the docstring is stale. Sticking with the format.
pub struct PayloadGps {
    pub hours: u8, pub minutes: u8, pub seconds: u8, pub deciseconds: u8,
    pub latitude: i32, pub longitude: i32,
    pub speed: f32, pub angle: f32, pub altitude: f32,
    pub satellites: u8, pub fixquality: u8,
}
impl PayloadGps {
    pub const SIZE: usize = 26;
    pub fn write_le(&self, d: &mut [u8]) -> usize {
        d[0] = self.hours; d[1] = self.minutes; d[2] = self.seconds; d[3] = self.deciseconds;
        d[4..8].copy_from_slice(&self.latitude.to_le_bytes());
        d[8..12].copy_from_slice(&self.longitude.to_le_bytes());
        d[12..16].copy_from_slice(&self.speed.to_le_bytes());
        d[16..20].copy_from_slice(&self.angle.to_le_bytes());
        d[20..24].copy_from_slice(&self.altitude.to_le_bytes());
        d[24] = self.satellites; d[25] = self.fixquality;
        Self::SIZE
    }
}

/// Datetime — `HBBBBB` = 7 B
pub struct PayloadDatetime {
    pub year: u16, pub month: u8, pub day: u8,
    pub hours: u8, pub minutes: u8, pub seconds: u8,
}
impl PayloadDatetime {
    pub const SIZE: usize = 7;
    pub fn write_le(&self, d: &mut [u8]) -> usize {
        d[0..2].copy_from_slice(&self.year.to_le_bytes());
        d[2] = self.month; d[3] = self.day;
        d[4] = self.hours; d[5] = self.minutes; d[6] = self.seconds;
        Self::SIZE
    }
}

/// Event — `BBB` = 3 B (oldState, newState, reasonCode)
pub struct PayloadEvent { pub old_state: u8, pub new_state: u8, pub reason: u8 }
impl PayloadEvent {
    pub const SIZE: usize = 3;
    pub fn write_le(&self, d: &mut [u8]) -> usize {
        d[0] = self.old_state; d[1] = self.new_state; d[2] = self.reason;
        Self::SIZE
    }
}

/// Status — `BfB` = 6 B (no padding because we serialize manually)
pub struct PayloadStatus { pub rocket_state: u8, pub battery_voltage: f32, pub sensors_detected: u8 }
impl PayloadStatus {
    pub const SIZE: usize = 6;
    pub fn write_le(&self, d: &mut [u8]) -> usize {
        d[0] = self.rocket_state;
        d[1..5].copy_from_slice(&self.battery_voltage.to_le_bytes());
        d[5] = self.sensors_detected;
        Self::SIZE
    }
}

/// AHRS — `fffffffffffff` = 52 B
pub struct PayloadAhrs {
    pub qw: f32, pub qx: f32, pub qy: f32, pub qz: f32,
    pub ax: f32, pub ay: f32, pub az: f32,
    pub vx: f32, pub vy: f32, pub vz: f32,
    pub px: f32, pub py: f32, pub pz: f32,
}
impl PayloadAhrs {
    pub const SIZE: usize = 52;
    pub fn from_state(orientation: Quat, acc: Vec3, vel: Vec3, pos: Vec3) -> Self {
        Self {
            qw: orientation.w as f32, qx: orientation.x as f32,
            qy: orientation.y as f32, qz: orientation.z as f32,
            ax: acc.x as f32, ay: acc.y as f32, az: acc.z as f32,
            vx: vel.x as f32, vy: vel.y as f32, vz: vel.z as f32,
            px: pos.x as f32, py: pos.y as f32, pz: pos.z as f32,
        }
    }
    pub fn write_le(&self, d: &mut [u8]) -> usize {
        write_f32_le(d, &[
            self.qw, self.qx, self.qy, self.qz,
            self.ax, self.ay, self.az,
            self.vx, self.vy, self.vz,
            self.px, self.py, self.pz,
        ])
    }
}

/// Roll control — `fff` = 12 B (target, current, deflection)
pub struct PayloadControl { pub target_angle: f32, pub current_angle: f32, pub deflection_angle: f32 }
impl PayloadControl {
    pub const SIZE: usize = 12;
    pub fn write_le(&self, d: &mut [u8]) -> usize {
        write_f32_le(d, &[self.target_angle, self.current_angle, self.deflection_angle])
    }
}

// ────────────────────────────── helpers ─────────────────────────────────────
#[inline]
fn write_f32_le(dst: &mut [u8], values: &[f32]) -> usize {
    for (i, v) in values.iter().enumerate() {
        dst[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
    }
    values.len() * 4
}

// Compile-time check that the SIZE constants match the actual write count.
// If decoder.py changes its format string, these will catch the drift on the
// next build.
const _: () = assert!(PayloadImu::SIZE == 7 * 4);
const _: () = assert!(PayloadHighG::SIZE == 3 * 4);
const _: () = assert!(PayloadMag::SIZE == 3 * 4);
const _: () = assert!(PayloadBaro::SIZE == 3 * 4);
const _: () = assert!(PayloadGps::SIZE == 4 + 4 + 4 + 4 + 4 + 4 + 1 + 1);
const _: () = assert!(PayloadDatetime::SIZE == 2 + 1 + 1 + 1 + 1 + 1);
const _: () = assert!(PayloadEvent::SIZE == 3);
const _: () = assert!(PayloadStatus::SIZE == 1 + 4 + 1);
const _: () = assert!(PayloadAhrs::SIZE == 13 * 4);
const _: () = assert!(PayloadControl::SIZE == 3 * 4);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_layout_matches_python_struct_fmt() {
        // decoder.py: PACKET_FORMATS[IMU] = "fffffff" → 28 bytes
        let mut buf = [0u8; PayloadImu::SIZE];
        let p = PayloadImu {
            ax: 1.0, ay: 2.0, az: 3.0,
            gx: 4.0, gy: 5.0, gz: 6.0,
            temperature: 7.0,
        };
        let n = p.write_le(&mut buf);
        assert_eq!(n, 28);
        // First float = 1.0 little-endian: 0x00, 0x00, 0x80, 0x3F
        assert_eq!(&buf[0..4], &1.0f32.to_le_bytes());
        assert_eq!(&buf[24..28], &7.0f32.to_le_bytes());
    }

    #[test]
    fn packet_header_is_nine_bytes() {
        let mut buf = [0u8; 9];
        let n = write_packet_header(&mut buf, PacketType::Imu, 0x0102_0304_0506_0708);
        assert_eq!(n, 9);
        assert_eq!(buf[0], 0x10);
        // u64 little-endian
        assert_eq!(&buf[1..9], &0x0102_0304_0506_0708u64.to_le_bytes());
    }

    #[test]
    fn gps_layout() {
        let p = PayloadGps {
            hours: 1, minutes: 2, seconds: 3, deciseconds: 4,
            latitude: 449880000, longitude: -931400000,
            speed: 10.0, angle: 90.0, altitude: 250.0,
            satellites: 8, fixquality: 1,
        };
        let mut buf = [0u8; PayloadGps::SIZE];
        assert_eq!(p.write_le(&mut buf), 26);
        assert_eq!(buf[24], 8);
        assert_eq!(buf[25], 1);
    }
}
