//! SD card binary logging. Skeleton.
//!
//! Multicore design: the hot-loop tasks (sensor read, AHRS, control) call
//! `log_*` functions that just push a fixed-size `LogPacket` enum into an
//! Embassy SPSC channel. A dedicated `sd_writer_task` running on **core 1**
//! pops packets, serializes them via the writers in `crate::log_packets`,
//! and writes to the file. `dataFile.sync()` becomes an `f.flush().await`
//! every `SYNC_INTERVAL_MS` — non-blocking from core 0's perspective.
//!
//! This is the single biggest architectural win over the C++ version: in
//! C++, `dataFile.sync()` blocks the AHRS loop for ~30 ms whenever it fires.
//! On Embassy that 30 ms lives on core 1 and never touches the control path.

#![allow(dead_code, unused_variables)]

use crate::log_packets::PacketType;

/// Enum of every payload variant that can be pushed onto the SD log channel.
/// Defined as a single sum type so the channel has a fixed element size
/// (heapless::spsc requires `Copy + 'static`).
///
/// TODO: this enum needs a fixed-size variant for each `Payload*` struct in
/// `log_packets`. We'll size the channel to ~256 entries (≈12 KiB RAM) which
/// gives plenty of headroom even at burnout when the AHRS fires bursts.
#[derive(Debug, Clone, Copy)]
pub enum LogEntry {
    // populated in the porting step
    Placeholder,
}

pub fn log_packet(_pkt: PacketType, _entry: LogEntry, _micros: u64) {
    // TODO: enqueue onto static SD channel.
}
