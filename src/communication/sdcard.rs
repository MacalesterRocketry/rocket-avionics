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

use crc::Crc;
use crate::communication::log_packets::PacketType;
use crate::config::board::SdConfig;
use crate::{mark_init_complete, mark_init_failed, mark_runtime_error, Subsystem, FLIGHT_STATE};
use defmt::{error, Format};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};
use postcard::ser_flavors::{Cobs, Slice};
use postcard::ser_flavors::crc::CrcModifier;
use serde::{Deserialize, Serialize};
use crate::navigation::gps::GPS_STATE;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Format)]
pub enum LogEntry {
    GPS(crate::navigation::gps::GpsState),
    State(crate::state::FlightState),
    Sensors(crate::sensors::SensorReadings),
    AHRS(crate::navigation::ahrs::AhrsState),
    Control(crate::state::ControlSetpoint),
    // TODO: Add some way to note PID, fin, and servo outputs (desired angular acceleration, desired deflection angle, and actual deflection angle).
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Format)]
pub struct LogPacket {
    pub timestamp_us: u64,
    pub entry: LogEntry,
}

pub async fn log_packet(entry: LogEntry) {
    let packet = LogPacket {
        timestamp_us: embassy_time::Instant::now().as_micros(),
        entry,
    };
    SD_LOGGING_CHANNEL.sender().send(packet).await
    // TODO: This blocks until the SD loop can write it.
    //  Figure out if we want that. I'm guessing not (dropping packets is better than blocking the hot loop), but we need to figure out how to handle that.
}

pub static SD_LOGGING_CHANNEL: Channel<CriticalSectionRawMutex, LogPacket, 256> = Channel::new();

const CRC: Crc<u32> = Crc::<u32>::new(&crc::CRC_32_ISCSI);
pub async fn sd_logging_loop(sd_config: SdConfig) {
    let sd_receiver = SD_LOGGING_CHANNEL.receiver();

    const WRITE_BUFFER_SIZE: usize = 512;
    let mut write_buffer = [0u8; WRITE_BUFFER_SIZE];
    let mut buffer_index = 0;

    mark_init_complete(Subsystem::SD_CARD);
    loop {
        let packet = sd_receiver.receive().await;

        // TODO: figure out what the length of the serialized maximum packet is. Can that be programmatic?
        let buf = &mut [0u8; 128];
        // let test = postcard::to_slice_cobs(&packet, buf);
        let ser_result = postcard::serialize_with_flavor::<LogPacket, _, _>(
            &packet,
            CrcModifier::new(
                Cobs::try_new(
                    Slice::new(buf)
                ).unwrap(), // TODO: fix unwrap()
                CRC.digest(),
            )
        );

        match ser_result {
            Ok(serialized_packet) => {
                let mut packet_remaining_bytes = serialized_packet;

                while !packet_remaining_bytes.is_empty() {
                    let buffer_space_left = WRITE_BUFFER_SIZE - buffer_index;
                    let chunk_size = packet_remaining_bytes.len().min(buffer_space_left); // how much can be written before either the buffer is full or the packet is finished

                    write_buffer[buffer_index..buffer_index + chunk_size]
                        .copy_from_slice(&packet_remaining_bytes[..chunk_size]);

                    buffer_index += chunk_size;
                    packet_remaining_bytes = &mut packet_remaining_bytes[chunk_size..]; // chop off the part that's been written to the buffer

                    // If buffer is full, flush it and reset so we can log the rest of the packet
                    if buffer_index >= WRITE_BUFFER_SIZE { // should never be >, but might as well be safe
                        // TODO: async sd_card.write_block(&write_buffer).await;
                        buffer_index = 0;
                    }
                }
            }
            Err(_e) => {
                error!("Failed to serialize packet");
                mark_runtime_error(Subsystem::SD_CARD);
                // TODO: Should probably add some packet saying there was an error (though who knows if it can be logged)
            }
        }
    }
}
