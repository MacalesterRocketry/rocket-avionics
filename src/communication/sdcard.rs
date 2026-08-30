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

use core::fmt::Write;
use crate::config::board::SdConfig;
use crate::navigation::gps::GPS_STATE;
use crate::{FLIGHT_STATE, Subsystem, mark_init_complete, mark_init_failed, mark_runtime_error};
use chrono::DateTime;
use core::cell::RefCell;
use core::convert::Infallible;
use core::fmt::write;
use core::ops::ControlFlow;
use crc::Crc;
use defmt::{Format, error};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi;
use embassy_rp::spi::{Blocking, Spi};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::Mode::{ReadWriteCreate, ReadWriteCreateOrTruncate};
use embedded_sdmmc::filesystem::ToShortFileName;
use embedded_sdmmc::sdcard::spi::Error;
use embedded_sdmmc::{Directory, File, SdCard, TimeSource, Timestamp, VolumeManager};
use heapless::String;
use postcard::ser_flavors::crc::CrcModifier;
use postcard::ser_flavors::{Cobs, Slice};
use serde::{Deserialize, Serialize};
use crate::utils::unwrap_infallible;

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
        timestamp_us: Instant::now().as_micros(),
        entry,
    };
    SD_LOGGING_CHANNEL.sender().send(packet).await
    // TODO: This blocks until the SD loop can write it.
    //  Figure out if we want that. I'm guessing not (dropping packets is better than blocking the hot loop), but we need to figure out how to handle that.
}

struct SDTimeSource {
    now: Instant,
    datetime: Option<DateTime<chrono::Utc>>,
}

impl TimeSource for SDTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        // TODO: this is a placeholder
        Timestamp::from_calendar(2026, 8, 29, 11, 08, 16).expect("Invalid date/time")
    }
}

pub static SD_LOGGING_CHANNEL: Channel<CriticalSectionRawMutex, LogPacket, 256> = Channel::new();

const CRC: Crc<u32> = Crc::<u32>::new(&crc::CRC_32_ISCSI);
pub async fn sd_logging_loop(sd_config: SdConfig) {
    let sd_receiver = SD_LOGGING_CHANNEL.receiver();

    let sd_card = init_sd_card(sd_config);
    let vol_mgr: RocketVolumeManager = VolumeManager::new(sd_card, SDTimeSource { now: Instant::now(), datetime: None });
    let file = match open_log_file(&vol_mgr) {
        Ok(file) => file,
        Err(e) => {
            error!("Failed to initialize SD card: {}", e);
            // TODO: Do I need to unmount it or something here?
            mark_init_failed(Subsystem::SD_CARD);
            return;
        }
    };

    const WRITE_BUFFER_SIZE: usize = 512;
    let mut write_buffer = [0u8; WRITE_BUFFER_SIZE];
    let mut buffer_index = 0;

    mark_init_complete(Subsystem::SD_CARD);
    loop {
        let packet = sd_receiver.receive().await;

        // TODO: figure out what the length of the serialized maximum packet is. Can that be programmatic?
        let buf = &mut [0u8; 128];
        let ser_result = postcard::serialize_with_flavor::<LogPacket, _, _>(
            &packet,
            CrcModifier::new(
                Cobs::try_new(
                    Slice::new(buf)
                ).unwrap(), // TODO: fix unwrap()
                CRC.digest(),
            )
        );

        // TODO: add ability to close SD card
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
                        // TODO: This doesn't write by blocks, since it's in a file. Figure out what's most efficient.
                        match file.write(&write_buffer) {
                            Ok(_) => {},
                            Err(e) => {
                                error!("Failed to write block: {}", e);
                                mark_runtime_error(Subsystem::SD_CARD);
                            }
                        };
                        // TODO: Should we write multiple blocks at once? Would require a small refactor.
                        buffer_index = 0;
                    }
                }
            }
            Err(e) => {
                error!("Failed to serialize packet: {}", e);
                mark_runtime_error(Subsystem::SD_CARD);
                // TODO: Should probably add some packet saying there was an error (though who knows if it can be logged)
            }
        }
    }
}

type RocketSdCard<'a> = SdCard<ExclusiveDevice<Spi<'a, SPI0, Blocking>, Output<'a>, Delay>, Delay>;
type RocketVolumeManager<'a> = VolumeManager<RocketSdCard<'a>, SDTimeSource>;
type RocketFile<'a, 'd> = File<'a, RocketSdCard<'d>, SDTimeSource, 4, 4, 1>;
fn open_log_file<'a, 'd>(vol_mgr: &'a RocketVolumeManager<'d>) -> Result<RocketFile<'a, 'd>, embedded_sdmmc::Error<Error>> {
    let vol = vol_mgr.open_volume(embedded_sdmmc::VolumeIdx(0))?;
    let root_dir = vol.open_root_dir()?;

    let filename = choose_filename(&root_dir)?;

    // TODO: better handle if the file already exists somehow (maybe switch to using UUID or something)
    let file = root_dir.open_file_in_dir(
        filename.as_str(),
        ReadWriteCreate,
    )?;
    Ok(file)
}

fn init_sd_card<'d>(sd_config: SdConfig) -> RocketSdCard<'d> {
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 50_000_000; // 50MHz
    // TODO: Needs to init a lot slower then move up. Look at SdCard docs.
    let spi = Spi::new_blocking(sd_config.spi, sd_config.sclk, sd_config.mosi, sd_config.miso, spi_config);
    let sd_cs = Output::new(sd_config.cs, Level::High);
    let spi_device = unwrap_infallible(ExclusiveDevice::new(spi, sd_cs, Delay));
    SdCard::new(spi_device, Delay)
}

fn choose_filename(root_dir: &Directory<RocketSdCard<'_>, SDTimeSource, 4, 4, 1>) -> Result<String<12>, embedded_sdmmc::Error<Error>> {
    let mut highest_file_id: i32 = -1;

    root_dir.iterate_dir(|entry| {
        // 12 chars long; FAT stores 11 chars excluding the . for the extension
        let mut name_buf: String<12> = String::new();

        // Write the filename we're checking into the String buffer
        if write!(&mut name_buf, "{}", entry.name).is_ok() {

            // Capitalized because FAT is case-insensitive and stores everything as uppercase
            if name_buf.starts_with("LOG") && name_buf.ends_with(".BIN") {
                // Slice out the numeric portion (everything after "LOG" and before ".BIN")
                let number_slice = &name_buf[3..(name_buf.len() - 4)];

                if let Ok(val) = number_slice.parse::<i32>() {
                    if val > highest_file_id {
                        highest_file_id = val;
                    }
                }
            }
        }
        ControlFlow::Continue(())
    })?;

    let next_file_id = (highest_file_id + 1) as u32;

    let mut filename: String<12> = String::new();
    write!(&mut filename, "LOG{}.BIN", next_file_id)?;
    Ok(filename)
}
