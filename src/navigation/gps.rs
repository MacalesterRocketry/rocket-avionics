//! GPS driver for the u-blox MAX-M10S.
//!
//! This module provides a driver for the u-blox MAX-M10S GPS/GNSS module, which is used to
//! obtain position, velocity, and time information.
//!
//! TODO: Note: This is untested. We don't have the GPS module yet.

#![allow(dead_code, unused_variables)]

use core::ops::BitAnd;
use crate::config::board::GpsConfig;
use crate::{Irqs, Subsystem, mark_init_complete, mark_init_failed};
use chrono::prelude::*;
use core::sync::atomic::{AtomicU32, Ordering};
use defmt::{Format, info};
use embassy_executor::Spawner;
use embassy_rp::uart;
use embassy_rp::uart::{Async, BufferedUart, Uart, UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read, Write};
use serde::{Deserialize, Serialize};
use ublox::cfg_msg::{CfgMsgAllPorts, CfgMsgAllPortsBuilder, CfgMsgSinglePortBuilder};
use ublox::cfg_prt::{CfgPrtUartBuilder, UartMode, UartPortId};
use ublox::cfg_rate::{AlignmentToReferenceTime, CfgRateBuilder};
use ublox::nav_pvt::proto33::{NavPvt, NavPvtRef};
use ublox::{FixedBuffer, GnssFixType, Parser, ParserError, UbxPacket, UbxPacketMeta, UbxPacketRequest};
use ublox::nav_pvt::common::NavPvtFlags;

pub static GPS_STATE: Watch<CriticalSectionRawMutex, GpsState, 3> = Watch::new();
// TODO: or maybe use mutex? idk

#[derive(Clone, Copy, Debug, Format)]
pub struct GpsState {
    pub last_packet_time: Option<Instant>,
    pub last_fix_time: Option<Instant>,
    pub has_fix: bool,
    pub datetime: Option<DateTime<Utc>>,
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    pub altitude_sealevel: Option<f64>,
    pub vel_down: Option<f64>,
    pub vel_east: Option<f64>,
    pub vel_north: Option<f64>,
    pub satellites: Option<u8>,
}

impl GpsState {
    pub fn new() -> Self {
        Self {
            last_packet_time: None,
            last_fix_time: None,
            has_fix: false,
            datetime: None,
            latitude: None,
            longitude: None,
            altitude_sealevel: None,
            vel_down: None,
            vel_east: None,
            vel_north: None,
            satellites: None,
        }
    }

    fn update(&mut self, nav_pvt: &NavPvtRef) {
        *self = Self {
            last_packet_time: Some(Instant::now()),
            datetime: get_datetime(&nav_pvt),
            has_fix: nav_pvt.flags().contains(NavPvtFlags::GPS_FIX_OK),
            last_fix_time: if self.has_fix { Some(Instant::now()) } else { self.last_fix_time },
            latitude: Some(nav_pvt.latitude()),
            longitude: Some(nav_pvt.longitude()),
            altitude_sealevel: Some(nav_pvt.height_msl()),
            vel_down: Some(nav_pvt.vel_down()),
            vel_east: Some(nav_pvt.vel_east()),
            vel_north: Some(nav_pvt.vel_north()),
            satellites: Some(nav_pvt.num_satellites()),
        };
        GPS_STATE.sender().send(self.clone());
    }
}

pub async fn gps_loop(gps_config: GpsConfig) {
    let c = gps_config;

    // Configure UART
    let mut config = uart::Config::default();
    config.baudrate = 9600; // TODO: Do I want to make it faster?
    config.parity = uart::Parity::ParityNone;
    config.stop_bits = uart::StopBits::STOP1;
    config.data_bits = uart::DataBits::DataBits8;
    let mut uart = BufferedUart::new(c.bus, c.tx, c.rx, Irqs, &mut [0u8; 128], &mut [0u8; 1024], config);

    // Configure the module with matching config to UART
    uart.write(
        &CfgPrtUartBuilder {
            portid: UartPortId::Uart1,
            reserved0: 0,
            tx_ready: 0,
            mode: UartMode::new(ublox::cfg_prt::DataBits::Eight, ublox::cfg_prt::Parity::None, ublox::cfg_prt::StopBits::One),
            baud_rate: 9600,
            in_proto_mask: ublox::cfg_prt::InProtoMask::UBLOX,
            out_proto_mask: ublox::cfg_prt::OutProtoMask::UBLOX, // disable NMEA
            flags: 0,
            reserved5: 0,
        }.into_packet_bytes()
    ).await.expect("Could not configure UBX-CFG-PRT-UART");
    // Wait a bit for the module to initialize
    Timer::after(Duration::from_millis(200)).await;
    // Set UBX-CFG-RATE to 100ms
    uart.write(
        &CfgRateBuilder {
            measure_rate_ms: 100, // measure every 100ms
            nav_rate: 1, // produce a navigation solution every 1 measurement
            time_ref: AlignmentToReferenceTime::Utc, // UTC time has leap seconds, while GPS time doesn't.
                                                     // If this is ever used for a monotonic clock, this needs to be changed.
        }.into_packet_bytes()
    ).await.expect("Could not configure UBX-CFG-RATE");
    Timer::after(Duration::from_millis(50)).await;
    // Set UBX-CFG-MSG to enable NAV-PVT on UART1
    uart.write(
        &CfgMsgSinglePortBuilder::set_rate_for::<NavPvt>(1).into_packet_bytes()
    ).await.expect("Could not configure UBX-CFG-MSG for NAV-PVT");
    Timer::after(Duration::from_millis(50)).await;

    // These buffer sizes are basically just a guess.
    // The UART buffer stores the raw bytes from the UART, and the parser buffer stores the parsed
    // packets before they've been processed. The sizes don't need to be the same.
    let mut uart_buffer = [0u8; 1024];
    let mut parser = Parser::<FixedBuffer<1024>, ublox::proto33::Proto33>::new_fixed();

    let mut state = GpsState::new();

    mark_init_complete(Subsystem::GPS);
    loop {
        match uart.read(&mut uart_buffer).await {
            Ok(bytes_read) => {
                let mut it = parser.consume_ubx(&uart_buffer[..bytes_read]);
                loop {
                    match it.next() {
                        Some(Ok(UbxPacket::Proto33(p))) => {
                            match p {
                                ublox::proto33::PacketRef::NavPvt(nav_pvt) => {
                                    // TODO: I'm not confident that every one of these packets
                                    //  will include all the data we need. How can I find if it
                                    //  does or not?
                                    state.update(&nav_pvt);
                                },
                                _ => (),
                            }
                        },
                        Some(Err(e)) => {
                            info!("Received malformed packet: {}", defmt::Debug2Format(&e));
                        },
                        None => {
                            // The internal buffer is now empty
                            break;
                        },
                    }
                }
            },
            Err(e) => {
                info!("Error reading from serial port: {}", defmt::Debug2Format(&e));
                break;
            },
        }
    }
}

fn get_datetime(nav_pvt: &NavPvtRef) -> Option<DateTime<Utc>> {
    let date = NaiveDate::from_ymd_opt(
        nav_pvt.year() as i32,
        nav_pvt.month() as u32,
        nav_pvt.day() as u32
    )?;

    let time = date.and_hms_nano_opt(
        nav_pvt.hour() as u32,
        nav_pvt.min() as u32,
        nav_pvt.sec() as u32,
        nav_pvt.nanosec() as u32
    )?;

    Some(time.and_utc())
}
