// TODO: Note: This is untested. We don't have the GPS module yet.
#![allow(dead_code, unused_variables)]

use crate::config::board::GpsConfig;
use crate::{Irqs, Subsystem, mark_init_complete, mark_init_failed};
use chrono::prelude::*;
use core::sync::atomic::{AtomicU32, Ordering};
use defmt::{Format, info};
use embassy_executor::Spawner;
use embassy_rp::uart;
use embassy_rp::uart::{Async, BufferedUart, Uart, UartRx, UartTx};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read, Write};
use ublox::cfg_prt::{CfgPrtUartBuilder, UartMode, UartPortId};
use ublox::nav_pvt::proto33::NavPvtRef;
use ublox::{FixedBuffer, GnssFixType, Parser, ParserError, UbxPacket, UbxPacketMeta, UbxPacketRequest};
use ublox::cfg_msg::CfgMsgAllPorts;

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
            datetime: Some(get_datetime(&nav_pvt)),
            has_fix: has_gps_fix(&nav_pvt),
            last_fix_time: if self.has_fix { Some(Instant::now()) } else { self.last_fix_time },
            latitude: Some(nav_pvt.latitude()),
            longitude: Some(nav_pvt.longitude()),
            altitude_sealevel: Some(nav_pvt.height_msl()),
            vel_down: Some(nav_pvt.vel_down()),
            vel_east: Some(nav_pvt.vel_east()),
            vel_north: Some(nav_pvt.vel_north()),
            satellites: Some(nav_pvt.num_satellites()),
        }
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

    // This buffer size is basically just a guess.
    let mut parser = Parser::<FixedBuffer<1024>, ublox::proto33::Proto33>::new_fixed();
    let mut buffer = [0u8; 1024];

    let mut state = GpsState::new();

    mark_init_complete(Subsystem::GPS);
    let mut ticker: Ticker = Ticker::every(Duration::from_hz(5)); // TODO: should this actually be 5 Hz? What happens if it's slightly off from the GPS clock?
    // TODO: Or should it be faster? Or maybe use a GPS interrupt?
    loop {
        ticker.next().await;

        match uart.read(&mut buffer).await { // TODO: This is async. Should I just not do a ticker and let it go as fast as it can?
            Ok(bytes_read) => {
                let mut it = parser.consume_ubx(&buffer[..bytes_read]);
                loop {
                    match it.next() {
                        Some(Ok(UbxPacket::Proto33(p))) => {
                            info!("Received UBX packet: {}", defmt::Debug2Format(&p));
                            match p {
                                ublox::proto33::PacketRef::NavPvt(nav_pvt) => {
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

fn has_gps_fix(nav_pvt: &NavPvtRef) -> bool {
    match nav_pvt.fix_type() {
        GnssFixType::NoFix => {
            info!("No GPS fix");
            false
        },
        GnssFixType::TimeOnlyFix => {
            info!("Time only");
            false
        },
        GnssFixType::Fix2D => {
            info!("2D GPS");
            false
        },
        GnssFixType::Fix3D => {
            info!("3D GPS");
            true
        },
        _ => {
            info!("Unknown fix type: {}", nav_pvt.fix_type() as u8);
            false
        },
    }
}

fn get_datetime(nav_pvt: &NavPvtRef) -> DateTime<Utc> {
    NaiveDate::from_ymd_opt(nav_pvt.year() as i32, nav_pvt.month() as u32, nav_pvt.day() as u32)
        .unwrap() // TODO: remove unwrap()
        .and_hms_nano_opt(nav_pvt.hour() as u32, nav_pvt.min() as u32, nav_pvt.sec() as u32, nav_pvt.nanosec() as u32)
        .unwrap()
        .and_utc()
}
