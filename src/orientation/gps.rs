// TODO: Note: This is untested. We don't have the GPS module yet.
#![allow(dead_code, unused_variables)]

use crate::config::board::GpsConfig;

use core::sync::atomic::{AtomicU32, Ordering};
use defmt::{info, Format};
use embassy_executor::Spawner;
use embassy_rp::uart;
use embassy_rp::uart::{Async, BufferedUart, Uart, UartRx, UartTx};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read, Write};
use ublox::{FixedBuffer, ParserError, UbxPacket, UbxPacketRequest};
use crate::{mark_init_complete, Irqs, Subsystem, mark_init_failed};

static PARSER_BUFFER_SIZE: usize = 1024; // basically just a guess

pub async fn gps_loop(gps_config: GpsConfig) {
    let c = gps_config;
    let mut config = uart::Config::default();
    config.baudrate = 115200;
    // let mut uart = Uart::new(c.bus, c.tx, c.rx, Irqs, c.tx_dma, c.rx_dma, config);
    let mut uart = BufferedUart::new(c.bus, c.tx, c.rx, Irqs, &mut [0u8; 128], &mut [0u8; 1024], config);
    mark_init_complete(Subsystem::GPS);
    let mut ticker: Ticker = Ticker::every(Duration::from_hz(5)); // TODO: should this actually be 5 Hz? What happens if it's slightly off from the GPS clock?
    loop {
        ticker.next().await;
    }
}

pub fn has_gps_fix() -> bool {
    // TODO: read from shared latest-fix state once the GPS task is wired up.
    false
}







async fn main(gps_config: GpsConfig) -> Result<(), ()> {
    let c = gps_config;
    let mut config = uart::Config::default();
    config.baudrate = 115200;
    let mut port = BufferedUart::new(c.bus, c.tx, c.rx, Irqs, &mut [0u8; 128], &mut [0u8; 128], config);
    let mut parser = Parser::<FixedBuffer<PARSER_BUFFER_SIZE>, ublox::proto33::Proto33>::new_fixed();
    let mut buffer = [0u8; 1024];

    loop {
        match port.read(&mut buffer).await {
            Ok(bytes_read) => {
                let mut it = parser.consume_ubx(&buffer[..bytes_read]);
                loop {
                    match it.next() {
                        Some(Ok(UbxPacket::Proto33(p))) => {
                            handle_packet_proto33(p).await;
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

    Ok(())
}
async fn handle_packet_proto33(p: ublox::proto33::PacketRef<'_>) {
    info!("Received UBX packet: {}", defmt::Debug2Format(&p));
    match p {
        ublox::proto33::PacketRef::NavPvt(nav_pvt) => {
            info!("Speed: {} [m/s]", nav_pvt.ground_speed_2d())
        },
        ublox::proto33::PacketRef::EsfMeas(esf_meas) => {
            for data in esf_meas.data() {
                info!("ESF MEAS DATA: {}", defmt::Debug2Format(&data));
            }
        },
        _ => (),
    }
}

pub use ublox;
use ublox::{
    cfg_prt::{CfgPrtUart, CfgPrtUartBuilder, UartMode},
    Parser, UbxPacketMeta, UbxProtocol,
};
use ublox::cfg_msg::CfgMsgAllPortsBuilder;
use ublox::cfg_prt::UartPortId;
use ublox::esf_raw::EsfRaw;
use ublox::mon_ver::MonVer;

pub trait UbxPacketHandler {
    fn handle(&mut self, _packet: UbxPacket) {}
}

/// Implement handler for simple callbacks / closures
impl<F: FnMut(UbxPacket)> UbxPacketHandler for F {
    fn handle(&mut self, package: UbxPacket) {
        self(package)
    }
}

pub struct Device {
    port: BufferedUart,
    parser: Parser<FixedBuffer<PARSER_BUFFER_SIZE>>,
}

impl Device {
    pub fn new(port: BufferedUart) -> Device {
        let parser = Parser::<FixedBuffer<PARSER_BUFFER_SIZE>>::new_fixed();
        Device { port, parser }
    }

    pub async fn on_data_available<F: FnMut(UbxPacket)>(
        &mut self,
        mut callback: F,
    ) -> Result<(), ParserError> {
        self.process(&mut callback).await
    }

    pub async fn process(&mut self, handler: &mut impl UbxPacketHandler) -> Result<(), ParserError> {
        loop {
            const MAX_PAYLOAD_LEN: usize = 1240;
            let mut local_buf = [0; MAX_PAYLOAD_LEN];
            let nbytes = match self.port.read(&mut local_buf).await {
                Ok(n) => {
                    if n == 0 {
                        break;
                    }
                    n
                }
                Err(e) => {
                    info!("Error reading from serial port: {}", defmt::Debug2Format(&e));
                    // TODO: maybe return ParserError here?
                    break;
                }
            };

            // parser.consume_ubx adds the buffer to its internal buffer, and
            // returns an iterator-like object we can use to process the packets
            let mut it: ublox::UbxParserIter<'_, FixedBuffer<PARSER_BUFFER_SIZE>> =
                self.parser.consume_ubx(&local_buf[..nbytes]);
            loop {
                match it.next() {
                    Some(Ok(packet)) => {
                        handler.handle(packet);
                    },
                    Some(Err(e)) => {
                        info!("Malformed packet, ignore it; cause {}", defmt::Debug2Format(&e));
                    },
                    None => {
                        // debug!("Parsed all data in buffer ...");
                        break;
                    },
                }
            }
        }
        Ok(())
    }

    pub async fn wait_for_ack<T: UbxPacketMeta>(&mut self) -> Result<(), ParserError> {
        let mut found_packet = false;
        let start = Instant::now();
        let timeout = Duration::from_secs(3);
        while !found_packet {
            self.on_data_available(|packet| match packet {
                UbxPacket::Proto33(packet_ref) => {
                    if let ublox::proto33::PacketRef::AckAck(ack) = packet_ref {
                        if ack.class() == T::CLASS && ack.msg_id() == T::ID {
                            found_packet = true;
                        }
                    }
                },
            }).await?;

            if start.elapsed().as_millis() > timeout.as_millis() {
                info!("Did not receive ACK message for request");
                break;
            }
        }
        Ok(())
    }
}



async fn sending_thread(baud_rate: u32, port: BufferedUart) {
    let mut device: Device = Device::new(port);
    // Send out 4 bytes every second
    info!("Configuration thread: configuring UART1 port ...");
    // - configure the device UART1 to talk UBX with baud rate from CLI input
    device.port.write(
            &CfgPrtUartBuilder {
                portid: UartPortId::Uart1,
                reserved0: 0,
                tx_ready: 0,
                mode: UartMode::new(ublox::cfg_prt::DataBits::Eight, ublox::cfg_prt::Parity::None, ublox::cfg_prt::StopBits::One),
                baud_rate,
                in_proto_mask: ublox::cfg_prt::InProtoMask::UBLOX,
                out_proto_mask: ublox::cfg_prt::OutProtoMask::UBLOX, // disable NMEA
                flags: 0,
                reserved5: 0,
            }
            .into_packet_bytes(),
        ).await
        .expect("Could not configure UBX-CFG-PRT-UART");

    info!("Enable UBX-ESF-RAW message on selected ports ...");
    device.port.write(
            &CfgMsgAllPortsBuilder::set_rate_for::<EsfRaw>([0, 0, 0, 1, 0, 0])
                .into_packet_bytes(),
        ).await
        .expect("Could not configure ports for UBX-ESF-RAW");

    loop {
        info!(
            "Configuration thread: send request for UBX-ESF-RAW and UBX-MON-VER message  ..."
        );
        device.port
            .write(&UbxPacketRequest::request_for::<MonVer>().into_packet_bytes()).await
            .expect("Failed to send poll/request for UBX-MON-VER message");
        device.port
            .write(&UbxPacketRequest::request_for::<EsfRaw>().into_packet_bytes()).await
            .expect("Failed to send poll/request for UBX-ESF-RAW message");
        Timer::after(Duration::from_millis(1000)).await;
    }
}