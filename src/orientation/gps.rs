//! MTK3333 GPS over UART. Skeleton.
//!
//! In the C++ source the GPS is fed by a 1 kHz repeating timer that drains
//! the UART RX buffer into the Adafruit_GPS library's ring buffer. In Embassy
//! the natural shape is an `async fn gps_task()` that does
//! `uart.read(&mut byte).await` in a loop and feeds bytes into the `nmea`
//! crate's parser.
//!
//! Steps to bring up:
//!   1. Init UART1 @ 9600 baud (RP2350 supports up to 460800 — overkill for 5 Hz NMEA).
//!   2. Send PMTK config strings: RMC+GGA, 5 Hz fix rate, 5 Hz NMEA rate, no antenna msgs.
//!   3. On each parsed sentence, update `LATEST` (an Embassy `Mutex<GpsFix>`)
//!      and ship a `PayloadGps` to the SD task via the log channel.
//!
//! Public surface mirrors `gps.h`: `init_gps`, `read_gps` (= drive parser one
//! step), `has_gps_fix`.

#![allow(dead_code, unused_variables)]

pub fn has_gps_fix() -> bool {
    // TODO: read from shared latest-fix state once the GPS task is wired up.
    false
}
