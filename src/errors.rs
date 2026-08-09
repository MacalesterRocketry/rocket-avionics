use core::panic::PanicInfo;
use defmt::{error, info};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::Peri;
use embassy_rp::Peripherals;
use embassy_rp::peripherals::*;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Receiver;
use embassy_time::{block_for, Duration, Instant, Ticker};
use smart_leds::hsv::{hsv2rgb, Hsv};
use smart_leds::RGB8;
use crate::config::board::{IndicatorsConfig, Neopixel, NUM_LEDS};
use crate::{mark_init_complete, mark_init_failed, Irqs, Subsystem, FLIGHT_STATE};
use crate::output::indication::{drive_buzzer, set_neopixel_color, LedColor};
use crate::state::FlightState;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    error!("{}", _info);

    handle_unrecoverable_error()
}

/// Heavily simplified version of the code in src/output/indication.rs, since most of the system is presumably not working when this function runs.
pub fn handle_unrecoverable_error() -> ! {
    let p = unsafe { Peripherals::steal() };

    // Configure the buzzer pin manually
    // We avoid complex macros here to ensure nothing relies on the crashed state.
    let mut buzzer = Output::new(p.PIN_43, Level::Low);
    // TODO: Once I decide which pin to use for the LED, update it here
    let mut led = Output::new(p.PIN_44, Level::Low);

    // Constant fast beeping on and off (Synchronous)
    loop {
        buzzer.set_high();
        led.set_high();
        block_for(Duration::from_millis(100)); // Blocks the CPU, no executor needed

        buzzer.set_low();
        led.set_low();
        block_for(Duration::from_millis(100));
    }
}