use crate::config::board::{IndicatorsConfig, NUM_LEDS, Neopixel};
use crate::output::indication::{LedColor, drive_buzzer, set_neopixel_color};
use crate::state::FlightState;
use crate::{FLIGHT_STATE, Irqs, Subsystem, mark_init_complete, mark_init_failed};
use core::panic::PanicInfo;
use defmt::{error, info};
use embassy_rp::Peri;
use embassy_rp::Peripherals;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::*;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Receiver;
use embassy_time::{Duration, Instant, Ticker, block_for};
use smart_leds::RGB8;
use smart_leds::hsv::{Hsv, hsv2rgb};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    error!("{}", _info);

    handle_unrecoverable_error()
}

/// Heavily simplified version of the code in src/output/indication.rs, since most of the system is presumably not working when this function runs.
pub fn handle_unrecoverable_error() -> ! {
    // TODO: either stop core1 or set up watchdog

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