//! Entry point + multicore executor setup.
//!
//! Core split (mirrors the porting plan):
//!   • Core 0 — sensor sampling, AHRS, state machine, roll PID, servo PWM.
//!     This is the hot loop; every µs matters.
//!   • Core 1 — SD card writes, GPS UART parsing, NeoPixel + buzzer.
//!     These can tolerate latency spikes; isolating them on a separate core
//!     keeps a 30 ms `sd.flush()` from stalling the control loop.

#![no_std]
#![no_main]
#![allow(dead_code, unused_imports)]

extern crate uom;

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

use embassy_executor::{Executor, SpawnError, SpawnToken};
use embassy_rp::multicore::{spawn_core1, Stack};
use static_cell::StaticCell;

use crate::config::board::{AvionicsHardware, I2cConfig, IndicatorsConfig, InterruptConfig, Neopixel, PeripheralConfig, SdConfig, UartConfig};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output};
use embassy_rp::i2c;
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use embassy_rp::{bind_interrupts, dma, pio, pio_programs, Peri, Peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal_async::i2c::I2c;
use smart_leds::hsv::{hsv2rgb, Hsv};
use smart_leds::{RGB8, RGBA};
use {defmt_rtt as _, panic_probe as _};
use crate::config::G;
use crate::config::board::NUM_LEDS;
use crate::state::{indicator_loop, FlightState, SystemState};

mod config;
mod log_packets;
mod math;
mod orientation;
mod output;
mod sensors;
mod state;
mod types;

// 8 KiB of stack for core 1. Lives in SCRATCH_X (see memory.x) so it doesn't
// share cache lines with the AHRS data on core 0. If a panic-probe trace shows
// core 1 stack overflow, bump this — there's 4 KiB of headroom in SCRATCH_X
// minus the linker-reserved bits.
#[unsafe(link_section = ".core1_stack")]
static mut CORE1_STACK: Stack<4096> = Stack::new();

// Bind the interrupt handler with the peripheral
bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

pub static FLIGHT_STATE: Watch<CriticalSectionRawMutex, FlightState, 2> = Watch::new();
pub static READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();  // core 1 → core 0

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    FLIGHT_STATE.sender().send(FlightState::PreLaunch(state::GroundSubState::Startup));

    let p = embassy_rp::init(Default::default());
    let hw = take_hardware!(p);

    // Spawn core 1's executor first so it's ready to receive packets the
    // moment core 0 starts producing them.
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || spawn_core(
            EXECUTOR1.init(Executor::new()),
            core1_main(hw.sd, hw.uart, hw.indicators)
        ),
    );

    spawn_core(
        EXECUTOR0.init(Executor::new()),
        core0_main(hw.i2c, hw.interrupts, hw.peripherals),
    );
}

fn spawn_core(
    executor: &'static mut Executor,
    core: Result<SpawnToken<impl Sized>, SpawnError>,
) -> ! {
    executor.run(|spawner| {
        let task = core.unwrap_or_else(|e| {
            error!("core: failed to spawn task: {:?}", e);
            defmt::panic!("core task spawn failed");
        });

        spawner.spawn(task);
    });
}

/// Core 0: the avionics hot loop. Skeleton — tasks below are TODOs.
#[embassy_executor::task]
async fn core0_main(
    i2c_config: I2cConfig,
    interrupt_config: InterruptConfig,
    peripheral_config: PeripheralConfig,
) {
    info!("core 0: avionics task starting");

    info!("initializing I²C bus");
    let mut config = i2c::Config::default();
    config.frequency = 400_000;
    let i2c = i2c::I2c::new_async(i2c_config.bus, i2c_config.scl, i2c_config.sda, Irqs, config);
    info!("I²C bus initialized");

    info!("initializing GPIO");
    let mut eject_button = Input::new(peripheral_config.eject_button, embassy_rp::gpio::Pull::Up);
    info!("GPIO initialized");

    info!("initializing sensors");
    let sensors = match sensors::init_all(i2c) {
        Ok(sensors) => sensors,
        Err(e) => {
            error!("Error initializing sensors: {:?}", defmt::Debug2Format(&e));
            return;
        }
    };
    info!("sensors initialized");

    info!("initializing system state");
    let mut system = SystemState::new(sensors).unwrap_or_else(|_| defmt::panic!("Failed to initialize system state"));
    // TODO: handle errors with Neopixel notifs and logging and stuff instead of panicking
    info!("system state initialized");

    // TODO (in this order, matching original `setup()`):
    //   1. init servo PWMs
    //   2. start AHRS state (record start time)
    //   3. transition state machine: Starting -> ReadyToLaunch
    let mut ticker = Ticker::every(Duration::from_hz(400));
    loop {
        ticker.next().await;
        // TODO: I'm wondering if the whole systemState thing should be split out into like 4-ish loops:
        //  state handling, sensors, AHRS, and PID/servos. I guess state handling would combine all the data?
        //  Or maybe just extract PID/servos? But tick() should definitely be more tightly integrated
        //  with the loop and ticker.
        system.tick().await;
    }
}

/// Core 1: I/O background. Owns SD card + GPS UART + indicators.
#[embassy_executor::task]
async fn core1_main(sd_config: SdConfig,
                    uart_config: UartConfig,
                    indicators_config: IndicatorsConfig,
) {
    info!("core 1: I/O task starting");
    info!("initializing buzzer");
    let mut buzzer = Output::new(indicators_config.buzzer, embassy_rp::gpio::Level::Low);
    buzzer.set_low();
    info!("buzzer initialized");

    info!("initializing NeoPixel");
    let Pio {
        mut common, sm0, ..
    } = Pio::new(indicators_config.neopixel_pio, Irqs);
    let program = PioWs2812Program::new(&mut common);
    let neopixel: Neopixel = PioWs2812::new(
        &mut common,
        sm0,
        indicators_config.neopixel_channel,
        Irqs,
        indicators_config.neopixel,
        &program,
    );
    info!("NeoPixel initialized");

    let state_receiver_option = FLIGHT_STATE.receiver();
    if state_receiver_option.is_none() { // TODO: switch to match
        defmt::panic!("Failed to get flight state receiver; have too many receivers been initialized?");
    }
    let state = state_receiver_option.unwrap();

    // TODO:
    //   • init SD card (SPI1 @ 50 MHz, embedded-sdmmc::VolumeManager)
    //   • init UART1 for GPS @ 9600 baud, send PMTK config


    READY.signal(());
    embassy_futures::join::join4(
        indicator_loop(neopixel, buzzer, state),
        sd_logging_loop(),
        gps_loop(),
        telemetry_loop(),
    ).await;
}

// TODO: All of these should be moved to their own files and actually implemented.
async fn sd_logging_loop() {
    let mut ticker: Ticker = Ticker::every(Duration::from_hz(20));
    loop {
        ticker.next().await;
    }
}

async fn gps_loop() {
    let mut ticker: Ticker = Ticker::every(Duration::from_hz(5));
    loop {
        ticker.next().await;
    }
}

async fn telemetry_loop() {
    let mut ticker: Ticker = Ticker::every(Duration::from_hz(5));
    loop {
        ticker.next().await;
    }
}