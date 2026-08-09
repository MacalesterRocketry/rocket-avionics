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

use crate::config::G;
use crate::config::board::NUM_LEDS;
use crate::config::board::{
    AvionicsHardware, I2cConfig, IndicatorsConfig, InterruptConfig, Neopixel, PeripheralConfig,
    SdConfig, UartConfig,
};
use crate::state::{FlightState, SystemState};
use core::sync::atomic::{AtomicU8, Ordering};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::{Executor, SpawnError, SpawnToken, Spawner};
use embassy_rp::gpio::{Input, Output};
use embassy_rp::i2c;
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use embassy_rp::{Peri, Peripherals, bind_interrupts, dma, pio, pio_programs};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::i2c::I2c;
use output::indication::indicator_loop;
use smart_leds::hsv::{Hsv, hsv2rgb};
use smart_leds::{RGB8, RGBA};
use static_cell::StaticCell;

mod config;
mod errors;
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

bitflags! {
    pub struct Subsystem: u8 {
        const BASE_SYSTEM = 1 << 0;
        const INDICATORS  = 1 << 1;
        const SD_CARD     = 1 << 2;
        const GPS         = 1 << 3;
        const SENSORS     = 1 << 4; // core 0
    }
}
pub static FLIGHT_CRITICAL_SUBSYSTEMS: &[Subsystem] = &[Subsystem::BASE_SYSTEM, Subsystem::SENSORS];
pub static INIT_DONE: AtomicU8 = AtomicU8::new(0);
pub static INIT_FAILED: AtomicU8 = AtomicU8::new(0);
pub static RUNTIME_FAILURES: AtomicU8 = AtomicU8::new(0);


fn check_subsystems_any(subsystems: &[Subsystem], subsystem_flags: &AtomicU8) -> bool {
    subsystems
        .iter()
        .any(|s| {
            Subsystem::from_bits_truncate(subsystem_flags.load(Ordering::Acquire)).contains(*s)
        })
}
fn check_subsystems_all(subsystems: &[Subsystem], subsystem_flags: &AtomicU8) -> bool {
    subsystems
        .iter()
        .all(|s| {
            Subsystem::from_bits_truncate(subsystem_flags.load(Ordering::Acquire)).contains(*s)
        })
}

pub fn mark_init_complete(subsystem: Subsystem) {
    info!("subsystem {} initialized", subsystem);
    INIT_DONE.fetch_or(subsystem.bits(), Ordering::Release);
}
pub fn mark_init_failed(subsystem: Subsystem) {
    error!("subsystem {} failed to initialize", subsystem);
    INIT_FAILED.fetch_or(subsystem.bits(), Ordering::Release);
}
pub fn is_init_all_complete() -> bool {
    Subsystem::from_bits_truncate(INIT_DONE.load(Ordering::Acquire)) == Subsystem::all()
}
pub fn is_init_critical_complete() -> bool {
    check_subsystems_all(FLIGHT_CRITICAL_SUBSYSTEMS, &INIT_DONE)
}
pub fn is_init_any_failed() -> bool {
    check_subsystems_any(FLIGHT_CRITICAL_SUBSYSTEMS, &INIT_FAILED)
}
pub fn is_init_critical_failed() -> bool {
    check_subsystems_any(FLIGHT_CRITICAL_SUBSYSTEMS, &INIT_FAILED)
}
pub fn is_runtime_critical_failure() -> bool {
    check_subsystems_any(FLIGHT_CRITICAL_SUBSYSTEMS, &RUNTIME_FAILURES)
}
pub fn is_critical_failure() -> bool {
    is_init_critical_failed() || is_runtime_critical_failure()
}
// TODO: continue implementing error handling stuff

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    FLIGHT_STATE
        .sender()
        .send(FlightState::PreLaunch(state::GroundSubState::Startup));

    let p = embassy_rp::init(Default::default());
    let hw = take_hardware!(p);

    // Spawn core 1's executor first so it's ready to receive packets the
    // moment core 0 starts producing them.
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            spawn_core(
                EXECUTOR1.init(Executor::new()),
                core1_main(hw.sd, hw.uart, hw.indicators),
            )
        },
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

/// Core 0: everything fast and time-sensitive: sensors, AHRS, PID, servos, and the state machine.
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
        Ok(sensors) => {
            mark_init_complete(Subsystem::SENSORS);
            sensors
        }
        Err(e) => {
            error!("Error initializing sensors: {:?}", Debug2Format(&e));
            mark_init_failed(Subsystem::SENSORS);
            defmt::panic!("Failed to initialize sensors");
        }
    };
    info!("sensors initialized");

    info!("initializing system state");
    let mut system = match SystemState::new(sensors) {
        Ok(system) => {
            mark_init_complete(Subsystem::BASE_SYSTEM);
            system
        }
        Err(_) => {
            error!("Failed to initialize system state");
            mark_init_failed(Subsystem::BASE_SYSTEM);
            return;
        }
    };
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
        //  What I'm thinking now: Sensors and AHRS definitely need to be together (but I need to
        //  figure out how to handle slower sensors). PID and servos can be slower; it's probably
        //  fine to be something like 50Hz instead of 400Hz, and it doesn't need to be tightly
        //  integrated with sensors and AHRS. It can just read the AHRS data at any given moment.
        //  I'm not totally sure where the state machine should go, but I guess the sensor loop makes sense.
        system.tick().await;
    }
}

/// Core 1: everything slow or where timing is unimportant: GPS, SD writes, and indication.
#[embassy_executor::task]
async fn core1_main(
    sd_config: SdConfig,
    uart_config: UartConfig,
    indicators_config: IndicatorsConfig,
) {
    info!("core 1: I/O task starting");

    // TODO:
    //   • init SD card (SPI1 @ 50 MHz, embedded-sdmmc::VolumeManager)
    //   • init UART1 for GPS @ 9600 baud, send PMTK config

    embassy_futures::join::join3(
        indicator_loop(indicators_config),
        sd_logging_loop(sd_config),
        gps_loop(),
    )
    .await;
}

// TODO: All of these should be moved to their own files and actually implemented.
async fn sd_logging_loop(sd_config: SdConfig) {
    mark_init_complete(Subsystem::SD_CARD);
    let mut ticker: Ticker = Ticker::every(Duration::from_hz(20));
    loop {
        ticker.next().await;
    }
}

async fn gps_loop() {
    mark_init_complete(Subsystem::GPS);
    let mut ticker: Ticker = Ticker::every(Duration::from_hz(5)); // TODO: should this actually be 5 Hz? What happens if it's slightly off from the GPS clock?
    loop {
        ticker.next().await;
    }
}
