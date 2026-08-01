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

use crate::config::board::{
    AvionicsHardware, I2cConfig, InterruptConfig, NeopixelConfig, PeripheralConfig, SdConfig,
    UartConfig,
};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output};
use embassy_rp::i2c;
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::{bind_interrupts, dma, pio, pio_programs, Peri, Peripherals};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::i2c::I2c;
use smart_leds::hsv::{hsv2rgb, Hsv};
use smart_leds::{RGB8, RGBA};
use {defmt_rtt as _, panic_probe as _};
use crate::config::G;
use crate::state::SystemState;

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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let hw = take_hardware!(p);

    // Spawn core 1's executor first so it's ready to receive packets the
    // moment core 0 starts producing them.
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || spawn_core(EXECUTOR1.init(Executor::new()), core1_main(hw.sd, hw.uart)),
    );

    spawn_core(
        EXECUTOR0.init(Executor::new()),
        core0_main(hw.i2c, hw.interrupts, hw.peripherals, hw.neopixel),
    );
}

fn spawn_core(
    executor: &'static mut Executor,
    core_main: Result<SpawnToken<impl Sized>, SpawnError>,
) -> ! {
    executor.run(|spawner| {
        let core2_task = core_main.unwrap_or_else(|e| {
            error!("core 1: failed to spawn task: {:?}", e);
            defmt::panic!("core 1 task spawn failed");
        });

        spawner.spawn(core2_task);
    });
}

/// Core 0: the avionics hot loop. Skeleton — tasks below are TODOs.
#[embassy_executor::task]
async fn core0_main(
    i2c_config: I2cConfig,
    interrupt_config: InterruptConfig,
    peripheral_config: PeripheralConfig,
    neopixel_config: NeopixelConfig,
) {
    info!("core 0: avionics task starting");

    info!("initializing I²C bus");
    let mut config = i2c::Config::default();
    config.frequency = 400_000;
    let mut i2c = i2c::I2c::new_async(i2c_config.bus, i2c_config.scl, i2c_config.sda, Irqs, config);
    // i2c.write(0x76u8, &[1, 2, 3]).await.unwrap();
    info!("I²C bus initialized");

    info!("initializing GPIO");
    let mut buzzer = Output::new(peripheral_config.buzzer, embassy_rp::gpio::Level::Low);
    buzzer.set_low();
    let mut eject_button = Input::new(peripheral_config.eject_button, embassy_rp::gpio::Pull::Up);

    info!("initializing NeoPixel");
    let Pio {
        mut common, sm0, ..
    } = Pio::new(neopixel_config.pio, Irqs);
    let program = PioWs2812Program::new(&mut common);
    let neopixel = PioWs2812::new(
                &mut common,
                sm0,
                neopixel_config.channel,
                Irqs,
                neopixel_config.pin,
                &program,
            );
    info!("NeoPixel initialized");

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
    let mut system = SystemState::new(neopixel, sensors);
    info!("system state initialized");

    // TODO (in this order, matching original `setup()`):
    //   1. init servo PWMs
    //   2. start AHRS state (record start time)
    //   3. transition state machine: Starting -> ReadyToLaunch
    loop {
        system.tick().await;
        Timer::after_millis(5).await;
    }
}

/// Core 1: I/O background. Owns SD card + GPS UART + indicators.
#[embassy_executor::task]
async fn core1_main(sd_config: SdConfig, uart_config: UartConfig) {
    info!("core 1: I/O task starting");
    // TODO:
    //   • init SD card (SPI1 @ 50 MHz, embedded-sdmmc::VolumeManager)
    //   • init UART1 for GPS @ 9600 baud, send PMTK config
    //   • spawn `sd_writer_task`, `gps_task`, `indicator_task`
    loop {
        Timer::after_millis(1000).await;
    }
}
