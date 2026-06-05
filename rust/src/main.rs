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

use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1, Stack};
use static_cell::StaticCell;

mod ahrs;
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
// #[link_section = ".core1_stack"]
static mut CORE1_STACK: Stack<4096> = Stack::new();

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    // Spawn core 1's executor first so it's ready to receive packets the
    // moment core 0 starts producing them.
    spawn_core1(p.CORE1, unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| {
            spawner.spawn(core1_main()).ok();
        });
    });

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(core0_main()).ok();
    });
}

/// Core 0: the avionics hot loop. Skeleton — tasks below are TODOs.
#[embassy_executor::task]
async fn core0_main() {
    defmt::info!("core 0: avionics task starting");
    // TODO (in this order, matching original `setup()`):
    //   1. init I²C bus (Wire), set 400 kHz fast mode
    //   2. init NeoPixel, buzzer, eject-button GPIO
    //   3. init all four sensors (sensors::init_all)
    //   4. init servo PWMs
    //   5. start AHRS state (record start time)
    //   6. transition state machine: Starting -> ReadyToLaunch
    //   7. enter sample/update/control ticker @ ~200 Hz
    loop {
        embassy_time::Timer::after_millis(1000).await;
    }
}

/// Core 1: I/O background. Owns SD card + GPS UART + indicators.
#[embassy_executor::task]
async fn core1_main() {
    defmt::info!("core 1: I/O task starting");
    // TODO:
    //   • init SD card (SPI1 @ 50 MHz, embedded-sdmmc::VolumeManager)
    //   • init UART1 for GPS @ 9600 baud, send PMTK config
    //   • spawn `sd_writer_task`, `gps_task`, `indicator_task`
    loop {
        embassy_time::Timer::after_millis(1000).await;
    }
}
