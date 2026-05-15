#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

mod config;
mod utils;
mod orientation;
mod output;
mod states;

use crate::orientation::sensors::Sensors;
use crate::orientation::ahrs::Ahrs;
use crate::output::roll_controller::RollController;
use crate::states::StateMachine;
use embassy_rp::i2c::{I2c, Config as I2cConfig};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::I2C0;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Rocket Avionics Initializing...");

    // 1. Initialize I2C
    let i2c = I2c::new_async(p.I2C0, p.PIN_1, p.PIN_0, Irqs, I2cConfig::default());
    let mut sensors = Sensors::new(i2c);
    
    // 2. Initialize AHRS and Control
    let mut ahrs = Ahrs::new();
    let mut roll_controller = RollController::new();
    let mut state_machine = StateMachine::new();

    info!("System Ready.");

    let mut last_time = embassy_time::Instant::now();

    loop {
        // 1. Read Sensors
        let readings = sensors.read_all().await;

        // 2. Update AHRS
        let now = embassy_time::Instant::now();
        let dt = now.duration_since(last_time).as_micros() as f64 / 1_000_000.0;
        last_time = now;

        let in_flight = state_machine.get_state() == utils::SystemState::Ascent;
        ahrs.update(&readings, dt, in_flight);

        // 3. Handle State Machine
        state_machine.handle().await;

        // 4. Update Control
        if in_flight {
            let _deflection = roll_controller.update(&ahrs, 0.0, dt);
            // Command servos here
        }

        Timer::after(Duration::from_millis(10)).await;
    }
}
