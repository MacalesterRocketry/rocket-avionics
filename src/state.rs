//! State machine, ported from `states.cpp`. Skeleton.
//!
//! In the C++ version this was a single `handleState()` switch called from
//! `loop()`. Here we split it into an Embassy task that wakes on each sensor
//! sample (driven by `embassy_time::Ticker`) plus an indicator subtask that
//! drives the NeoPixel + buzzer. See PORTING_PLAN.md "state machine" section.

#![allow(dead_code, unused_variables)]

use defmt::{error, info};
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812;
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program, RgbColorOrder};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Receiver;
use embassy_time::{Duration, Instant, Ticker, Timer};
use smart_leds::hsv::{hsv2rgb, Hsv};
use smart_leds::RGB8;
use uom::si::reciprocal_length::reciprocal_centimeter;
use crate::config::HAS_DROGUE_CHUTE;
use crate::math::Deg;
use crate::{is_critical_failure, is_init_all_complete, is_init_critical_complete, mark_init_complete, mark_init_failed, state, Irqs, Subsystem, FLIGHT_STATE};
use crate::config::board::{IndicatorsConfig, Neopixel, NUM_LEDS};
use crate::errors::handle_unrecoverable_error;
use crate::sensors::Sensors;
use crate::types::SensorReadings;
use crate::orientation::ahrs;
use crate::orientation::ahrs::AhrsState;
use crate::output::indication::{LedColor, StateIndicator};
use crate::output::roll_controller;
use crate::output::roll_controller::RollPid;

pub struct SystemState<'a, I2C: embedded_hal::i2c::I2c> {
    pub state: Receiver<'a, CriticalSectionRawMutex, FlightState, 2>,
    pub ahrs: AhrsState,
    pub roll_pid: RollPid,
    pub sensors: Sensors<I2C>,
    pub(crate) ignition_time: Option<Instant>,
    pub(crate) last_tick: Instant,
    // pub last_event: Option<EventType>, // who knows, these last two are just ideas about what might be interesting to have
    // pub error_flags: ErrorFlags,
}
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum FlightState {
    PreLaunch(GroundSubState),
    Ascent(AscentSubState),
    Recovery(RecoverySubState),
}

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum GroundSubState {
    Startup,
    ReadyToLaunch,
}

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum AscentSubState {
    Burn,
    Coast,
}

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum RecoverySubState {
    DrogueDeploy,
    MainDeploy,
    Landed,
}

impl FlightState {
    const SILENT: &'static [(Duration, bool)] = &[];
    const ONE_PER_16S: &'static [(Duration, bool)] = &[
        (Duration::from_millis(100), true),
        (Duration::from_secs(16), false), // TODO: make some macro of something like buzzer_pattern!(beeps, cycle_duration) where beeps is a tuple of (beep_duration, beep_on)
        // TODO: Maybe also make a similar macro for a series of beeps, like with parameters of num beeps, beep length, and time between beeps?
    ];
    pub fn indicator(&self) -> StateIndicator {
        // TODO: should probably add GPS lock check here
        match &self {
            FlightState::PreLaunch(GroundSubState::Startup) => StateIndicator {
                led: LedColor::Blue,
                buzzer: FlightState::SILENT,
            },
            FlightState::PreLaunch(GroundSubState::ReadyToLaunch) => StateIndicator {
                led: LedColor::Green,
                buzzer: FlightState::ONE_PER_16S, // one per 16 seconds
            },
            FlightState::Ascent(AscentSubState::Burn) => StateIndicator {
                led: LedColor::Purple,
                buzzer: FlightState::SILENT,
            },
            FlightState::Recovery(RecoverySubState::Landed) => StateIndicator {
                led: LedColor::Cyan,
                buzzer: FlightState::SILENT,
            },
            // TODO: handle errors somehow (fatal: red, non-fatal: orange)
            _ => StateIndicator {
                led: LedColor::Off,
                buzzer: FlightState::SILENT,
            },
        }
    }
}

impl<'a, I2C: embedded_hal::i2c::I2c> SystemState<'a, I2C> {
    pub fn new(sensors: Sensors<I2C>) -> Result<Self, ()> {
        let state = match FLIGHT_STATE.receiver() {
            Some(receiver) => receiver,
            None => {
                error!("Failed to get flight state receiver; have too many receivers been initialized?");
                return Err(())
            }
        };
        Ok(Self {
            state,
            ahrs: AhrsState::default(),
            roll_pid: RollPid::default(),
            sensors,
            ignition_time: None,
            last_tick: Instant::now(),
        })
    }

    /// The main loop tick
    pub async fn tick(&mut self) {
        if is_critical_failure() {
            error!("Critical failure detected; shutting down");
            // TODO: close SD card, etc.
            handle_unrecoverable_error()
        }
        let now = Instant::now();
        let tick_time = now - self.last_tick;
        self.last_tick = now;

        let sensor_data = self.sensors.read_all().await;
        let gyro = sensor_data.lsm.gyro;
        let accel = sensor_data.merged_accel();
        let mag = sensor_data.lis3.mag;

        self.ahrs.update(gyro, accel, mag, now);

        match self.state.get().await {
            // Everything that occurs on the ground prior to launch.
            FlightState::PreLaunch(sub) => match sub {
                GroundSubState::Startup => {
                    if is_init_all_complete() { // TODO: AHRS should probably signal if it's ready too
                        self.transition_to(FlightState::PreLaunch(GroundSubState::ReadyToLaunch));
                    }
                }
                GroundSubState::ReadyToLaunch => {
                    if sensor_data.has_launched() {
                        self.transition_to(FlightState::Ascent(AscentSubState::Burn));
                    }
                }
            },

            FlightState::Ascent(sub) => {
                if self.ignition_time.is_none() { // shouldn't ever happen because of our transition function, but just in case
                    self.ignition_time = Some(now);
                }
                let time_since_ignition = now - self.ignition_time.unwrap_or(now);
                self.roll_pid.step(roll_program(time_since_ignition), self.ahrs, tick_time);
                match sub {
                    AscentSubState::Burn => {
                        // TODO: test; if acceleration is negative in z and it's already off the rail (velocity is somewhat high), switch to Coast
                        if self.ahrs.acceleration_earth.z < -0.01 && self.ahrs.velocity_earth.z > 20.0 {
                            self.transition_to(FlightState::Ascent(AscentSubState::Coast));
                        }
                    }
                    AscentSubState::Coast => {
                        // TODO: is apogee enough? do I need to do something more precise to detect drogue?
                        if self.ahrs.velocity_earth.z < 0.0 { // Apogee detected
                            if HAS_DROGUE_CHUTE {
                                self.transition_to(FlightState::Recovery(RecoverySubState::DrogueDeploy));
                            } else {
                                self.transition_to(FlightState::Recovery(RecoverySubState::MainDeploy))
                            }
                        }
                    }
                }
            }

            FlightState::Recovery(sub) => {
                match sub {
                    RecoverySubState::DrogueDeploy => {}
                    RecoverySubState::MainDeploy => {}
                    RecoverySubState::Landed => {}
                }
            }
        }
    }

    fn transition_to(&mut self, next_state: FlightState) {
        // TODO: log
        if next_state == FlightState::Ascent(AscentSubState::Burn) {
            self.ahrs.launch();
            self.roll_pid.launch(self.ahrs);
            self.ignition_time = Some(Instant::now());
        }
        if next_state == FlightState::Recovery(RecoverySubState::Landed) {
            self.ahrs.landing();
        }

        FLIGHT_STATE.sender().send(next_state);
    }
}

/// Pre-programmed roll command: returns target roll angle (deg) as a function
/// of time since ignition. Currently just a stub matching `rollProgram()` in
/// states.cpp.
pub fn roll_program(time_since_ignition: Duration) -> Deg {
    let time_s = time_since_ignition.as_micros() as f64 * 1e-6;
    if (0.0..3.0).contains(&time_s) {
        0.0
    } else if (3.0..6.0).contains(&time_s) {
        90.0
    } else {
        0.0
    }
}

