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
use crate::{state, Irqs, FLIGHT_STATE, READY};
use crate::config::board::{Neopixel, NUM_LEDS};
use crate::sensors::Sensors;
use crate::types::SensorReadings;
use crate::orientation::ahrs;
use crate::orientation::ahrs::AhrsState;
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
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlightState {
    PreLaunch(GroundSubState),
    Ascent(AscentSubState),
    Recovery(RecoverySubState),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GroundSubState {
    Startup,
    ReadyToLaunch,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AscentSubState {
    Burn,
    Coast,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RecoverySubState {
    DrogueDeploy,
    MainDeploy,
    Landed,
}

pub enum LedColor {
    Red,
    Orange,
    Yellow,
    Green,
    Cyan,
    Blue,
    Magenta,
    Purple,
    // maybe useful?
    White,
    Off,
}

fn map_color(color: LedColor) -> Hsv {
    match color {
        LedColor::Off => Hsv { hue: 0, sat: 0, val: 0 },
        LedColor::Red => Hsv { hue: 0, sat: 255, val: 255 },
        LedColor::Orange => Hsv { hue: 14, sat: 255, val: 255 },
        LedColor::Yellow => Hsv { hue: 42, sat: 255, val: 255 },
        LedColor::Green => Hsv { hue: 85, sat: 255, val: 255 },
        LedColor::Cyan => Hsv { hue: 127, sat: 255, val: 255 },
        LedColor::Blue => Hsv { hue: 170, sat: 255, val: 255 },
        LedColor::Magenta => Hsv { hue: 212, sat: 255, val: 255 },
        LedColor::Purple => Hsv { hue: 255, sat: 255, val: 255 },
        LedColor::White => Hsv { hue: 0, sat: 0, val: 255 },
    }
}

pub struct StateIndicator {
    pub led: LedColor,
    pub buzzer: &'static [(Duration, bool)],
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

pub async fn indicator_loop(
    mut neopixel: Neopixel,
    mut buzzer: Output<'static>,
    mut receiver: Receiver<'static, CriticalSectionRawMutex, FlightState, 2>
) {
    // Every 20Hz, check the state and proceed with the according buzzer pattern.
    let mut ticker = Ticker::every(Duration::from_hz(20));
    let mut current_state = receiver.get().await;
    let mut entered_at = Instant::now();
    loop {
        ticker.next().await;
        let state_change = receiver.try_changed();
        match state_change {
            Some(new_state) => { // state changed
                current_state = new_state;
                entered_at = Instant::now();
            }
            None => {}
        }
        let config = current_state.indicator();

        if state_change.is_some() { // no need to update the LED if the state hasn't changed
            let color = map_color(config.led);
            set_neopixel_color(&mut neopixel, color, 0.3).await;
        }

        let pattern = config.buzzer;
        drive_buzzer(&mut buzzer, pattern, Instant::now() - entered_at);
    }
}

fn drive_buzzer(buzzer: &mut Output, pattern: &[(Duration, bool)], elapsed: Duration) {
    let total: Duration = pattern.iter().map(|(d, _)| *d).sum();
    if total.as_ticks() == 0 {
        buzzer.set_low();
        return;
    }
    let mut phase = elapsed.as_ticks() % total.as_ticks();
    let on = pattern.iter()
        .find_map(|(d, on)| if phase < d.as_ticks() { Some(*on) } else { phase -= d.as_ticks(); None })
        .unwrap_or(false);
    buzzer.set_level(on.into());
}

impl<'a, I2C: embedded_hal::i2c::I2c> SystemState<'a, I2C> {
    pub fn new(sensors: Sensors<I2C>) -> Result<Self, ()> {
        let state_receiver_option = FLIGHT_STATE.receiver();
        if state_receiver_option.is_none() { // TODO: switch to match
            error!("Failed to get flight state receiver; have too many receivers been initialized?");
            return Err(())
        }
        let state = state_receiver_option.unwrap();
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
        let now = Instant::now();
        let tick_time = now - self.last_tick;
        self.last_tick = now;

        let sensor_data = self.sensors.read_all().await;
        let gyro = sensor_data.lsm.gyro;
        let accel = sensor_data.merged_accel();
        let mag = sensor_data.lis3.mag;

        self.ahrs.update(gyro, accel, mag, now);
        self.stream_telemetry(&sensor_data);
        self.log_to_flash(&sensor_data);

        match self.state.get().await {
            // Everything that occurs on the ground prior to launch.
            FlightState::PreLaunch(sub) => match sub {
                GroundSubState::Startup => {
                    if READY.signaled() { // TODO: should probably also check if AHRS ready
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

    // Stub methods for demonstration
    fn stream_telemetry(&self, _: &SensorReadings) {}
    fn log_to_flash(&self, _: &SensorReadings) {}
    fn update_gps(&self) {}
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

async fn set_neopixel_color(
    neopixel: &mut Neopixel,
    color_hsv: Hsv,
    brightness: f32,
) {
    let color_hsv_dimmed = Hsv {
        hue: color_hsv.hue,
        sat: color_hsv.sat,
        val: (brightness * color_hsv.val as f32) as u8,
    };
    let data = [hsv2rgb(color_hsv_dimmed); NUM_LEDS];
    neopixel.write(&data).await;
}

async fn set_neopixel_color_rgb(
    neopixel: &mut Neopixel,
    color_rgb: RGB8,
    brightness: f32,
) {
    let color_rgb_dimmed = RGB8 {
        r: (brightness * color_rgb.r as f32) as u8,
        g: (brightness * color_rgb.g as f32) as u8,
        b: (brightness * color_rgb.b as f32) as u8,
    };
    let data = [color_rgb_dimmed; NUM_LEDS];
    neopixel.write(&data).await;
}
