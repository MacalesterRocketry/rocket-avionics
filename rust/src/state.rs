//! State machine, ported from `states.cpp`. Skeleton.
//!
//! In the C++ version this was a single `handleState()` switch called from
//! `loop()`. Here we split it into an Embassy task that wakes on each sensor
//! sample (driven by `embassy_time::Ticker`) plus an indicator subtask that
//! drives the NeoPixel + buzzer. See PORTING_PLAN.md "state machine" section.

#![allow(dead_code, unused_variables)]

use defmt::info;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812;
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program, RgbColorOrder};
use embassy_time::{Duration, Instant};
use smart_leds::hsv::{hsv2rgb, Hsv};
use smart_leds::RGB8;
use crate::config::{ACCELEROMETER_SWITCH_THRESHOLD, HAS_DROGUE_CHUTE};
use crate::math::{Deg, Vec3};
use crate::{sensors, Irqs};
use crate::config::board::{NeopixelColorOrder, NeopixelConfig, NUM_LEDS};
use crate::types::SensorReadings;
use crate::orientation::ahrs;
use crate::orientation::ahrs::AhrsState;
use crate::output::roll_controller;
use crate::output::roll_controller::RollPid;

pub struct SystemState<'a, PioInstance: pio::Instance, ColorOrder: RgbColorOrder> {
    pub state: FlightState,
    pub ahrs: AhrsState,
    pub roll_pid: RollPid,
    pub neopixel: PioWs2812<'a, PioInstance, 0, { NUM_LEDS }, ColorOrder>,
    pub(crate) ignition_time: Option<Instant>,
    pub(crate) last_tick: Instant,
    // pub last_event: Option<EventType>, // who knows, these last three are just ideas about what might be interesting to have
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

pub enum BeepCode {
    Off,
    // TODO: add support
}

pub struct StateIndicator {
    pub led: LedColor,
    pub buzzer: BeepCode,
}

impl<'a, PioInstance: pio::Instance, ColorOrder: RgbColorOrder> SystemState<'a, PioInstance, ColorOrder> {
    pub fn new(neopixel: PioWs2812<'a, PioInstance, 0, { NUM_LEDS }, ColorOrder>) -> Self {
        Self {
            state: FlightState::PreLaunch(GroundSubState::Startup),
            ahrs: AhrsState::default(),
            roll_pid: RollPid::default(),
            neopixel,
            ignition_time: None,
            last_tick: Instant::now(),
        }
    }

    /// The main loop tick
    pub async fn tick(&mut self) {
        let now = Instant::now();
        let tick_time = now - self.last_tick;
        self.last_tick = now;

        let sensor_data = sensors::read_all().await;
        let gyro = sensor_data.lsm.gyro;
        let mut accel: Vec3 = sensor_data.lsm.accel;
        if accel.mag() >= ACCELEROMETER_SWITCH_THRESHOLD { // If the low-G accelerometer is saturated, switch to high-G readings for AHRS
            accel = sensor_data.adxl.highg_accel;
        }
        let mag = sensor_data.lis3.mag;

        self.ahrs.update(gyro, accel, mag, now);
        self.stream_telemetry(&sensor_data);
        self.log_to_flash(&sensor_data);
        self.update_hardware_indicators().await;

        match &self.state {
            // Everything that occurs on the ground prior to launch.
            FlightState::PreLaunch(sub) => match sub {
                GroundSubState::Startup => {
                    // TODO: startup sequence, but how to do it async and non-blocking? Maybe have it in main and just check in here if it's all done?
                    self.transition_to(FlightState::PreLaunch(GroundSubState::ReadyToLaunch));
                }
                GroundSubState::ReadyToLaunch => {
                    if sensors::has_launched(sensor_data.adxl.highg_accel.mag()) {
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
        self.state = next_state;
    }

    pub fn indicator(&self) -> StateIndicator {
        // TODO: should probably add GPS lock check here
        match &self.state {
            FlightState::PreLaunch(GroundSubState::Startup) => StateIndicator {
                led: LedColor::Blue,
                buzzer: BeepCode::Off,
            },
            FlightState::PreLaunch(GroundSubState::ReadyToLaunch) => StateIndicator {
                led: LedColor::Green,
                buzzer: BeepCode::Off,
            },
            FlightState::Ascent(AscentSubState::Burn) => StateIndicator {
                led: LedColor::Purple,
                buzzer: BeepCode::Off,
            },
            FlightState::Recovery(RecoverySubState::Landed) => StateIndicator {
                led: LedColor::Cyan,
                buzzer: BeepCode::Off,
            },
            // TODO: handle errors somehow (fatal: red, non-fatal: orange)
            _ => StateIndicator {
                led: LedColor::Off,
                buzzer: BeepCode::Off,
            },
        }
    }

    async fn update_hardware_indicators(&mut self) {
        let config = self.indicator();
        set_neopixel_color(&mut self.neopixel, map_color(config.led), 0.3).await;
        // TODO: implement buzzer
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

async fn set_neopixel_color<PioInstance: pio::Instance, ColorOrder: RgbColorOrder>(
    neopixel: &mut PioWs2812<'_, PioInstance, 0, 1, ColorOrder>,
    color_hsv: Hsv,
    brightness: f32,
) {
    let color_hsv_dimmed = Hsv {
        hue: color_hsv.hue,
        sat: color_hsv.sat,
        val: (brightness * color_hsv.val as f32) as u8,
    };
    let data = [hsv2rgb(color_hsv_dimmed); 1];
    neopixel.write(&data).await;
}

async fn set_neopixel_color_rgb<PioInstance: pio::Instance, ColorOrder: RgbColorOrder>(
    neopixel: &mut PioWs2812<'_, PioInstance, 0, 1, ColorOrder>,
    color_rgb: RGB8,
    brightness: f32,
) {
    let color_rgb_dimmed = RGB8 {
        r: (brightness * color_rgb.r as f32) as u8,
        g: (brightness * color_rgb.g as f32) as u8,
        b: (brightness * color_rgb.b as f32) as u8,
    };
    let data = [color_rgb_dimmed; 1];
    neopixel.write(&data).await;
}
