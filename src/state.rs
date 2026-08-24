//! State machine, ported from `states.cpp`. Skeleton.
//!
//! In the C++ version this was a single `handleState()` switch called from
//! `loop()`. Here we split it into an Embassy task that wakes on each sensor
//! sample (driven by `embassy_time::Ticker`) plus an indicator subtask that
//! drives the NeoPixel + buzzer. See PORTING_PLAN.md "state machine" section.

#![allow(dead_code, unused_variables)]

use crate::communication::indication::{BeepCycle, BeepSequence, LedColor, StateIndicator};
use crate::config::HAS_DROGUE_CHUTE;
use crate::config::board::{I2cConfig, IndicatorsConfig, NUM_LEDS, Neopixel, PeripheralConfig};
use crate::navigation::ahrs;
use crate::navigation::ahrs::{AHRS_STATE, AhrsState};
use crate::navigation::gps::{GPS_STATE, GpsState};
use crate::sensors::SensorReadings;
use crate::sensors::Sensors;
use crate::utils::errors::handle_unrecoverable_error;
use crate::utils::math::{Deg, Quat, roll_deg_to_quat};
use crate::{FLIGHT_STATE, Irqs, Subsystem, is_critical_failure, is_init_all_complete, is_init_critical_complete, mark_init_complete, mark_init_failed, sensors};
use defmt::{Debug2Format, error, info};
use embassy_rp::gpio::{Input, Output};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812;
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program, RgbColorOrder};
use embassy_rp::{i2c, pio};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{Receiver, Watch};
use embassy_time::{Duration, Instant, Ticker, Timer};
use smart_leds::RGB8;
use smart_leds::hsv::{Hsv, hsv2rgb};
use uom::si::reciprocal_length::reciprocal_centimeter;

/// What the control loop should be doing. The state machine owns the mission
/// timeline, so it decides the attitude to hold; the control loop only knows how
/// to get there. Publishing an absolute attitude rather than a roll offset keeps
/// the ignition-time reference on this side and generalizes to 3-axis unchanged.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlSetpoint {
    /// Fins centered, PID held in reset.
    Disarmed,
    /// Drive the airframe to this absolute attitude.
    Attitude(Quat),
}

/// Latest control setpoint. A `Watch` because the control loop wants whatever is
/// current when its own ticker fires — a queue would let setpoints pile up and
/// be actuated late.
pub static CONTROL_SETPOINT: Watch<CriticalSectionRawMutex, ControlSetpoint, 2> = Watch::new();

pub struct SystemState<'a, I2C: embedded_hal::i2c::I2c> {
    pub state: Receiver<'a, CriticalSectionRawMutex, FlightState, 2>,
    pub ahrs: AhrsState,
    pub gps: Receiver<'a, CriticalSectionRawMutex, GpsState, 3>,
    pub sensors: Sensors<I2C>,
    pub(crate) ignition_time: Option<Instant>,
    /// Orientation captured at ignition. Every roll command is relative to it,
    /// so it is the reference the published attitude setpoint is built from.
    pub(crate) launch_orientation: Quat,
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
    ReadyToLaunch(ReadyToLaunchSubState),
}

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum ReadyToLaunchSubState {
    WaitingForGPS,
    GPSLock,
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

fn standard_beep_cycle<'a>(count: u32, cycle_secs: u64) -> BeepCycle<'a> {
    BeepCycle::Pulse {
        count,
        on_time: Duration::from_millis(100),
        off_time: Duration::from_millis(100),
        cycle_duration: Duration::from_secs(cycle_secs),
    }
}

impl FlightState {
    /// Returns the appropriate state indicator for the current flight state.
    pub fn indicator(&self) -> StateIndicator {
        // TODO: should probably add GPS lock check here
        match &self {
            FlightState::PreLaunch(GroundSubState::Startup) => StateIndicator {
                led: LedColor::Blue,
                buzzer: BeepCycle::Silent,
            },
            FlightState::PreLaunch(GroundSubState::ReadyToLaunch(gps_state)) => match gps_state {
                ReadyToLaunchSubState::WaitingForGPS => StateIndicator {
                    led: LedColor::Green,
                    buzzer: standard_beep_cycle(1, 8), // TODO: should probably eventually rethink this (e.g. 2 beeps if GPS lock), but fine for now
                },
                ReadyToLaunchSubState::GPSLock => StateIndicator {
                    led: LedColor::Green,
                    buzzer: standard_beep_cycle(1, 16),
                },
            },
            FlightState::Ascent(AscentSubState::Burn) => StateIndicator {
                led: LedColor::Purple,
                buzzer: BeepCycle::Silent,
            },
            FlightState::Recovery(RecoverySubState::Landed) => StateIndicator {
                led: LedColor::Cyan,
                buzzer: BeepCycle::Silent,
            },
            // TODO: handle errors somehow (fatal: red, non-fatal: orange)
            _ => StateIndicator {
                led: LedColor::Off,
                buzzer: BeepCycle::Silent,
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
        let gps = match GPS_STATE.receiver() {
            Some(receiver) => receiver,
            None => {
                error!("Failed to get GPS state receiver; have too many receivers been initialized?");
                return Err(())
            }
        };
        Ok(Self {
            state,
            ahrs: AhrsState::default(),
            gps,
            sensors,
            ignition_time: None,
            launch_orientation: Quat::IDENTITY,
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
        AHRS_STATE.sender().send(self.ahrs);

        match self.state.get().await {
            // Everything that occurs on the ground prior to launch.
            FlightState::PreLaunch(sub) => {
                CONTROL_SETPOINT.sender().send(ControlSetpoint::Disarmed);
                match sub {
                GroundSubState::Startup => {
                    if is_init_all_complete() { // TODO: AHRS should probably signal if it's ready too
                        self.transition_to(FlightState::PreLaunch(GroundSubState::ReadyToLaunch(ReadyToLaunchSubState::WaitingForGPS)));
                    }
                }
                GroundSubState::ReadyToLaunch(gps_state) => {
                    if sensor_data.has_launched() {
                        self.transition_to(FlightState::Ascent(AscentSubState::Burn));
                    }
                    match gps_state {
                        ReadyToLaunchSubState::WaitingForGPS => {
                            if self.gps.get().await.has_fix {
                                self.transition_to(FlightState::PreLaunch(GroundSubState::ReadyToLaunch(ReadyToLaunchSubState::GPSLock)));
                            }
                        },
                        ReadyToLaunchSubState::GPSLock => {
                            if !self.gps.get().await.has_fix {
                                self.transition_to(FlightState::PreLaunch(GroundSubState::ReadyToLaunch(ReadyToLaunchSubState::WaitingForGPS)));
                            }
                        }
                    }
                }
            }},

            FlightState::Ascent(sub) => {
                if self.ignition_time.is_none() { // shouldn't ever happen because of our transition function, but just in case
                    self.ignition_time = Some(now);
                }
                let time_since_ignition = now - self.ignition_time.unwrap_or(now);
                CONTROL_SETPOINT.sender().send(ControlSetpoint::Attitude(
                    attitude_target(self.launch_orientation, roll_program(time_since_ignition)),
                ));
                match sub {
                    AscentSubState::Burn => {
                        // TODO: test;
                        // if acceleration is negative in z and it's already off the rail (velocity is somewhat high), switch to Coast
                        if self.ahrs.acceleration_earth.z < -0.01 && self.ahrs.velocity_earth.z > 20.0 {
                            self.transition_to(FlightState::Ascent(AscentSubState::Coast));
                        }
                    }
                    AscentSubState::Coast => {
                        // TODO: is apogee enough? do I need to do something more precise to detect drogue?
                        if self.ahrs.velocity_earth.z < 0.0 { // Apogee detected
                            if HAS_DROGUE_CHUTE { // TODO: How does Rust do things like this? Features?
                                self.transition_to(FlightState::Recovery(RecoverySubState::DrogueDeploy));
                            } else {
                                self.transition_to(FlightState::Recovery(RecoverySubState::MainDeploy))
                            }
                        }
                    }
                }
            }

            FlightState::Recovery(sub) => {
                CONTROL_SETPOINT.sender().send(ControlSetpoint::Disarmed);
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
        info!("transitioning to state {:?}", next_state);
        if next_state == FlightState::Ascent(AscentSubState::Burn) {
            self.ahrs.launch();
            // Captured here rather than in the control loop so the reference is
            // the attitude at the instant of ignition, not up to one control
            // period later.
            self.launch_orientation = self.ahrs.get_orientation_earth();
            self.ignition_time = Some(Instant::now());
        }
        if next_state == FlightState::Recovery(RecoverySubState::Landed) {
            self.ahrs.landing();
        }

        FLIGHT_STATE.sender().send(next_state);
    }
}

/// Absolute attitude to hold: the orientation captured at ignition, rolled by
/// the commanded angle. This is the guidance half of the control problem, and it
/// lives here because the mission timeline does.
fn attitude_target(launch_orientation: Quat, target_roll: Deg) -> Quat {
    let qroll = roll_deg_to_quat(-target_roll); // Negative for body frame
    // TODO: This probably has different pitch and yaw than what we actually want.
    //  Replace them with the current ones from qcurrent? Or just ignore them
    //  since we're only controlling roll?
    (launch_orientation * qroll).normalized()
}

/// Pre-programmed roll command: returns target roll angle (deg) as a function
/// of time since ignition. Currently just a stub matching `rollProgram()` in
/// states.cpp.
///
/// NOTE: this angle is in the same convention as `calculate_roll_deg`, which
/// flight data showed to be earth-frame — it tracked integrated gyro closely
/// through most of the flight but diverged sharply once the rocket tilted at
/// apogee.
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

pub async fn system_loop(i2c_config: I2cConfig, peripheral_config: PeripheralConfig) {
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
            defmt::panic!("Failed to initialize system state");
        }
    };
    // TODO: handle errors with Neopixel notifs and logging and stuff instead of panicking
    info!("system state initialized");

    // TODO: init servos and PID
    let mut ticker = Ticker::every(Duration::from_hz(250));
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
        //  Actually, it might be a problem to keep AHRS data atomic. I don't know.
        system.tick().await;
    }
}