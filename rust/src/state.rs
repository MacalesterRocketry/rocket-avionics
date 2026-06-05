//! State machine, ported from `states.cpp`. Skeleton.
//!
//! In the C++ version this was a single `handleState()` switch called from
//! `loop()`. Here we split it into an Embassy task that wakes on each sensor
//! sample (driven by `embassy_time::Ticker`) plus an indicator subtask that
//! drives the NeoPixel + buzzer. See PORTING_PLAN.md "state machine" section.

#![allow(dead_code, unused_variables)]

use crate::types::SystemState;

/// Global rocket state. Read-mostly; updated only by the supervisor task.
/// Backed by an `embassy_sync::Signal<SystemState>` so other tasks can await
/// state transitions without polling.
pub struct StateMachine {
    pub current: SystemState,
    pub ignition_time_us: u64,
}

impl StateMachine {
    pub const fn new() -> Self {
        Self { current: SystemState::Starting, ignition_time_us: 0 }
    }

    /// Transition to a new state. Emits a `PayloadEvent` log row on entry.
    pub fn set(&mut self, _next: SystemState) {
        // TODO: log transition (`logEvent`), then run any on-entry side effects.
    }
}

/// Indicator policy. Maps state → NeoPixel color + buzzer cadence. Port of
/// `indicateState()` from states.cpp. TODO.
pub fn indicate(_state: SystemState, _has_gps_fix: bool) {
    // TODO: drive NeoPixel + buzzer pin based on state + GPS lock.
}

/// Pre-programmed roll command: returns target roll angle (deg) as a function
/// of time since ignition. Currently just a stub matching `rollProgram()` in
/// states.cpp.
pub fn roll_program(time_since_ignition_s: f64) -> f64 {
    if (0.0..3.0).contains(&time_since_ignition_s) {
        0.0
    } else if (3.0..6.0).contains(&time_since_ignition_s) {
        90.0
    } else {
        0.0
    }
}
