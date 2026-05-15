use crate::utils::SystemState;
use defmt::*;

pub struct StateMachine {
    state: SystemState,
}

impl StateMachine {
    pub fn new() -> Self {
        Self {
            state: SystemState::Starting,
        }
    }

    pub fn set_state(&mut self, new_state: SystemState) {
        info!("State change: {:?} -> {:?}", self.state, new_state);
        self.state = new_state;
    }

    pub fn get_state(&self) -> SystemState {
        self.state
    }

    pub async fn handle(&mut self) {
        match self.state {
            SystemState::Starting => {
                // Init logic
            }
            SystemState::ReadyToLaunch => {
                // Wait for launch
            }
            SystemState::Ascent => {
                // Control logic
            }
            _ => {}
        }
    }
}
