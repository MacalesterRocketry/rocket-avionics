use embassy_rp::gpio::Output;
use embassy_time::{Duration, Timer};

pub struct Buzzer<'d> {
    pin: Output<'d>,
}

impl<'d> Buzzer<'d> {
    pub fn new(pin: Output<'d>) -> Self {
        Self { pin }
    }

    pub async fn beep(&mut self, duration: Duration) {
        self.pin.set_high();
        Timer::after(duration).await;
        self.pin.set_low();
    }

    pub async fn run_pattern(&mut self, duration: Duration, interval: Duration) {
        self.pin.set_high();
        Timer::after(duration).await;
        self.pin.set_low();
        Timer::after(interval).await;
    }
}
