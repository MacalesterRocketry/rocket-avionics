pub struct Gps;

impl Gps {
    pub fn new() -> Self {
        Self
    }

    pub async fn init(&mut self) -> Result<(), ()> {
        Ok(())
    }

    pub fn read(&mut self) {
        // GPS reading logic
    }
}
