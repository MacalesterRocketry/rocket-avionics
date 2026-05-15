pub struct SdCard;

impl SdCard {
    pub fn new() -> Self {
        Self
    }

    pub async fn init(&mut self) -> Result<(), ()> {
        Ok(())
    }

    pub fn log(&mut self, _data: &str) {
        // Logging logic
    }
}
