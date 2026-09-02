use core::sync::atomic::{AtomicU8, Ordering};
use defmt::{Format, bitflags, error, info};

bitflags! {
    pub struct Subsystem: u8 {
        const BASE_SYSTEM = 1 << 0; // core 0
        const SENSORS     = 1 << 1;
        const CONTROL     = 1 << 2;

        const INDICATORS  = 1 << 5; // core 1
        const SD_CARD     = 1 << 6;
        const GPS         = 1 << 7;
    }
}

pub static INIT_DONE: AtomicU8 = AtomicU8::new(0);
pub static INIT_FAILURES: AtomicU8 = AtomicU8::new(0);
pub static RUNTIME_FAILURES: AtomicU8 = AtomicU8::new(0);

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
fn check_subsystems_any(subsystems: &[Subsystem], subsystem_flags: &AtomicU8) -> bool {
    subsystems
        .iter()
        .any(|s| {
            check_subsystem(*s, subsystem_flags)
        })
}
fn check_subsystems_all(subsystems: &[Subsystem], subsystem_flags: &AtomicU8) -> bool {
    subsystems
        .iter()
        .all(|s| {
            check_subsystem(*s, subsystem_flags)
        })
}
fn check_subsystem(subsystem: Subsystem, subsystem_flags: &AtomicU8) -> bool {
    Subsystem::from_bits_truncate(subsystem_flags.load(Ordering::Acquire)).contains(subsystem)
}
fn set_flag(subsystem_flags: &AtomicU8, subsystem: Subsystem) {
    subsystem_flags.fetch_or(subsystem.bits(), Ordering::Release);
}
fn clear_flag(subsystem_flags: &AtomicU8, subsystem: Subsystem) {
    subsystem_flags.fetch_and(!subsystem.bits(), Ordering::Release);
}
fn any_subsystem(subsystem_flags: &AtomicU8) -> bool {
    Subsystem::from_bits_truncate(subsystem_flags.load(Ordering::Acquire)) != Subsystem::empty()
}
fn all_subsystems(subsystem_flags: &AtomicU8) -> bool {
    Subsystem::from_bits_truncate(subsystem_flags.load(Ordering::Acquire)) == Subsystem::all()
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
pub fn mark_init_complete(subsystem: Subsystem) {
    info!("subsystem {} initialized", subsystem);
    set_flag(&INIT_DONE, subsystem)
}
pub fn is_init_all_complete() -> bool {
    all_subsystems(&INIT_DONE)
}
/// Clear `subsystem`'s runtime-failure bit, announcing that it is working again.
pub fn clear_runtime_error(subsystem: Subsystem) {
    if has_runtime_error(subsystem) {
        info!("subsystem {} working again, clearing runtime error", subsystem);
    }
    clear_flag(&RUNTIME_FAILURES, subsystem);
}
pub fn has_runtime_error(subsystem: Subsystem) -> bool {
    check_subsystem(subsystem, &RUNTIME_FAILURES)
}

// ---------------------------------------------------------------------------
// Subsystem error reporting
// ---------------------------------------------------------------------------

/// An error that can be attributed to exactly one [`Subsystem`].
///
/// Implement this for each subsystem's error enum. Call sites then report
/// failures through [`report_init_error`] / [`report_runtime_error`] using this error type.
pub trait SubsystemError: Format {
    fn subsystem(&self) -> Subsystem;
}

/// Log `err` and mark its subsystem as having failed to initialize.
pub fn report_init_error<E: SubsystemError>(err: E) {
    let subsystem = err.subsystem();
    error!("init error: subsystem {} failed to initialize", subsystem);
    set_flag(&INIT_FAILURES, subsystem)
}

/// Log `err` and set its subsystem's runtime-failure bit. This latches the bit, so
/// use [`clear_runtime_error`] once the subsystem recovers.
pub fn report_runtime_error<E: SubsystemError>(err: E) {
    // TODO: This should probably send a message to the SD card.
    let subsystem = err.subsystem();
    error!("runtime error: subsystem {} failed at runtime", subsystem);
    set_flag(&RUNTIME_FAILURES, subsystem)
}