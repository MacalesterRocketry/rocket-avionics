//! Compile-time configuration. Direct port of the C++ `config.h`.
//!
//! Pin names use generic `PIN_*` labels keyed by GPIO number so the user can
//! remap them when the RP2350 PCB layout is finalized. The hardware-revision
//! gate from the C++ source becomes a Cargo feature (`hw-v2` / `hw-v3`).

#![allow(dead_code)]

// ─────────────────────────────── debug knobs ────────────────────────────────
// In the C++ these are `#define X 0/1` preprocessor flags. Here they're plain
// `const bool` — the optimizer constant-folds them just like the preprocessor,
// and they're typesafe.
pub const DEBUG: bool = false;
pub const DEBUG_SD: bool = false;
pub const DEBUG_PRINT_SENSORS: bool = false;
pub const DEBUG_PRINT_ORIENTATION: bool = false;
pub const DEBUG_PRINT_ROLL_CONTROL: bool = false;

pub const PID_TUNING: bool = false;
pub const SERVO_TESTING: bool = false;

pub const PROFILING: bool = false;
pub const PROFILING_SAMPLES: usize = 100;

pub const SYNC_INTERVAL_MS: u64 = 5_000;

pub const USE_GPS: bool = true;
pub const USE_TURN_SIGNALS: bool = true;

// ───────────────────────────── physical constants ───────────────────────────
/// North Branch, MN local gravity (m/s²).
pub const G: f64 = 9.805_79;

// Magnetometer hard-iron offsets (µT). Calibrate per-flight.
pub const MAG_BIAS_X: f64 = -35.63;
pub const MAG_BIAS_Y: f64 = 19.96;
pub const MAG_BIAS_Z: f64 = -33.43;

// High-G (ADXL375) zero offsets. NOTE from C++ source: the ADXL375 may apply
// internal gravity correction — verify on bench before trusting these.
pub const HIGHG_BIAS_X: f64 = 9.8;
pub const HIGHG_BIAS_Y: f64 = 7.85;
pub const HIGHG_BIAS_Z: f64 = 9.8 - G;
pub const HIGHG_TRIM_X: i8 = -5; // raw LSB counts, not m/s²
pub const HIGHG_TRIM_Y: i8 = -4;
pub const HIGHG_TRIM_Z: i8 = 0;

// Low-G (LSM6DSOX accel) zero offsets.
pub const LOWG_BIAS_X: f64 = 0.05;
pub const LOWG_BIAS_Y: f64 = 0.03;
pub const LOWG_BIAS_Z: f64 = 10.03 - G;

// Gyro zero-rate offsets (rad/s).
pub const GYRO_BIAS_X: f64 = -0.01;
pub const GYRO_BIAS_Y: f64 = 0.0;
pub const GYRO_BIAS_Z: f64 = 0.0;

// ───────────────────────────── AHRS constants ───────────────────────────────
/// Madgwick β for accelerometer correction. Higher → trust accel more, faster
/// convergence, more noise.
pub const AHRS_ACC_BETA: f64 = 0.1;
/// Madgwick β for magnetometer correction. 0.0 disables it (current default).
pub const AHRS_MAG_BETA: f64 = 0.0;

// ────────────────────────────── PID constants ───────────────────────────────
/// Roll-axis moment of inertia (kg·m²).
pub const MOMENT_OF_INERTIA: f64 = 0.006_85;
/// CFD-derived torque at 10° fin deflection, 50 m/s airspeed (N·m).
pub const BASE_TORQUE: f64 = 0.429_709_735_258_359;
/// Linear approximation of torque per degree at 50 m/s (N·m/deg).
pub const TORQUE_PER_DEG_50MS: f64 = BASE_TORQUE / 10.0;
pub const ROLL_PID_KP: f64 = 68.1295;
pub const ROLL_PID_KI: f64 = 2.2519;
pub const ROLL_PID_KD: f64 = 20.8804;

// ─────────────────────────────── thresholds ────────────────────────────────
pub const LAUNCH_ACCEL_THRESHOLD_G: f64 = 4.0;
/// Mechanical sweep of the smaller servo, degrees.
pub const SERVO_DEGREE_RANGE: f64 = 100.0;
pub const SERVO_MAX_ANGLE: f64 = 50.0;
pub const SERVO_MIN_ANGLE: f64 = -50.0;
pub const SERVO_NEUTRAL_ANGLE: f64 = -3.0;
pub const SERVO_MICROS_MIN: u32 = 1000;
pub const SERVO_MICROS_MAX: u32 = 2000;
/// Datasheet stall torque at 7.4 V (N·m).
pub const SERVO_MAX_TORQUE: f64 = 0.51;

/// LSM6DSOX low-G accel saturates at 16 g; switch to ADXL375 above this.
pub const ACCELEROMETER_SWITCH_THRESHOLD: f64 = 15.9 * G;

// ──────────────────────────── battery divider ───────────────────────────────
pub const BATTERY_VOLTAGE_R1: f64 = 100_000.0;
pub const BATTERY_VOLTAGE_R2: f64 = 100_000.0;

// ───────────────────────── pin assignments (rev-gated) ──────────────────────
// The custom RP2350 PCB pinout isn't finalized; these placeholders keep the
// module structure consistent. Override per-revision under `cfg(feature)`.
// Use `Pin::PIN_NN` constants from embassy-rp when wiring these into peripheral
// init code — the integer here is the GPIO index.
#[cfg(feature = "hw-v2")]
pub mod pins {
    // RP2040 Adalogger custom FeatherWing (current flight hardware).
    pub const SD_CS: u8 = 23;
    pub const BUZZER: u8 = 28; // formerly A2
    pub const NEOPIXEL: u8 = 4;
    pub const EJECT_BUTTON: u8 = 24;
    pub const ADXL_INT1: u8 = 5;
    pub const ADXL_INT2: u8 = 6;
    pub const LSM_INT1: u8 = 9;
    pub const LSM_INT2: u8 = 10;
    pub const LIS3_INT1: u8 = 11;
    pub const LIS3_INT2: u8 = 12;
    pub const BMP_INT: u8 = 13;
    pub const SERVO_XPLUS: u8 = 25;
    pub const SERVO_XMINUS: u8 = 14;
    pub const SERVO_YPLUS: u8 = 15;
    pub const SERVO_YMINUS: u8 = 8;
    pub const BATTERY_VOLTAGE: u8 = 29; // formerly A3
    pub const TURN_SIGNAL_LEFT: u8 = 26; // formerly A0
    pub const TURN_SIGNAL_RIGHT: u8 = 27; // formerly A1
}

#[cfg(feature = "hw-v3")]
pub mod pins {
    // Custom RP2350 PCB — TODO: replace placeholders once the schematic is
    // finalized. Keeping the same logical names lets the rest of the firmware
    // stay revision-agnostic.
    pub const SD_CS: u8 = 0;
    pub const BUZZER: u8 = 1;
    pub const NEOPIXEL: u8 = 2;
    pub const EJECT_BUTTON: u8 = 3;
    pub const ADXL_INT1: u8 = 4;
    pub const ADXL_INT2: u8 = 5;
    pub const LSM_INT1: u8 = 6;
    pub const LSM_INT2: u8 = 7;
    pub const LIS3_INT1: u8 = 8;
    pub const LIS3_INT2: u8 = 9;
    pub const BMP_INT: u8 = 10;
    pub const SERVO_XPLUS: u8 = 11;
    pub const SERVO_XMINUS: u8 = 12;
    pub const SERVO_YPLUS: u8 = 13;
    pub const SERVO_YMINUS: u8 = 14;
    pub const BATTERY_VOLTAGE: u8 = 26;
    pub const TURN_SIGNAL_LEFT: u8 = 15;
    pub const TURN_SIGNAL_RIGHT: u8 = 16;
}

// Default to v2 (flight hardware) if no feature was set so `cargo check`
// against an unconfigured build still resolves all the pin references.
#[cfg(not(any(feature = "hw-v2", feature = "hw-v3")))]
pub mod pins {
    pub const SD_CS: u8 = 23;
    pub const BUZZER: u8 = 28;
    pub const NEOPIXEL: u8 = 4;
    pub const EJECT_BUTTON: u8 = 24;
    pub const ADXL_INT1: u8 = 5;
    pub const ADXL_INT2: u8 = 6;
    pub const LSM_INT1: u8 = 9;
    pub const LSM_INT2: u8 = 10;
    pub const LIS3_INT1: u8 = 11;
    pub const LIS3_INT2: u8 = 12;
    pub const BMP_INT: u8 = 13;
    pub const SERVO_XPLUS: u8 = 25;
    pub const SERVO_XMINUS: u8 = 14;
    pub const SERVO_YPLUS: u8 = 15;
    pub const SERVO_YMINUS: u8 = 8;
    pub const BATTERY_VOLTAGE: u8 = 29;
    pub const TURN_SIGNAL_LEFT: u8 = 26;
    pub const TURN_SIGNAL_RIGHT: u8 = 27;
}
