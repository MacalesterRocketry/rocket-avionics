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
pub const HAS_DROGUE_CHUTE: bool = true;

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

macro_rules! define_hardware {
    (
        $main_struct:ident {
            // Match all grouped subsystems (e.g., I2cConfig, SdConfig)
            $(
                $group_field:ident : $group_struct:ident {
                    $( $sub_field:ident : $sub_pin:ident ),* $(,)?
                }
            ),* $(,)?
        }
    ) => {
        // 1. Generate all the sub-structs
        $(
            pub struct $group_struct {
                $( pub $sub_field: embassy_rp::Peri<'static, embassy_rp::peripherals::$sub_pin> ),*
            }
        )*

        // 2. Generate the main hardware struct
        pub struct $main_struct {
            $( pub $group_field: $group_struct, )*
        }

        // 3. Generate the partial-move extraction macro
        #[macro_export]
        macro_rules! take_hardware {
            ($p:expr) => {
                crate::config::board::$main_struct {
                    $(
                        $group_field: crate::config::board::$group_struct {
                            $( $sub_field: $p.$sub_pin ),*
                        },
                    )*
                }
            }
        }
    };
}

#[cfg(feature = "hw-v3")]
pub mod board {
    use embassy_rp::Peri;
    use embassy_rp::peripherals::*;
    use embassy_rp::pio_programs::ws2812::PioWs2812;

    /// External high-speed crystal on the Metro RP2350 board is 12 MHz, as with most RP2350 boards
    pub(crate) const XTAL_FREQ_HZ: u32 = 12_000_000u32;
    pub(crate) const NUM_LEDS: usize = 1;
    pub type NeopixelColorOrder = embassy_rp::pio_programs::ws2812::Grb;
    pub type Neopixel = PioWs2812<'static, PIO0, 0, NUM_LEDS, NeopixelColorOrder>;

    // Now you map logical names to physical pins exactly once.
    // To add a pin, just add one line here.
    define_hardware!(AvionicsHardware {
        i2c: I2cConfig {
            bus: I2C0,
            sda: PIN_20,
            scl: PIN_21,
        },
        gps: GpsConfig {
            rx: PIN_1,
            tx: PIN_0,
            bus: UART0,
        },
        telemetry: TelemetryConfig {
            rx: PIN_5,
            tx: PIN_4,
            bus: UART1,
        },
        sd: SdConfig {
            sclk: PIN_34,
            mosi: PIN_35,
            miso: PIN_36,
            data1: PIN_37,
            data2: PIN_38,
            cs: PIN_39,
            detect: PIN_40,
        },
        interrupts: InterruptConfig {
            adxl_int1: PIN_7,
            adxl_int2: PIN_8,
            lsm_int1: PIN_9,
            lsm_int2: PIN_10,
            lis3_int1: PIN_11,
            lis3_int2: PIN_12,
            bmp_int: PIN_13,
        },
        indicators: IndicatorsConfig {
            buzzer: PIN_43, // Note: Any changes to this should also be changed in errors.rs
            neopixel: PIN_25,
            neopixel_pio: PIO0,
            neopixel_dma: DMA_CH0,
        },
        peripherals: PeripheralConfig {
            eject_button: PIN_24,
        },
    });
}

#[cfg(feature = "hw-v2")]
pub mod board {
    /// Adafruit Feather RP2040 Adalogger's external high-speed crystal is 12 MHz
    pub const XTAL_FREQ_HZ: u32 = 12_000_000;
    pub(crate) const NUM_LEDS: usize = 1;
    pub type NeopixelColorOrder = embassy_rp::pio_programs::ws2812::Grb;

    use embassy_rp::Peri;
    use embassy_rp::peripherals::*;

    define_hardware!(AvionicsHardware {
        i2c: I2cConfig {
            bus: I2C0,
            sda: PIN_2,
            scl: PIN_3,
        },
        uart: UartConfig {
            rx: PIN_1,
            tx: PIN_0,
        },
        sd: SdConfig {
            sclk: PIN_18,
            mosi: PIN_19,
            miso: PIN_20,
            data1: PIN_21,
            data2: PIN_22,
            cs: PIN_23,
            detect: PIN_16,
        },
        interrupts: InterruptConfig {
            adxl_int1: PIN_5,
            adxl_int2: PIN_6,
            lsm_int1: PIN_9,
            lsm_int2: PIN_10,
            lis3_int1: PIN_11,
            lis3_int2: PIN_12,
            bmp_int: PIN_13,
        },
        peripherals: PeripheralConfig {
            neopixel: PIN_4,
            buzzer: PIN_28,
            eject_button: PIN_24,
            battery_voltage: PIN_29,
            turn_signal_left: PIN_26,
            turn_signal_right: PIN_27,
        },
        servos: ServoConfig {
            xplus: PIN_25,
            xminus: PIN_14,
            yplus: PIN_15,
            yminus: PIN_8,
        },
    });
}
