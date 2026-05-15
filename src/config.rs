pub const DEBUG: bool = false;
pub const DEBUG_PRINT_SENSORS: bool = false;
pub const DEBUG_PRINT_ORIENTATION: bool = false;
pub const DEBUG_PRINT_ROLL_CONTROL: bool = false;

pub const PID_TUNING: bool = false;
pub const SERVO_TESTING: bool = false;

pub const PROFILING: bool = false;
pub const PROFILING_SAMPLES: usize = 100;

pub const SYNC_INTERVAL_MS: u64 = 5000;

pub const USE_GPS: bool = true;
pub const USE_TURN_SIGNALS: bool = true;

// Calibration
pub const G: f64 = 9.80579; // North Branch gravity in m/s²

pub const MAG_BIAS_X: f64 = -35.63;
pub const MAG_BIAS_Y: f64 = 19.96;
pub const MAG_BIAS_Z: f64 = -33.43;

pub const HIGHG_BIAS_X: f64 = 9.8;
pub const HIGHG_BIAS_Y: f64 = 7.85;
pub const HIGHG_BIAS_Z: f64 = 9.8 - G;
pub const HIGHG_TRIM_X: i16 = -5;
pub const HIGHG_TRIM_Y: i16 = -4;
pub const HIGHG_TRIM_Z: i16 = 0;

pub const LOWG_BIAS_X: f64 = 0.05;
pub const LOWG_BIAS_Y: f64 = 0.03;
pub const LOWG_BIAS_Z: f64 = 10.03 - G;

pub const GYRO_BIAS_X: f64 = -0.01;
pub const GYRO_BIAS_Y: f64 = 0.0;
pub const GYRO_BIAS_Z: f64 = 0.0;

// AHRS Constants
pub const AHRS_ACC_BETA: f64 = 0.1;
pub const AHRS_MAG_BETA: f64 = 0.0;

// PID constants
pub const MOMENT_OF_INERTIA: f64 = 0.00685; // MoI in kg m²
pub const BASE_TORQUE: f64 = 0.0983817; // Nm
pub const TORQUE_PER_DEG_50MS: f64 = BASE_TORQUE / 10.0;
pub const ROLL_PID_KP: f64 = 99.4232;
pub const ROLL_PID_KI: f64 = 9.1821;
pub const ROLL_PID_KD: f64 = 24.1417;

// Thresholds
pub const LAUNCH_ACCEL_THRESHOLD_G: f64 = 4.0;
pub const SERVO_DEGREE_RANGE: f64 = 100.0;
pub const SERVO_MAX_ANGLE: f64 = 50.0;
pub const SERVO_MIN_ANGLE: f64 = -50.0;
pub const SERVO_NEUTRAL_ANGLE: f64 = -3.0;
pub const SERVO_MICROS_MIN: u32 = 1000;
pub const SERVO_MICROS_MAX: u32 = 2000;
pub const SERVO_MAX_TORQUE: f64 = 0.51;

pub const ACCELEROMETER_SWITCH_THRESHOLD: f64 = 15.9 * G;

// Pin definitions (Hardware Version 2)
pub const SD_CS_PIN: u8 = 23;
pub const BUZZER_PIN: u8 = 28; // A2 corresponds to GPIO28 on Pico
pub const NEOPIXEL_PIN: u8 = 4;
pub const EJECT_BUTTON: u8 = 24;

pub const ADXL_INT1_PIN: u8 = 5;
pub const ADXL_INT2_PIN: u8 = 6;
pub const LSM_INT1_PIN: u8 = 9;
pub const LSM_INT2_PIN: u8 = 10;
pub const LIS3_INT1_PIN: u8 = 11;
pub const LIS3_INT2_PIN: u8 = 12;
pub const BMP_INT_PIN: u8 = 13;

pub const SERVO_XPLUS_PIN: u8 = 25;
pub const SERVO_XMINUS_PIN: u8 = 14;
pub const SERVO_YPLUS_PIN: u8 = 15;
pub const SERVO_YMINUS_PIN: u8 = 8;

pub const BATTERY_VOLTAGE_PIN: u8 = 29; // A3 corresponds to GPIO29 on Pico
pub const BATTERY_VOLTAGE_R1: f64 = 100000.0;
pub const BATTERY_VOLTAGE_R2: f64 = 100000.0;

pub const TURN_SIGNAL_LEFT_PIN: u8 = 26; // A0 corresponds to GPIO26
pub const TURN_SIGNAL_RIGHT_PIN: u8 = 27; // A1 corresponds to GPIO27
