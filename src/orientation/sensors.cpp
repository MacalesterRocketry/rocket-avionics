#include "sensors.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_ADXL375 adxl_accel = Adafruit_ADXL375(12345); // high-g accelerometer
Adafruit_LSM6DSOX lsm6dsox; // low-g accelerometer and gyro 
Adafruit_LIS3MDL lis3mdl; // magnetometer
BMP3XX bmp; // barometer

sensors_event_t lowg_accel;
sensors_event_t gyro;
sensors_event_t lsm6ds_temp;
sensors_event_t magnetometer;
sensors_event_t highg_accel;
float bmp_altitude;

void initLowGAccelerometer() {
  if (!lsm6dsox.begin_I2C()) {
    error("Failed to find LSM6DS chip; no low-g accelerometer data", false);
  }

  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
}

void initMagnetometer() {
  if (!lis3mdl.begin_I2C()) {
    error("Failed to find LIS3MDL chip; no magnetometer data", false);
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_560_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
}

void initHighGAccelerometer() {
  if (!adxl_accel.begin()) {
    error("Failed to find ADXL375 chip; no high-g accelerometer data", false);
  }

  adxl_accel.setDataRate(ADXL343_DATARATE_800_HZ);

  // Set activity threshold for launch detection
  constexpr float launch_threshold_mg = LAUNCH_ACCEL_THRESHOLD_G * 1000; // convert from g to mg
  constexpr float scale_factor = 780.0; // units of mg/LSB
  const uint8_t launch_threshold_raw = floor(launch_threshold_mg / scale_factor);
  adxl_accel.writeRegister(ADXL3XX_REG_THRESH_ACT, launch_threshold_raw);

  // Map activity interrupt to INT1
  int_config interrupt_config = {};
  interrupt_config.bits.activity = 0; // 0 means INT1, 1 means INT2
  adxl_accel.mapInterrupts(interrupt_config);

  // Enable activity interrupt
  int_config enabled_interrupts = {};
  enabled_interrupts.bits.activity = 1;
  adxl_accel.enableInterrupts(enabled_interrupts);
}

/**
 * Checks for the G activity interrupt on the ADXL375 high-G accelerometer and clears interrupts
 * Note: The interrupts are cleared if data is read from the sensor before this.
 * @return True if accelerometer has recorded more than LAUNCH_ACCEL_THRESHOLD_G Gs of acceleration since the last check
 */
bool hasLaunched() {
  // For testing, just check the values and see if the magnitude is greater than the threshold.
  return sqrt(
              highg_accel.acceleration.x * highg_accel.acceleration.x +
              highg_accel.acceleration.y * highg_accel.acceleration.y +
              highg_accel.acceleration.z * highg_accel.acceleration.z
             ) >= (LAUNCH_ACCEL_THRESHOLD_G * SENSORS_GRAVITY_EARTH);
  // return (adxl_accel.checkInterrupts() >> 4) & 1; // Note: This is on _any_ axis, not the magnitude.
  // 0          | 0            | 0            | 0        | 0          | 0      | 0         | 0
  // DATA_READY | SINGLE_SHOCK | DOUBLE_SHOCK | Activity | Inactivity | Ignore | Watermark | Overrun
  // I want Activity, so I'm bitshifting it right by four and using a bitmask to isolate it.
}

void initBarometer() {
  if (!bmp.begin_I2C()) {
    error("Failed to find BMP390 chip; no barometric altitude data", false);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);
}

void initSensors() {
  initLowGAccelerometer();
  initMagnetometer();
  initHighGAccelerometer();
  initBarometer();

#if DEBUG
  Serial.println("ADXL375 details:");
  adxl_accel.printSensorDetails();
#endif
}

LSMReading readLSM() {
  lsm6dsox.getEvent(&lowg_accel, &gyro, &lsm6ds_temp);
  const LSMReading reading{
    .accel = {
      lowg_accel.acceleration.x - LOWG_BIAS_X,
      lowg_accel.acceleration.y - LOWG_BIAS_Y,
      lowg_accel.acceleration.z - LOWG_BIAS_Z
    },
    .gyro = {gyro.gyro.x - GYRO_BIAS_X, gyro.gyro.y - GYRO_BIAS_Y, gyro.gyro.z - GYRO_BIAS_Z},
    .temperature = lsm6ds_temp.temperature
  };
  logIMU(reading.accel.x, reading.accel.y, reading.accel.z,
         reading.gyro.x, reading.gyro.y, reading.gyro.z,
         reading.temperature);
#if DEBUG and DEBUG_PRINT_SENSORS
  Serial.printf("Low-G Accel: %.2f X, %.2f Y, %.2f Z\n", reading.accel.x, reading.accel.y, reading.accel.z);
  Serial.printf("Gyro: %.2f X, %.2f Y, %.2f Z, temp: %.2f\n", reading.gyro.x, reading.gyro.y, reading.gyro.z, reading.temperature);
#endif
  return reading;
}

LIS3Reading readLIS3() {
  lis3mdl.getEvent(&magnetometer);
  const LIS3Reading reading{
    .mag = {
      magnetometer.magnetic.x - MAG_BIAS_X,
      magnetometer.magnetic.y - MAG_BIAS_Y,
      magnetometer.magnetic.z - MAG_BIAS_Z
    }
  };
  logMagnetometer(reading.mag.x, reading.mag.y, reading.mag.z);
#if DEBUG and DEBUG_PRINT_SENSORS
  Serial.printf("Mag: %.2f X, %.2f Y, %.2f Z\n", reading.mag.x, reading.mag.y, reading.mag.z);
#endif
  return reading;
}

ADXLReading readADXL() {
  adxl_accel.getEvent(&highg_accel);
  const ADXLReading reading{
    .highg_accel = {
      highg_accel.acceleration.x - HIGHG_BIAS_X,
      highg_accel.acceleration.y - HIGHG_BIAS_Y,
      highg_accel.acceleration.z - HIGHG_BIAS_Z
    }
  };
  logHighG(reading.highg_accel.x, reading.highg_accel.y, reading.highg_accel.z);
#if DEBUG and DEBUG_PRINT_SENSORS
  Serial.printf("High-G Accel: %.2f X, %.2f Y, %.2f Z\n", reading.highg_accel.x, reading.highg_accel.y, reading.highg_accel.z);
#endif
  return reading;
}

BMPReading readBMP() {
  bmp.performReading();
  bmp_altitude = bmp.getAltitude(SEALEVELPRESSURE_HPA);
  logBarometer(bmp.pressure, bmp_altitude, bmp.temperature); // TODO: Read only when new data received
#if DEBUG and DEBUG_PRINT_SENSORS
  Serial.printf("BMP: %.2f Pa, %.2f m, %.2f C\n", bmp.pressure, bmp_altitude, bmp.temperature);
#endif
  return BMPReading {
    .pressure = bmp.pressure,
    .temperature = bmp.temperature,
    .altitude = bmp_altitude
  };
}

SensorReadings readSensors() {
  return SensorReadings {
    .lsm = readLSM(),
    .lis3 = readLIS3(),
    .adxl = readADXL(),
    .bmp = readBMP()
  };
}
