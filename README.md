# Macalester Rocket Avionics

This is the code for the avionics stack for the Macalester College High Power Rocketry team. It is currently in a very early state and in active development. At the moment, it just logs data, but active roll control with sensor fusion is in progress.

Hardware:
- Microcontroller: Adafruit Feather RP2040 Adalogger
- High-G accelerometer: ADXL375
- Gyroscope and low-G accelerometer: LSM6DSOX
- Magnetometer: LIS3MDL
- Barometer: BMP390
- GPS module: MTK3333 on an Adafruit Ultimate GPS Featherwing

At the moment, all sensors but the GPS are connected over I²C using Stemma QT, and the GPS module is connected over UART. A custom FeatherWing PCB is in progress to combine all the sensors onto one board, improve interrupt support, add battery monitoring, and much more.

Supports a buzzer with built-in driver circuitry for state notification; PWM buzzers are not supported at the moment.

## Log decoder

Use `decoder.py` to decode a `logNN.bin` file to CSV and optionally re-export a filtered binary by timestamp (`micros`) range.

```bash
python decoder.py --search-dir /path/to/sd_dump --start-micros 1200000 --end-micros 15200000 --output-bin ignition_to_landing.bin --output-csv ignition_to_landing.csv
```

Notes:
- Range bounds are inclusive (`start <= micros <= end`).
- If `--input-bin` is omitted, the script selects the latest `logNN.bin` in `--search-dir`.
- The filtered `.bin` keeps the original file header and packet bytes so it can be decoded again later.

