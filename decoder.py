import os
import struct
import pandas as pd
from enum import Enum


class PacketType(Enum):
    IMU     = 0x10  # IMU (low-G Accelerometer + gyroscope + temperature)
                    # length: 28 bytes, all floats (accX, accY, accZ, gyroX, gyroY, gyroZ, temp)
    HIGHG   = 0x11  # High-G Accelerometer
                    # length: 12 bytes, all floats (accX, accY, accZ)
    MAG     = 0x12  # Magnetometer
                    # length: 12 bytes, all floats (magX, magY, magZ)
    BARO    = 0x13  # Barometer (pressure, altitude, temperature)
                    # length: 12 bytes, all floats (pressure, altitude, temp)
    GPS     = 0x20  # GPS (includes time, position, etc.)
                    # length: 14 bytes:
                    #     uint8_t hours;
                    #     uint8_t minutes;
                    #     uint8_t seconds;
                    #     uint8_t deciseconds; // milliseconds are only ever 0, 200, 400, 600, 800 at 5Hz, so we can save space
                    #     int32_t latitude;  // Decimal degrees * 10,000,000 (standard int notation)
                    #     int32_t longitude;
                    #     float speed; // knots
                    #     float angle; // degrees
                    #     float altitude;  // meters
                    #     uint8_t satellites;
                    #     uint8_t fixquality; // 0 = Invalid, 1 = GPS, 2 = DGPS
    EVENT   = 0x30  # Discrete Events (Launch, Apogee)
                    # length: 3 bytes, all uint8_t (oldState, newState, reasonCode)
    STATUS  = 0x40  # Battery, etc.
                    # length: 6 bytes: uint8_t rocketState, float batteryVoltage, uint8_t sensorsDetected
    QUAT = 0x50     # Orientation Quaternion
                    # length: 16 bytes: 4 floats (qW, qX, qY, qZ)

data_list = []

files = os.listdir('/run/media/bensimmons/8214-BC9F/')
logFile = ""
highestLogNum = -1
for file in files:
    if file.startswith("log") and file.endswith(".bin"):
        logNum = int(file[3:5])
        if logNum > highestLogNum:
            highestLogNum = logNum
            logFile = file
print(f"Reading log file: {logFile}")

with open(f"/run/media/bensimmons/8214-BC9F/{logFile}", 'rb') as f:
    endian = "little"
    version = f.read(1)
    endianRaw = f.read(1)
    endianPrefix = ""
    if endianRaw == 0x0.to_bytes(1, 'little'):
        endian = "little"
        endianPrefix = "<"
    else:
        endian = "big"
        endianPrefix = ">"
    if version != 0x2.to_bytes(1, endian):
        print("Unsupported version")
        exit(1)

    while True:
        # Read the Header (9 bytes)
        header_bytes = f.read(9)
        if len(header_bytes) < 9: break # bad packet or EOF

        pkt_type, micros, millis = struct.unpack(endianPrefix + 'BII', header_bytes)
        row = {'type': pkt_type, 'micros': micros, 'millis': millis}

        # Switch based on Type to read the Payload
        if pkt_type == PacketType.IMU.value:
            data = struct.unpack(endianPrefix + 'fffffff', f.read(28))
            row.update({
                'accX': data[0], 'accY': data[1], 'accZ': data[2],
                'gyroX': data[3], 'gyroY': data[4], 'gyroZ': data[5],
                'IMUtemp': data[6]
            })

        elif pkt_type == PacketType.HIGHG.value:
            data = struct.unpack(endianPrefix + 'fff', f.read(12))
            row.update({
                'highg_accX': data[0], 'highg_accY': data[1], 'highg_accZ': data[2]
            })

        elif pkt_type == PacketType.MAG.value:
            data = struct.unpack(endianPrefix + 'fff', f.read(12))
            row.update({
                'magX': data[0], 'magY': data[1], 'magZ': data[2]
            })

        elif pkt_type == PacketType.BARO.value:
            data = struct.unpack(endianPrefix + 'fff', f.read(12))
            row.update({
                'pressure': data[0], 'altitude': data[1], 'baro_temp': data[2]
            })

        elif pkt_type == PacketType.GPS.value:
            data = struct.unpack(endianPrefix + 'BBBBiifffBB', f.read(26))
            row.update({
                'gps_hours': data[0], 'gps_minutes': data[1], 'gps_seconds': data[2], 'gps_milliseconds': data[3] * 100,
                'latitude': data[4] / 10_000_000, 'longitude': data[5] / 10_000_000,
                'speed': data[6], 'angle': data[7], 'gps_altitude': data[8],
                'satellites': data[9], 'fixquality': data[10]
            })

        elif pkt_type == PacketType.EVENT.value:
            data = struct.unpack(endianPrefix + 'BBB', f.read(3))
            row.update({
                'oldState': data[0], 'newState': data[1], 'reasonCode': data[2]
            })

        elif pkt_type == PacketType.STATUS.value:
            data = struct.unpack(endianPrefix + 'BfB', f.read(6))
            row.update({
                'rocketState': data[0], 'batteryVoltage': data[1], 'sensorsDetected': data[2]
            })

        elif pkt_type == PacketType.QUAT.value:
            data = struct.unpack(endianPrefix + 'ffff', f.read(16))
            row.update({
                'qW': data[0], 'qX': data[1], 'qY': data[2], 'qZ': data[3]
            })

        else:
            print(f"Unknown Packet Type: {hex(pkt_type)}")
            break # Stop to avoid reading garbage

        data_list.append(row)
df = pd.DataFrame(data_list)

# add millis delta column (time between readings)
df['millis_delta'] = df['millis'].diff().fillna(0)
df['micros_delta'] = df['micros'].diff().fillna(0)
# print average millis delta
avg_millis_delta = df['millis_delta'].mean()
print(f"Average millis delta: {avg_millis_delta}")
# max millis delta
max_millis_delta = df['millis_delta'].max()
print(f"Max millis delta: {max_millis_delta}")

print(df.head(8))
df.to_csv("flight_data.csv", index=False) # Save as CSV if needed
