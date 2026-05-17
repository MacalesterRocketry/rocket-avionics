#!/usr/bin/env python3

import argparse
import os
import struct
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import foxglove

import pandas as pd
from pandas import DataFrame


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
    DATETIME = 0x21 # DateTime (from GPS)
    # length: 7 bytes: uint16_t year, uint8_t month, uint8_t day, uint8_t hours, uint8_t minutes, uint8_t seconds
    # Should only appear once, at beginning of log
    EVENT   = 0x30  # Discrete Events (Launch, Apogee)
    # length: 3 bytes, all uint8_t (oldState, newState, reasonCode)
    STATUS  = 0x40  # Battery, etc.
    # length: 6 bytes: uint8_t rocketState, float batteryVoltage, uint8_t sensorsDetected
    AHRS    = 0x50  # Orientation Quaternion, earth-frame acceleration, velocity, position from AHRS filter
    # length: 52 bytes: float qW, qX, qY, qZ, accX_earth, accY_earth, accZ_earth, velX_earth, velY_earth, velZ_earth, posX_earth, posY_earth, posZ_earth
    CONTROL = 0x51  # Roll control data: float targetAngle, float currentAngle, float deflectionAngle


FILE_VERSION = 3
LITTLE_ENDIAN = 0
BIG_ENDIAN = 1
PACKET_HEADER_SIZE = 9

PACKET_FORMATS = {
    PacketType.IMU.value: "fffffff",
    PacketType.HIGHG.value: "fff",
    PacketType.MAG.value: "fff",
    PacketType.BARO.value: "fff",
    PacketType.GPS.value: "BBBBiifffBB",
    PacketType.DATETIME.value: "HBBBBB",
    PacketType.EVENT.value: "BBB",
    PacketType.STATUS.value: "BfB",
    PacketType.AHRS.value: "fffffffffffff",
    PacketType.CONTROL.value: "fff",
}


@dataclass
class ParsedPacket:
    pkt_type: int
    micros: int
    raw_packet: bytes
    decoded_row: dict

eventTypes: dict = {
    1: "Launch Detected",
    2: "Apogee Detected",
    3: "Burnout Detected",
    4: "SD Sync",
    255: "Other"
}
systemStates: dict = {
    0: "Starting",
    1: "Ready to Launch",
    2: "Ascent",
    3: "Error",
    4: "Warning",
    5: "File Closed",
    255: "Irrelevant"
}


def map_state(state: int) -> str:
    if state in systemStates:
        state_str = systemStates[state]
    else:
        state_str = f"Unknown ({state})"
    return state_str


def map_event(event: int) -> str:
    if event in eventTypes:
        event_str = eventTypes[event]
    else:
        event_str = f"Unknown ({event})"
    return event_str


def find_latest_log_file(search_dir: str) -> str:
    highest_log_num = -1
    latest_log_file = ""
    for file_name in os.listdir(search_dir):
        if not (file_name.startswith("log") and file_name.endswith(".bin")):
            continue
        log_index = file_name[3:5]
        if not log_index.isdigit():
            continue
        log_num = int(log_index)
        if log_num > highest_log_num:
            highest_log_num = log_num
            latest_log_file = file_name

    if latest_log_file == "":
        raise FileNotFoundError(f"No logNN.bin files found in {search_dir}")

    return os.path.join(search_dir, latest_log_file)


def decode_packet_payload(pkt_type: int, unpacked_data: tuple) -> dict:
    row: dict = {}

    if pkt_type == PacketType.IMU.value:
        row.update({
            "accX": unpacked_data[0], "accY": unpacked_data[1], "accZ": unpacked_data[2],
            "gyroX": unpacked_data[3], "gyroY": unpacked_data[4], "gyroZ": unpacked_data[5],
            "IMUtemp": unpacked_data[6]
        })
    elif pkt_type == PacketType.HIGHG.value:
        row.update({
            "highg_accX": unpacked_data[0], "highg_accY": unpacked_data[1], "highg_accZ": unpacked_data[2]
        })
    elif pkt_type == PacketType.MAG.value:
        row.update({
            "magX": unpacked_data[0], "magY": unpacked_data[1], "magZ": unpacked_data[2]
        })
    elif pkt_type == PacketType.BARO.value:
        # Firmware currently writes (pressure, altitude, temperature).
        row.update({
            "pressure": unpacked_data[0], "altitude": unpacked_data[1], "baro_temp": unpacked_data[2]
        })
    elif pkt_type == PacketType.GPS.value:
        row.update({
            "gps_hours": unpacked_data[0], "gps_minutes": unpacked_data[1], "gps_seconds": unpacked_data[2],
            "gps_milliseconds": unpacked_data[3] * 100,
            "latitude": unpacked_data[4] / 10_000_000, "longitude": unpacked_data[5] / 10_000_000,
            "speed": unpacked_data[6], "angle": unpacked_data[7], "gps_altitude": unpacked_data[8],
            "satellites": unpacked_data[9], "fixquality": unpacked_data[10]
        })
    elif pkt_type == PacketType.DATETIME.value:
        row.update({
            "year": unpacked_data[0], "month": unpacked_data[1], "day": unpacked_data[2],
            "dt_hours": unpacked_data[3], "dt_minutes": unpacked_data[4], "dt_seconds": unpacked_data[5]
        })
    elif pkt_type == PacketType.EVENT.value:
        row.update({
            "oldState": map_state(unpacked_data[0]), "newState": map_state(unpacked_data[1]),
            "reasonCode": map_event(unpacked_data[2])
        })
    elif pkt_type == PacketType.STATUS.value:
        row.update({
            "rocketState": unpacked_data[0], "batteryVoltage": unpacked_data[1], "sensorsDetected": unpacked_data[2]
        })
    elif pkt_type == PacketType.AHRS.value:
        row.update({
            "qW": unpacked_data[0], "qX": unpacked_data[1], "qY": unpacked_data[2], "qZ": unpacked_data[3],
            "accX_earth": unpacked_data[4], "accY_earth": unpacked_data[5], "accZ_earth": unpacked_data[6],
            "velX_earth": unpacked_data[7], "velY_earth": unpacked_data[8], "velZ_earth": unpacked_data[9],
            "posX_earth": unpacked_data[10], "posY_earth": unpacked_data[11], "posZ_earth": unpacked_data[12]
        })
    elif pkt_type == PacketType.CONTROL.value:
        row.update({
            "targetAngle": unpacked_data[0], "currentAngle": unpacked_data[1], "deflectionAngle": unpacked_data[2]
        })

    return row


def parse_file_header(file_obj) -> tuple[bytes, str]:
    header = file_obj.read(2)
    if len(header) != 2:
        raise ValueError("Log file is missing FileHeader")

    version, endian_raw = header[0], header[1]
    if version != FILE_VERSION:
        raise ValueError(f"Unsupported version {version}; expected {FILE_VERSION}")

    if endian_raw == LITTLE_ENDIAN:
        return header, "<"
    if endian_raw == BIG_ENDIAN:
        return header, ">"

    raise ValueError(f"Unsupported endianness marker {endian_raw}")


def iter_packets(file_obj, endian_prefix: str):
    while True:
        header_bytes = file_obj.read(PACKET_HEADER_SIZE)
        if len(header_bytes) == 0:
            return
        if len(header_bytes) < PACKET_HEADER_SIZE:
            raise ValueError("Truncated packet header")

        pkt_type, micros = struct.unpack(endian_prefix + "BQ", header_bytes)
        payload_fmt = PACKET_FORMATS.get(pkt_type)
        if payload_fmt is None:
            raise ValueError(f"Unknown packet type: 0x{pkt_type:02x}")

        payload_size = struct.calcsize(endian_prefix + payload_fmt)
        payload_bytes = file_obj.read(payload_size)
        if len(payload_bytes) < payload_size:
            raise ValueError(
                f"Truncated payload for packet type 0x{pkt_type:02x}; "
                f"expected {payload_size} bytes, got {len(payload_bytes)}"
            )

        unpacked = struct.unpack(endian_prefix + payload_fmt, payload_bytes)
        row = {"type": pkt_type, "micros": micros}
        row.update(decode_packet_payload(pkt_type, unpacked))

        yield ParsedPacket(
            pkt_type=pkt_type,
            micros=micros,
            raw_packet=header_bytes + payload_bytes,
            decoded_row=row,
        )


def read_log_file(log_path: Path | str) -> tuple[bytes, dict[PacketType, list[ParsedPacket]]]:
    packets: dict[PacketType, list[ParsedPacket]] = {}

    with open(log_path, "rb") as file_obj:
        file_header, endian_prefix = parse_file_header(file_obj)
        for packet in iter_packets(file_obj, endian_prefix):
            packet_type = PacketType(packet.pkt_type)
            if packet_type not in packets:
                packets[packet_type] = []
            packets[packet_type].append(packet)

    return file_header, packets


def filter_packets_by_time(packets: dict[PacketType, list[ParsedPacket]], start_micros: int | None = None, end_micros: int | None = None,
                              start_secs: float | None = None, end_secs: float | None = None) -> dict[PacketType, list[ParsedPacket]]:
    if start_secs is not None:
        start_micros = int(start_secs * 1_000_000)
    if end_secs is not None:
        end_micros = int(end_secs * 1_000_000)
    data: dict[PacketType, list[ParsedPacket]] = {}
    for pkt_type in packets:
        filtered = []
        for packet in packets[pkt_type]:
            if start_micros is not None and packet.micros < start_micros:
                continue
            if end_micros is not None and packet.micros > end_micros:
                continue
            filtered.append(packet)
        data[pkt_type] = filtered
    return data


def filter_dataframes_by_time(dataframes: dict[PacketType, DataFrame], start_micros: int | None = None, end_micros: int | None = None,
                              start_secs: float | None = None, end_secs: float | None = None) -> dict[PacketType, DataFrame]:
    dfs = dataframes.copy()
    for pkt_type in dfs:
        if start_secs is not None:
            start_micros = int(start_secs * 1_000_000)
        if end_secs is not None:
            end_micros = int(end_secs * 1_000_000)
        if start_micros is not None:
            dfs[pkt_type] = dfs[pkt_type][dfs[pkt_type]["micros"] >= start_micros]
        if end_micros is not None:
            dfs[pkt_type] = dfs[pkt_type][dfs[pkt_type]["micros"] <= end_micros]
    return dfs


def export_filtered_bin(output_path: str, file_header: bytes, packets: dict[PacketType, list[ParsedPacket]]) -> None:
    with open(output_path, "wb") as file_obj:
        file_obj.write(file_header)

        for pkt_type, pkt_list in packets.items():
            pkt_list.sort(key=lambda p: p.micros)
        all_packets = [pkt for pkt_list in packets.values() for pkt in pkt_list]
        all_packets.sort(key=lambda p: p.micros)

        for packet in all_packets:
            file_obj.write(packet.raw_packet)


def packets_to_dataframes(packets: dict[PacketType, list[ParsedPacket]]) -> dict[PacketType, pd.DataFrame]:
    df_dict: dict[PacketType, pd.DataFrame] = {}
    for pkt_type, pkt_list in packets.items():
        rows = [packet.decoded_row for packet in pkt_list]
        if rows:
            df_dict[pkt_type] = pd.DataFrame(rows)
        else:
            df_dict[pkt_type] = pd.DataFrame(columns=["type", "micros"])
        df_dict[pkt_type]["seconds"] = df_dict[pkt_type]["micros"] / 1_000_000
        df_dict[pkt_type].set_index("seconds", inplace=True)

    return df_dict


def bin_to_dataframes(bin_path: Path | str,
                      start_sec: float | None = None, end_sec: float | None = None,
                      start_micros: int | None = None, end_micros: int | None = None
                      ) -> dict[PacketType, DataFrame]:
    _, packets = read_log_file(bin_path)
    if start_sec is not None:
        start_micros = int(start_sec * 1_000_000)
    if end_sec is not None:
        end_micros = int(end_sec * 1_000_000)
    if start_micros is not None or end_micros is not None:
        packets = filter_packets_by_time(packets, start_micros=start_micros, end_micros=end_micros)
    dfs = packets_to_dataframes(packets)
    return dfs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Decode rocket avionics logs and optionally export a timestamp-filtered .bin")
    parser.add_argument("--input-bin", type=Path, help="Path to input .bin file. If omitted, latest logNN.bin in --search-dir is used.")
    parser.add_argument("--search-dir", type=Path, default=".", help="Directory to search for latest logNN.bin when --input-bin is not provided")
    parser.add_argument("--start-micros", type=float, help="Inclusive lower micros timestamp bound")
    parser.add_argument("--start-secs", type=float, help="Inclusive lower seconds timestamp bound")
    parser.add_argument("--end-micros", type=int, help="Inclusive upper micros timestamp bound")
    parser.add_argument("--end-secs", type=int, help="Inclusive upper seconds timestamp bound")
    parser.add_argument("--output-bin", type=Path, help="Optional output path for filtered .bin file")
    parser.add_argument("--output-csv", type=Path, help="Optional CSV output path for decoded filtered rows")
    parser.add_argument("--output-mcap", type=Path, help="Optional MCAP output path for decoded filtered rows")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    has_time_range = any([args.start_micros, args.start_secs]) and any([args.end_micros, args.end_secs])
    if has_time_range:
        if args.start_micros is not None and args.start_secs is not None:
            raise ValueError("Cannot specify both --start-micros and --start-secs")
        if args.end_micros is not None and args.end_secs is not None:
            raise ValueError("Cannot specify both --end-micros and --end-secs")
        if args.start_secs is not None:
            args.start_micros = int(args.start_secs * 1_000_000)
        if args.end_secs is not None:
            args.end_micros = int(args.end_secs * 1_000_000)
        if args.start_micros is None or args.end_micros is None:
            raise ValueError("Both start and end times must be specified when filtering by time")
        if args.start_micros > args.end_micros:
            raise ValueError("Start time must be <= end time")

    if args.input_bin:
        input_path = args.input_bin
    else:
        input_path = find_latest_log_file(args.search_dir)

    print(f"Reading log file: {input_path}")

    file_header, packets = read_log_file(input_path)
    packets = filter_packets_by_time(packets, start_micros=args.start_micros, end_micros=args.end_micros)

    dfs = packets_to_dataframes(packets)

    for pkt_type, df in dfs.items():
        print(f"\nPacket Type: {pkt_type.name} (0x{pkt_type.value:02x}), {len(df)} packets")
        print(df.head(8))

    if args.output_csv:
        combined_df = pd.concat(dfs.values())
        combined_df.sort_values("micros", inplace=True)
        combined_df.to_csv(args.output_csv, index=False)
        print(f"Wrote CSV: {args.output_csv}")

    if args.output_bin:
        export_filtered_bin(args.output_bin, file_header, packets)
        print(f"Wrote filtered BIN: {args.output_bin}")

    if args.output_mcap:
        # combined_df = pd.concat(dfs.values())
        # combined_df.sort_values("micros", inplace=True)
        with foxglove.open_mcap(args.output_mcap):
            # find which df has the most recent packet
            while True:
                latest_pkt_type = None
                latest_micros = -1
                for pkt_type, df in dfs.items():
                    if not df.empty:
                        last_micros = df["micros"].iloc[-1]
                        if last_micros > latest_micros:
                            latest_micros = last_micros
                            latest_pkt_type = pkt_type
                if latest_pkt_type is None:
                    break
                df = dfs[latest_pkt_type]
                row = df.iloc[0]
                topic = f"/{latest_pkt_type.name}"
                nsec = int(row["micros"] * 1000)
                if latest_pkt_type == PacketType.AHRS:
                    pose = foxglove.messages.Pose(
                        orientation=foxglove.messages.Quaternion(x=row["qX"],
                                                                 y=row["qY"],
                                                                 z=row["qZ"],
                                                                 w=row["qW"]),
                        position=foxglove.messages.Vector3(x=row["posX_earth"],
                                                           y=row["posY_earth"],
                                                           z=row["posZ_earth"]))
                    sec = int(row["micros"] / 1_000_000)
                    timestamp: foxglove.messages.Timestamp = foxglove.messages.Timestamp(sec=sec, nsec=nsec % 1_000_000_000)
                    poses_in_frame = foxglove.messages.PosesInFrame(timestamp=timestamp, poses=[pose], frame_id="earth")
                    foxglove.log(topic, poses_in_frame, log_time=nsec)

                    frame = foxglove.messages.FrameTransform(
                        parent_frame_id="earth",
                        child_frame_id="body",
                        translation=foxglove.messages.Vector3(x=row["posX_earth"], y=row["posY_earth"], z=row["posZ_earth"]),
                        rotation=foxglove.messages.Quaternion(x=row["qX"], y=row["qY"], z=row["qZ"], w=row["qW"]),
                        timestamp=timestamp
                    )
                    foxglove.log("/tf", frame, log_time=nsec)
                else:
                    dict_to_log = row.drop(labels=["type", "micros"]).to_dict()
                    foxglove.log(topic, dict_to_log, log_time=nsec)
                dfs[latest_pkt_type] = df.iloc[1:]
        print(f"Wrote MCAP: {args.output_mcap}")


if __name__ == "__main__":
    main()
