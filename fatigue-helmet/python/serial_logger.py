#!/usr/bin/env python3
"""
Serial logger for IoT Helmet fatigue detection.
Reads CSV sensor data from ESP32 over USB-Serial and saves to a dated CSV file.

Usage:
    python serial_logger.py                        # interactive port selection
    python serial_logger.py --port COM3            # specify port
    python serial_logger.py --port COM3 --output my_session.csv
"""

import argparse
import csv
import sys
import time
from datetime import datetime

import serial
import serial.tools.list_ports

# ── Constants ─────────────────────────────────────────────────────────────
BAUD_RATE   = 115200
NUM_FIELDS  = 11        # fields emitted by firmware CSV line
STATS_EVERY = 5.0       # seconds between live stats printout

# Firmware emits 11 fields; Python adds pc_timestamp + label
CSV_COLUMNS = [
    "pc_timestamp",
    "timestamp_ms", "hr_bpm", "pulse_raw",
    "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps",
    "head_movement", "signal_quality",
    "label",          # filled manually after session using KSS 1-9 scale
]

FIRMWARE_FIELD_NAMES = CSV_COLUMNS[1:-1]   # 11 names matching firmware output order


def list_available_ports():
    return sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)


def select_port_interactively():
    ports = list_available_ports()
    if not ports:
        print("[ERROR] No COM ports found. Is the ESP32 plugged in?")
        sys.exit(1)

    print("\nAvailable serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}]  {p.device:<12} {p.description}")

    while True:
        try:
            idx = int(input("\nEnter port number: ").strip())
            if 0 <= idx < len(ports):
                return ports[idx].device
        except (ValueError, EOFError):
            pass
        print("  Invalid choice — try again.")


def make_output_filename():
    return f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"


def parse_args():
    parser = argparse.ArgumentParser(
        description="ESP32 helmet sensor → CSV logger"
    )
    parser.add_argument("--port",   help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--output", help="Output CSV path (auto-named if omitted)")
    return parser.parse_args()


def open_serial(port: str) -> serial.Serial:
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=2)
        return ser
    except serial.SerialException as exc:
        print(f"[ERROR] Cannot open {port}: {exc}")
        sys.exit(1)


def print_stats(line_count: int, hr: int, pulse_raw: int, quality: int, elapsed: float):
    quality_str = "OK" if quality == 1 else "NO CONTACT"
    print(
        f"[{elapsed:6.0f}s]  lines={line_count:6d}  "
        f"HR={hr:3d} BPM  pulse_raw={pulse_raw}  signal={quality_str}  "
        f"duration={elapsed / 60:.1f} min"
    )


def main():
    args     = parse_args()
    port     = args.port   or select_port_interactively()
    out_path = args.output or make_output_filename()

    print(f"\nConnecting to {port} @ {BAUD_RATE} baud ...")
    ser = open_serial(port)
    print(f"Logging to  : {out_path}")
    print("Press Ctrl+C to stop.\n")

    line_count     = 0
    last_hr        = 0
    last_pulse_raw = 0
    last_quality   = 0
    start_time     = time.time()
    last_stat_ts   = start_time

    try:
        with open(out_path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
            writer.writeheader()

            while True:
                # Read one line from the ESP32
                try:
                    raw = ser.readline().decode("utf-8", errors="replace").strip()
                except serial.SerialException as exc:
                    print(f"\n[ERROR] Serial disconnect: {exc}")
                    break

                if not raw:
                    continue

                # Lines with # prefix are firmware status/error messages — print but skip
                if raw.startswith("#"):
                    print(f"[ESP32] {raw}")
                    continue

                # Validate field count before touching the data
                fields = raw.split(",")
                if len(fields) != NUM_FIELDS:
                    print(f"[WARN ] Malformed line ({len(fields)} fields, expected {NUM_FIELDS}): "
                          f"{raw[:70]!r}")
                    continue

                row = {"pc_timestamp": datetime.now().isoformat(), "label": ""}
                row.update(dict(zip(FIRMWARE_FIELD_NAMES, fields)))

                writer.writerow(row)
                fh.flush()   # keep file intact on unexpected exit
                line_count += 1

                # Track latest values for stats display
                try:
                    last_hr        = int(fields[1])   # hr_bpm
                    last_pulse_raw = int(fields[2])   # pulse_raw
                    last_quality   = int(fields[10])  # signal_quality
                except ValueError:
                    pass

                # Periodic live stats
                now = time.time()
                if now - last_stat_ts >= STATS_EVERY:
                    print_stats(line_count, last_hr, last_pulse_raw, last_quality, now - start_time)
                    last_stat_ts = now

    except KeyboardInterrupt:
        elapsed = time.time() - start_time
        print(f"\n\nStopped. {line_count} rows saved to '{out_path}' "
              f"({elapsed:.0f}s / {elapsed / 60:.1f} min)")
    finally:
        if ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
