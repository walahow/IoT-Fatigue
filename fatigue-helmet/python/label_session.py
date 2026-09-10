#!/usr/bin/env python3
"""
label_session.py — Attach a KSS fatigue label to a recorded SD-card session.
=============================================================================
Standalone (esp32s3cam_sd) recordings write sensor_data.csv straight from
the firmware's #HEADER line, with no label column and no header row:

    timestamp_ms,hr_bpm,pulse_raw,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,
    head_movement,signal_quality,blink_rate,pitch_deg,gyro_var,nod_score,
    risk_pct,alert_level,imu_valid

KSS (Karolinska Sleepiness Scale, 1-9) is a single self-rated number for
the whole session, given right after the ride while it's still fresh --
see thresholds_walkthrough.md and CLAUDE.md's KSS reference. This script
writes a NEW file, sensor_data_labeled.csv (the raw on-device output is
left untouched), with a header row plus that KSS value repeated as a
`label` column on every row -- the per-row format a future training
pipeline would expect. It also records the label in metadata.txt so it's
visible without opening the CSV.

Usage:
    python label_session.py --session ../../sessions/session_023 --kss 3
    python label_session.py --session ../../sessions/session_023 --kss 3 --condition rested

KSS reference:
    1 = Extremely alert            6 = Some signs of sleepiness
    2 = Very alert                 7 = Sleepy, no effort to stay awake
    3 = Alert                      8 = Sleepy, some effort to stay awake
    4 = Fairly alert                9 = Extremely sleepy, fighting sleep
    5 = Neither alert nor sleepy
"""

import argparse
import csv
import os
import sys

SENSOR_HEADER = [
    "timestamp_ms", "hr_bpm", "pulse_raw", "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps", "head_movement", "signal_quality",
    "blink_rate", "pitch_deg", "gyro_var", "nod_score", "risk_pct",
    "alert_level", "imu_valid",
]


def parse_args():
    p = argparse.ArgumentParser(description="Attach a KSS label to a recorded session")
    p.add_argument("--session", required=True, help="Path to session directory (e.g. sessions/session_023)")
    p.add_argument("--kss", required=True, type=int, choices=range(1, 10), metavar="1-9",
                   help="Karolinska Sleepiness Scale rating for the whole session")
    p.add_argument("--condition", choices=["rested", "fatigued"], default=None,
                   help="Optional condition tag, for quick filtering later")
    return p.parse_args()


def main():
    args = parse_args()
    session_dir = args.session
    sensor_path = os.path.join(session_dir, "sensor_data.csv")
    out_path = os.path.join(session_dir, "sensor_data_labeled.csv")
    meta_path = os.path.join(session_dir, "metadata.txt")

    if not os.path.isdir(session_dir):
        sys.exit(f"[ERROR] Session directory not found: {session_dir}")
    if not os.path.exists(sensor_path):
        sys.exit(f"[ERROR] sensor_data.csv not found in {session_dir}")

    # ── Detect whether sensor_data.csv already has a header row ─────────────
    with open(sensor_path, "r", newline="") as f:
        first_line = f.readline().strip()
    has_header = first_line.startswith("timestamp_ms") or first_line.startswith("#")

    row_count = 0
    with open(sensor_path, "r", newline="") as fin, open(out_path, "w", newline="") as fout:
        reader = csv.reader(fin)
        writer = csv.writer(fout)
        writer.writerow(SENSOR_HEADER + ["label"])
        for i, row in enumerate(reader):
            if i == 0 and has_header:
                continue  # skip existing header, we just wrote our own
            if not row:
                continue
            writer.writerow(row + [args.kss])
            row_count += 1

    if row_count == 0:
        sys.exit(f"[ERROR] No data rows found in {sensor_path} -- nothing written to {out_path}")

    # ── Record the label in metadata.txt too (append or update in place) ───
    meta_lines = []
    if os.path.exists(meta_path):
        with open(meta_path, "r") as f:
            meta_lines = [l for l in f.read().splitlines()
                          if not l.startswith("kss=") and not l.startswith("condition=")]
    meta_lines.append(f"kss={args.kss}")
    if args.condition:
        meta_lines.append(f"condition={args.condition}")
    with open(meta_path, "w") as f:
        f.write("\n".join(meta_lines) + "\n")

    print(f"[OK] Labeled {row_count} rows with KSS={args.kss}"
          + (f", condition={args.condition}" if args.condition else ""))
    print(f"     Written : {out_path}")
    print(f"     Updated : {meta_path}")
    print(f"     Raw sensor_data.csv left untouched.")


if __name__ == "__main__":
    main()
