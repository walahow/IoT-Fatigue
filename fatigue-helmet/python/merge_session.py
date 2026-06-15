#!/usr/bin/env python3
"""
merge_session.py — Merge camera EAR features with sensor CSV by nearest timestamp.
====================================================================================
Combines:
  - sensor_data.csv    (1 Hz, from ESP32 SD card — LOOP_MS=1000 in firmware)
  - camera_features.csv (30 FPS, from ear_validation.py --input)

into a single dataset_merged.csv aligned by nearest timestamp_ms.
The label column is left empty for manual annotation using KSS (1-9) after the session.

Requires: pandas
    pip install pandas

Usage:
    python merge_session.py --session sessions/session_001
    python merge_session.py --session sessions/session_001 --tolerance 150

Sync accuracy note:
    Sensor logs at 1 Hz (1000ms intervals).
    Camera captures at ~17-20 FPS.
    Default tolerance=100ms: each sensor row is matched to the closest camera
    frame within ±50ms — well within any meaningful fatigue detection window.
"""

import argparse
import os
import sys

try:
    import pandas as pd
except ImportError:
    sys.exit("[ERROR] pandas not installed. Run: pip install pandas")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Merge camera_features.csv + sensor_data.csv by nearest timestamp"
    )
    p.add_argument("--session",   required=True,
                   help="Path to session directory (e.g. sessions/session_001)")
    p.add_argument("--tolerance", type=int, default=100,
                   help="Max ms gap for a valid match (default: 100 ms)")
    return p.parse_args()


def merge(session_path: str, tolerance_ms: int) -> None:
    sensor_path  = os.path.join(session_path, "sensor_data.csv")
    camera_path  = os.path.join(session_path, "camera_features.csv")
    output_path  = os.path.join(session_path, "dataset_merged.csv")

    # ── Pre-flight checks ────────────────────────────────────────────────────
    for path, label in [(sensor_path, "sensor_data.csv"),
                         (camera_path, "camera_features.csv")]:
        if not os.path.exists(path):
            sys.exit(
                f"[ERROR] {label} not found in {session_path}\n"
                f"        {'Run ear_validation.py --input first.' if label == 'camera_features.csv' else 'Check SD card data.'}"
            )

    # ── Load CSVs ────────────────────────────────────────────────────────────
    expected_sensor_cols = [
        "timestamp_ms", "hr_bpm", "pulse_raw",
        "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
        "head_movement", "signal_quality"
    ]
    
    # Read the first line to check if it contains headers
    with open(sensor_path, 'r') as f:
        first_line = f.readline()
    
    # If the first line starts with a digit or minus sign (data), it has no header
    has_header = True
    if first_line and (first_line.strip()[0].isdigit() or first_line.strip()[0] == '-'):
        has_header = False
        
    if has_header:
        sensor_df = pd.read_csv(sensor_path)
    else:
        print("[MERGE] sensor_data.csv appears to be headerless. Assigning default headers.")
        sensor_df = pd.read_csv(sensor_path, names=expected_sensor_cols)
        
    camera_df = pd.read_csv(camera_path)

    if "timestamp_ms" not in sensor_df.columns:
        sys.exit("[ERROR] sensor_data.csv has no 'timestamp_ms' column.")
    if "timestamp_ms" not in camera_df.columns:
        sys.exit("[ERROR] camera_features.csv has no 'timestamp_ms' column.")

    # ── Sort by timestamp (required for merge_asof) ──────────────────────────
    sensor_df = sensor_df.sort_values("timestamp_ms").reset_index(drop=True)
    camera_df = camera_df.sort_values("timestamp_ms").reset_index(drop=True)

    print(f"[MERGE] Sensor rows  : {len(sensor_df)}  "
          f"(~{len(sensor_df) / 1 / 60:.1f} min at 1 Hz)")
    print(f"[MERGE] Camera frames: {len(camera_df)}  "
          f"(~{len(camera_df) / 30 / 60:.1f} min at 30 FPS)")
    print(f"[MERGE] Tolerance    : +/-{tolerance_ms // 2} ms  "
          f"(reject matches > {tolerance_ms} ms apart)")

    # ── Nearest-timestamp merge ───────────────────────────────────────────────
    # For each sensor row, find the closest camera frame within tolerance_ms.
    # Rows with no camera match within tolerance are kept but camera columns = NaN.
    merged = pd.merge_asof(
        sensor_df,
        camera_df,
        on="timestamp_ms",
        direction="nearest",
        tolerance=tolerance_ms,
        suffixes=("_sensor", "_camera")
    )

    # ── Add empty label column for manual KSS annotation ─────────────────────
    merged["label"] = ""

    # ── Write output ─────────────────────────────────────────────────────────
    merged.to_csv(output_path, index=False)

    matched   = merged["ear"].notna().sum() if "ear" in merged.columns else "N/A"
    unmatched = len(merged) - int(matched) if not isinstance(matched, str) else len(merged)

    print(f"\n[MERGE] Merged rows    : {len(merged)}")
    print(f"[MERGE] Camera-matched : {matched}")
    print(f"[MERGE] No camera match: {unmatched}  "
          f"(timestamp gap > {tolerance_ms} ms - camera columns = NaN)")
    print(f"\n[MERGE] Output -> {output_path}")
    print(f"\n[NEXT]  Open {output_path} and fill the 'label' column")
    print(f"        using KSS (Karolinska Sleepiness Scale, 1-9).")


def main() -> None:
    args = parse_args()
    merge(args.session, args.tolerance)


if __name__ == "__main__":
    main()
