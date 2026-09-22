#!/usr/bin/env python3
"""
frames_to_mp4.py — Convert unpacked JPEG frames from a session into an MP4 video.
==================================================================================
Reads frames from sessions/<session>/frames/*.jpg (named by timestamp_ms).
Uses metadata.txt for FPS; falls back to 30 fps if missing.

Handles non-uniform timestamps (gaps / dropped frames) by duplicating the last
frame to fill silent gaps, so playback timing matches real recording time.

Usage:
    python frames_to_mp4.py --session D:/path/to/session_001
    python frames_to_mp4.py --session D:/path/to/session_001 --fps 30
    python frames_to_mp4.py --session D:/path/to/session_001 --no-timestamp
    python frames_to_mp4.py --session D:/path/to/session_001 --scale 2
    python frames_to_mp4.py --session D:/path/to/session_001 --scale 3
    python frames_to_mp4.py --session D:/path/to/session_001 --out my_video.mp4

Output:
    <session_dir>/video.mp4  (default)
"""

import argparse
import os
import sys
import glob

import cv2
import numpy as np
import pandas as pd

# ── Colour / font constants ───────────────────────────────────────────────────
OSD_FONT       = cv2.FONT_HERSHEY_SIMPLEX
OSD_SCALE      = 0.45
OSD_THICKNESS  = 1
OSD_COLOR      = (255, 255, 255)   # white text
OSD_SHADOW     = (0, 0, 0)         # black shadow for readability
OSD_MARGIN     = 6                 # pixels from edge
# ─────────────────────────────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Convert session frames to MP4"
    )
    p.add_argument("--session", required=True,
                   help="Path to session directory (e.g. sessions/session_001)")
    p.add_argument("--fps", type=float, default=None,
                   help="Output FPS (default: read from metadata.txt or 30)")
    p.add_argument("--out", default=None,
                   help="Output MP4 path (default: <session>/video.mp4)")
    p.add_argument("--scale", type=float, default=2.0,
                   help="Upscale factor (default: 2 = 640x480). Use 1 for native 320x240.")
    p.add_argument("--no-timestamp", action="store_true",
                   help="Disable timestamp OSD overlay")
    p.add_argument("--overlay-data", action="store_true",
                   help="Overlay sensor data HUD at the bottom of the video")
    p.add_argument("--no-fill-gaps", action="store_true",
                   help="Skip duplicate-frame gap filling; just encode frames as-is")
    p.add_argument("--codec", default="avc1",
                   help="FourCC codec string (default: avc1 H.264). Fallback: mp4v.")
    p.add_argument("--rotate", type=int, choices=[0, 90, 180, 270], default=0,
                   help="Rotate frames by N degrees clockwise before processing.")
    return p.parse_args()


def read_metadata(session_path: str) -> dict:
    """Parse metadata.txt into a key=value dict."""
    meta = {}
    meta_path = os.path.join(session_path, "metadata.txt")
    if os.path.exists(meta_path):
        with open(meta_path, "r") as f:
            for line in f:
                line = line.strip()
                if "=" in line:
                    k, v = line.split("=", 1)
                    meta[k.strip()] = v.strip()
    return meta


def collect_frames(frames_dir: str) -> list[tuple[int, str]]:
    """
    Return sorted list of (timestamp_ms, filepath) from frames/*.jpg.
    Filenames must be <timestamp_ms>.jpg.
    """
    pattern = os.path.join(frames_dir, "*.jpg")
    paths = glob.glob(pattern)
    if not paths:
        sys.exit(f"[ERROR] No .jpg frames found in {frames_dir}")

    result = []
    for p in paths:
        stem = os.path.splitext(os.path.basename(p))[0]
        try:
            ts = int(stem)
        except ValueError:
            print(f"[WARN] Skipping non-numeric filename: {p}")
            continue
        result.append((ts, p))

    result.sort(key=lambda x: x[0])
    return result


def draw_osd(frame, timestamp_ms: int, frame_num: int, total: int):
    """Draw a small timestamp / frame counter overlay."""
    secs    = timestamp_ms / 1000.0
    minutes = int(secs) // 60
    seconds = secs - minutes * 60
    text    = f"{minutes:02d}:{seconds:05.2f}  [{frame_num}/{total}]"

    # Shadow pass
    cv2.putText(frame, text,
                (OSD_MARGIN + 1, OSD_MARGIN + 15 + 1),
                OSD_FONT, OSD_SCALE, OSD_SHADOW, OSD_THICKNESS + 1,
                cv2.LINE_AA)
    # Main text
    cv2.putText(frame, text,
                (OSD_MARGIN, OSD_MARGIN + 15),
                OSD_FONT, OSD_SCALE, OSD_COLOR, OSD_THICKNESS,
                cv2.LINE_AA)


def load_session_data(session_path: str):
    merged_path = os.path.join(session_path, "dataset_merged.csv")
    if os.path.exists(merged_path):
        return pd.read_csv(merged_path).sort_values("timestamp_ms")
    
    sensor_path = os.path.join(session_path, "sensor_data.csv")
    if os.path.exists(sensor_path):
        df = pd.read_csv(sensor_path)
        if len(df.columns) > 0 and df.columns[0].startswith("#HEADER:"):
            df.columns = [c.replace("#HEADER:", "").strip() for c in df.columns]
        return df.sort_values("timestamp_ms")
    
    return None


def draw_data_hud(hud, ts, df):
    h, w = hud.shape[:2]
    cv2.rectangle(hud, (0, 0), (w, h), (20, 20, 25), -1) 
    
    idx = np.searchsorted(df['timestamp_ms'].values, ts)
    if idx >= len(df):
        idx = len(df) - 1
    elif idx > 0 and abs(ts - df['timestamp_ms'].iloc[idx-1]) < abs(df['timestamp_ms'].iloc[idx] - ts):
        idx = idx - 1
        
    row = df.iloc[idx]
    
    # 1. Physiology (Left)
    hr = row.get("hr_bpm", 0)
    sq = row.get("signal_quality", 0)
    cv2.putText(hud, f"HR: {hr:.0f} BPM", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    sq_col = (0, 255, 0) if sq == 1 else (0, 0, 255)
    cv2.putText(hud, f"SQ: {'OK' if sq==1 else 'BAD'}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, sq_col, 1)
    
    # 2. IMU State (Center-Left)
    pitch = row.get("pitch_deg", 0)
    gyro = row.get("gyro_var", 0)
    nod = row.get("nod_score", 0)
    cv2.putText(hud, f"Pitch: {pitch:.1f} deg", (150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(hud, f"GyroVar: {gyro:.0f}", (150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(hud, f"Nod: {nod:.2f}", (150, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # 3. Fuzzy Logic Output (Right)
    risk = row.get("risk_pct", 0)
    alert = row.get("alert_level", 0)
    if alert == 2:
        alert_str = "CRITICAL"
        alert_col = (0, 0, 255) 
    elif alert == 1:
        alert_str = "WARNING"
        alert_col = (0, 255, 255)
    else:
        alert_str = "SAFE"
        alert_col = (0, 255, 0)
        
    cv2.putText(hud, f"RISK: {risk:.1f}%", (w - 200, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
    cv2.putText(hud, f"{alert_str}", (w - 200, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, alert_col, 2)
    
    lbl = row.get("label", "")
    if pd.notna(lbl) and str(lbl).strip():
        cv2.putText(hud, f"LBL: {lbl}", (w - 200, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,100,100), 2)

    # 4. Eye Blinking Chart (Center-Right)
    chart_x1 = 280
    chart_y1 = 15
    chart_x2 = w - 220
    chart_y2 = h - 15
    cv2.rectangle(hud, (chart_x1, chart_y1), (chart_x2, chart_y2), (40, 40, 40), -1)
    
    start_idx = max(0, idx - 60)
    window = df.iloc[start_idx:idx+1]
    
    if len(window) > 1:
        blink_max = 30.0
        def get_pt(val, i, max_val):
            x = chart_x1 + int((i / (len(window) - 1)) * (chart_x2 - chart_x1))
            y = chart_y2 - int(min(1.0, max(0.0, val / max_val)) * (chart_y2 - chart_y1))
            return (x, y)
        
        br_pts = []
        for i in range(len(window)):
            v = window.iloc[i].get("blink_rate", 0)
            br_pts.append(get_pt(v, i, blink_max))
        
        cv2.polylines(hud, [np.array(br_pts, dtype=np.int32)], False, (255, 200, 0), 2)
        cv2.putText(hud, "Blink", (chart_x1+5, chart_y1+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 0), 1)

        if "ear" in window.columns and window["ear"].notna().any():
            ear_max = 0.5
            ear_pts = []
            for i in range(len(window)):
                v = window.iloc[i].get("ear", 0)
                if pd.isna(v): v = 0
                ear_pts.append(get_pt(v, i, ear_max))
            cv2.polylines(hud, [np.array(ear_pts, dtype=np.int32)], False, (0, 200, 255), 1)
            cv2.putText(hud, "EAR", (chart_x1+5, chart_y1+30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)


def draw_pip_ellipse(img, row, scale):
    """Draw a black and white PiP at the top right showing the thresholded eye and fitted ellipse."""
    if "eye_x" not in row or pd.isna(row["eye_x"]) or str(row["eye_x"]).strip() == "":
        return
    
    try:
        ex = int(float(row["eye_x"]) * scale)
        ey = int(float(row["eye_y"]) * scale)
        ew = int(float(row["eye_w"]) * scale)
        eh = int(float(row["eye_h"]) * scale)
        mx = int(8 * scale) # ROI margin
    except (ValueError, TypeError):
        return
        
    h, w = img.shape[:2]
    x1 = max(0, ex - mx)
    y1 = max(0, ey - mx)
    x2 = min(w, ex + ew + mx)
    y2 = min(h, ey + eh + mx)
    
    crop = img[y1:y2, x1:x2]
    if crop.size == 0:
        return
        
    gray_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    
    # Process like eye_ear.py
    crop_rs = cv2.resize(gray_crop, (64, 64), interpolation=cv2.INTER_LINEAR)
    blur = cv2.GaussianBlur(crop_rs, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    
    thresh_bgr = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for cnt in contours:
            if len(cnt) < 5 or cv2.contourArea(cnt) < 30:
                continue
            try:
                ellipse = cv2.fitEllipse(cnt)
                if ellipse[1][1] < 1e-6:
                    continue
                cv2.ellipse(thresh_bgr, ellipse, (0, 0, 255), 2)
                break
            except cv2.error:
                continue

    # Scale the 64x64 up to e.g. 128x128 for PiP (scaled by video scale)
    pip_size = int(64 * scale)
    pip = cv2.resize(thresh_bgr, (pip_size, pip_size), interpolation=cv2.INTER_NEAREST)
    
    cv2.rectangle(pip, (0, 0), (pip_size-1, pip_size-1), (255, 255, 255), 1)
    
    # Overlay on top-right
    pip_y = 10
    pip_x = w - pip_size - 10
    
    # ensure it fits
    if pip_y + pip_size <= h and pip_x >= 0:
        img[pip_y:pip_y+pip_size, pip_x:pip_x+pip_size] = pip


def encode(session_path: str, fps: float, out_path: str, scale: float,
           show_timestamp: bool, fill_gaps: bool, codec: str, overlay_data: bool, rotate_deg: int) -> None:  # noqa

    frames_dir = os.path.join(session_path, "frames")
    if not os.path.isdir(frames_dir):
        sys.exit(f"[ERROR] frames/ directory not found in {session_path}\n"
                 f"       Run unpack_session.py first.")

    frames = collect_frames(frames_dir)
    total  = len(frames)
    print(f"[INFO] Found {total} frames in {frames_dir}")

    # ── Determine frame size from first frame ─────────────────────────────────
    first_img = cv2.imread(frames[0][1])
    if first_img is None:
        sys.exit(f"[ERROR] Cannot read first frame: {frames[0][1]}")
        
    if rotate_deg == 90: first_img = cv2.rotate(first_img, cv2.ROTATE_90_CLOCKWISE)
    elif rotate_deg == 180: first_img = cv2.rotate(first_img, cv2.ROTATE_180)
    elif rotate_deg == 270: first_img = cv2.rotate(first_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
    h, w = first_img.shape[:2]

    # ── Apply upscale ─────────────────────────────────────────────────────────
    out_w = int(w * scale)
    out_h = int(h * scale)
    # Round to even dimensions (required by most codecs)
    out_w += out_w % 2
    out_h += out_h % 2
    interp = cv2.INTER_LANCZOS4 if scale > 1.0 else cv2.INTER_AREA
    print(f"[INFO] Source size : {w}x{h}")
    print(f"[INFO] Output size : {out_w}x{out_h}  (scale={scale}x, Lanczos4)")
    print(f"[INFO] Target FPS  : {fps:.1f}")
    print(f"[INFO] Output      : {out_path}")
    print(f"[INFO] Gap fill    : {'yes' if fill_gaps else 'no'}")

    session_data = None
    hud_h = 0
    if overlay_data:
        session_data = load_session_data(session_path)
        if session_data is not None:
            hud_h = 120
            print(f"[INFO] HUD Overlay : Enabled ({len(session_data)} data rows)")
        else:
            print(f"[WARN] HUD Overlay : Failed (No CSV data found)")

    # ── Set up VideoWriter ────────────────────────────────────────────────────
    fourcc = cv2.VideoWriter_fourcc(*codec)
    writer = cv2.VideoWriter(out_path, fourcc, fps, (out_w, out_h + hud_h))
    if not writer.isOpened():
        # avc1 may not be available on all Windows builds; fall back to mp4v
        print(f"[WARN] Codec '{codec}' unavailable, falling back to mp4v.")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        # Same frame size as the first attempt -- every frame written below is
        # (out_w, out_h + hud_h) whenever the HUD is on, so a writer opened for
        # out_h alone silently mismatches every write() call.
        writer = cv2.VideoWriter(out_path, fourcc, fps, (out_w, out_h + hud_h))
    if not writer.isOpened():
        sys.exit(f"[ERROR] VideoWriter failed to open with both avc1 and mp4v.")

    ms_per_frame = 1000.0 / fps

    # ── Encode ────────────────────────────────────────────────────────────────
    written       = 0
    gap_filled    = 0
    prev_frame    = None
    prev_ts       = frames[0][0]

    for idx, (ts, path) in enumerate(frames):
        img = cv2.imread(path)
        if img is None:
            print(f"[WARN] Cannot read {path} - skipping.")
            continue

        if rotate_deg == 90: img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        elif rotate_deg == 180: img = cv2.rotate(img, cv2.ROTATE_180)
        elif rotate_deg == 270: img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Upscale
        if scale != 1.0:
            img = cv2.resize(img, (out_w, out_h), interpolation=interp)

        # Draw bounding box if we have session data
        if session_data is not None:
            # row_idx, not idx: idx is the outer enumerate() counter the end-of-loop
            # progress print and gap-fill logic both key off -- reusing the name here
            # silently clobbered it, so "encoded N/total" printed the same N many times
            # in a row (once per source frame sharing that 1 Hz sensor row) rather than
            # counting frames. Cosmetic only: writer.write() below never read idx.
            row_idx = np.searchsorted(session_data['timestamp_ms'].values, ts)
            if row_idx >= len(session_data):
                row_idx = len(session_data) - 1
            elif row_idx > 0 and abs(ts - session_data['timestamp_ms'].iloc[row_idx-1]) < abs(session_data['timestamp_ms'].iloc[row_idx] - ts):
                row_idx = row_idx - 1
            row = session_data.iloc[row_idx]
            
            if "eye_x" in row and pd.notna(row["eye_x"]) and row["eye_x"] != "":
                try:
                    ex = int(float(row["eye_x"]) * scale)
                    ey = int(float(row["eye_y"]) * scale)
                    ew = int(float(row["eye_w"]) * scale)
                    eh = int(float(row["eye_h"]) * scale)
                    
                    is_blink = int(row.get("is_blink_frame", 0)) == 1
                    color = (0, 0, 255) if is_blink else (0, 255, 100)
                    cv2.rectangle(img, (ex, ey), (ex+ew, ey+eh), color, 2)
                    
                    ear = row.get("ear", 0.0)
                    cv2.putText(img, f"EAR:{float(ear):.2f}", (ex, max(0, ey - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45 * scale, color, max(1, int(1*scale)))
                    if is_blink:
                        cv2.putText(img, "BLINK", (ex, max(0, ey - int(24*scale))), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * scale, (0, 0, 255), max(1, int(2*scale)))
                except (ValueError, TypeError):
                    pass
            
            draw_pip_ellipse(img, row, scale)

        # Build HUD if needed
        if hud_h > 0 and session_data is not None:
            hud_canvas = np.zeros((hud_h, out_w, 3), dtype=np.uint8)
            draw_data_hud(hud_canvas, ts, session_data)
            img = np.vstack((img, hud_canvas))

        # Fill timing gap with previous frame duplicated
        if fill_gaps and prev_frame is not None:
            gap_ms       = ts - prev_ts
            extra_frames = max(0, round(gap_ms / ms_per_frame) - 1)
            if extra_frames > 0:
                for _ in range(extra_frames):
                    fill_img = prev_frame.copy()
                    if show_timestamp:
                        draw_osd(fill_img, prev_ts, written + 1, total)
                    writer.write(fill_img)
                    written     += 1
                    gap_filled  += 1

        if show_timestamp:
            draw_osd(img, ts, written + 1, total)

        writer.write(img)
        written    += 1
        prev_frame  = img
        prev_ts     = ts

        if (idx + 1) % 500 == 0 or (idx + 1) == total:
            print(f"  ... encoded {idx + 1}/{total} source frames "
                  f"({written} total written, {gap_filled} gap-filled)")

    writer.release()
    print(f"\n[DONE] Wrote {written} frames -> {out_path}")
    if gap_filled:
        print(f"       ({gap_filled} duplicate frames inserted to fill timestamp gaps)")
    file_mb = os.path.getsize(out_path) / (1024 * 1024)
    print(f"       File size : {file_mb:.1f} MB")


def main() -> None:
    args = parse_args()

    session_path = os.path.abspath(args.session)
    if not os.path.isdir(session_path):
        sys.exit(f"[ERROR] Session directory not found: {session_path}")

    # ── Resolve FPS ───────────────────────────────────────────────────────────
    fps = args.fps
    if fps is None:
        meta = read_metadata(session_path)
        raw  = meta.get("camera_fps", "30")
        try:
            fps = float(raw)
        except ValueError:
            fps = 30.0
            print(f"[WARN] Cannot parse camera_fps={raw!r}; defaulting to 30")
    print(f"[INFO] Session    : {session_path}")

    # ── Resolve output path ───────────────────────────────────────────────────
    out_path = args.out if args.out else os.path.join(session_path, "video.mp4")

    encode(
        session_path   = session_path,
        fps            = fps,
        out_path       = out_path,
        scale          = args.scale,
        show_timestamp = not args.no_timestamp,
        fill_gaps      = not args.no_fill_gaps,
        codec          = args.codec,
        overlay_data   = args.overlay_data,
        rotate_deg     = args.rotate,
    )


if __name__ == "__main__":
    main()
