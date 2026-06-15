#!/usr/bin/env python3
"""
eye_ear.py — Eye-only EAR detection for helmet-mounted close-up camera
=======================================================================
Designed for the ESP32-S3-CAM helmet setup where the camera is physically
mounted inside the helmet pointing at one eye.

Detection pipeline (Fixed-ROI, Lock-then-Run):
    Phase 1 — LOCK (first ~30 frames):
        Run Haar cascade → average detections → lock bounding box.

    Phase 2 — RUN (all remaining frames):
        Skip Haar cascade entirely. Crop the fixed ROI directly.
        Every 100 frames, run Haar once as a drift check.

    EAR-only blink detection:
        EAR = ellipse minor/major ratio of iris/pupil.
        Uses an adaptive baseline (75th-percentile of last 60 frames).
        Blink fires if EAR drops below 70% of baseline.
        Ignores the first 3 seconds while baseline builds.
"""

import argparse
import csv
import math
import os
import sys

import cv2
import numpy as np


# ── Blink detection defaults ──────────────────────────────────────────────────
EAR_RATIO_THRESH      = 0.70   # EAR < baseline * 0.70  → blink candidate
EAR_DROP_THRESH       = 0.0    # extra: require absolute drop >= this (0 = off)
BLINK_CONSEC_MIN      = 1      # consecutive trigger-frames to count one blink
BLINK_COOLDOWN_FRAMES = 10     # debounce: ignore frames after a blink fires
BASELINE_WINDOW       = 60     # frames used for adaptive baseline (75th pct)
WARMUP_SECS           = 3.0    # skip blink detection for first N seconds

# ── ROI lock parameters ───────────────────────────────────────────────────────
LOCK_FRAMES      = 30    # number of initial frames used to lock the ROI
DRIFT_CHECK_FREQ = 100   # run Haar every N frames to check drift
DRIFT_MAX_PX     = 20    # accept drift correction only if shift < this many px
ROI_MARGIN       = 8     # extra pixels added around the locked bbox for crop

# ── CLAHE defaults ────────────────────────────────────────────────────────────
CLAHE_CLIP_DEFAULT = 3.0
CLAHE_GRID_DEFAULT = 8

# ── Haar cascade defaults ─────────────────────────────────────────────────────
SCALE_FACTOR_DEFAULT  = 1.05
MIN_NEIGHBORS_DEFAULT = 3
MIN_EYE_SIZE          = (20, 10)

EAR_BLINK_THRESH_DEFAULT = 0.22
BLINK_CONSEC_MIN_DEFAULT = 1
EAR_FALLBACK             = -1.0


# ── Image enhancement ─────────────────────────────────────────────────────────

def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

def enhance_frame(bgr: np.ndarray, clahe: cv2.CLAHE) -> np.ndarray:
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l_eq = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l_eq, a, b]), cv2.COLOR_LAB2BGR)


# ── Eye detection (Haar) ──────────────────────────────────────────────────────

def detect_eye(
    gray: np.ndarray,
    cascade: cv2.CascadeClassifier,
    scale_factor: float,
    min_neighbors: int,
) -> tuple | None:
    eyes = cascade.detectMultiScale(
        gray,
        scaleFactor=scale_factor,
        minNeighbors=min_neighbors,
        minSize=MIN_EYE_SIZE,
        flags=cv2.CASCADE_SCALE_IMAGE,
    )
    if len(eyes) == 0:
        return None
    eyes = sorted(eyes, key=lambda e: e[2] * e[3], reverse=True)
    return tuple(eyes[0])


# ── EAR from iris/pupil ellipse fit ──────────────────────────────────────────

def ellipse_ear(gray_crop: np.ndarray) -> float:
    if gray_crop.size == 0:
        return EAR_FALLBACK

    crop_rs = cv2.resize(gray_crop, (64, 64), interpolation=cv2.INTER_LINEAR)
    blur = cv2.GaussianBlur(crop_rs, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return EAR_FALLBACK
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    for cnt in contours:
        if len(cnt) < 5 or cv2.contourArea(cnt) < 30:
            continue
        try:
            (_, _), (minor, major), _ = cv2.fitEllipse(cnt)
        except cv2.error:
            continue
        if major < 1e-6:
            continue
        return float(minor) / float(major)
    return EAR_FALLBACK


# ── Adaptive baseline ─────────────────────────────────────────────────────────

def adaptive_baseline(history: list, window: int = BASELINE_WINDOW) -> float:
    recent = history[-window:] if len(history) >= window else history
    if not recent:
        return 1.0
    sorted_r = sorted(recent)
    idx = int(len(sorted_r) * 0.75)
    return sorted_r[min(idx, len(sorted_r) - 1)]


# ── CSV output schema ─────────────────────────────────────────────────────────

CSV_COLUMNS = [
    "timestamp_ms", "ear", "left_ear", "right_ear",
    "is_blink_frame", "blink_count", "eye_detected",
    "face_detected",
    "eye_x", "eye_y", "eye_w", "eye_h",
]


# ── Core offline processor ────────────────────────────────────────────────────

def process_folder(
    session_path: str,
    clahe_clip: float = CLAHE_CLIP_DEFAULT,
    clahe_grid: int   = CLAHE_GRID_DEFAULT,
    scale_factor: float  = SCALE_FACTOR_DEFAULT,
    min_neighbors: int   = MIN_NEIGHBORS_DEFAULT,
    ear_blink_thresh: float  = EAR_BLINK_THRESH_DEFAULT,
    ear_blink_drop: float    = 0.0,
    blink_consec_min: int    = BLINK_CONSEC_MIN_DEFAULT,
    rotate_180: bool  = False,
    show: bool        = False,
    output_video: bool = False,
    verbose: bool     = False,
    no_enhance: bool  = False,
    max_frames: int | None = None,
) -> None:
    frame_folder = os.path.join(session_path, "frames")
    preview_mode = max_frames is not None
    output_csv   = (
        os.path.join(session_path, "camera_features_preview.csv")
        if preview_mode
        else os.path.join(session_path, "camera_features.csv")
    )

    if not os.path.isdir(frame_folder):
        sys.exit(f"[ERROR] frames/ folder not found: {frame_folder}")

    frame_files = sorted(
        [f for f in os.listdir(frame_folder) if f.lower().endswith(".jpg")],
        key=lambda x: int(os.path.splitext(x)[0]),
    )
    if not frame_files:
        sys.exit(f"[ERROR] No .jpg files in {frame_folder}")
    if preview_mode:
        frame_files = frame_files[:max_frames]

    print(f"[EYE_EAR] Session     : {session_path}")
    print(f"[EYE_EAR] Frames      : {len(frame_files)}")
    print(f"[EYE_EAR] Output CSV  : {output_csv}")
    print(f"[EYE_EAR] Mode        : Fixed-ROI lock-then-run (EAR-only)")
    print(f"[EYE_EAR] Blink       : EAR < {EAR_RATIO_THRESH:.0%} of baseline (skipping first {WARMUP_SECS}s)")

    clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_grid, clahe_grid))
    cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_eye.xml")
    if cascade.empty():
        sys.exit("[ERROR] Could not load haarcascade_eye.xml")

    # ── Phase 1: ROI lock ─────────────────────────────────────────────────────
    lock_detections = []
    lock_frames_to_use = min(LOCK_FRAMES, len(frame_files))

    for fname in frame_files[:lock_frames_to_use]:
        img = cv2.imread(os.path.join(frame_folder, fname))
        if img is None: continue
        if rotate_180: img = cv2.rotate(img, cv2.ROTATE_180)
        img = adjust_gamma(img, gamma=2.0)
        enh  = img if no_enhance else enhance_frame(img, clahe)
        gray = cv2.cvtColor(enh, cv2.COLOR_BGR2GRAY)
        bbox = detect_eye(gray, cascade, scale_factor, min_neighbors)
        if bbox is not None:
            lock_detections.append(bbox)

    if not lock_detections:
        print("[EYE_EAR] WARNING: Could not lock ROI — falling back to per-frame Haar.")
        locked_bbox = None
    else:
        lx = int(np.median([b[0] for b in lock_detections]))
        ly = int(np.median([b[1] for b in lock_detections]))
        lw = int(np.median([b[2] for b in lock_detections]))
        lh = int(np.median([b[3] for b in lock_detections]))
        locked_bbox = (lx, ly, lw, lh)
        print(f"[EYE_EAR] ROI locked  : x={lx} y={ly} w={lw} h={lh}")

    # ── Phase 2: Run ──────────────────────────────────────────────────────────
    blink_count     = 0
    processed       = 0
    detected_count  = 0
    blink_frame_ctr = 0
    video_out       = None
    cooldown_ctr    = 0
    t0_ms           = None

    ear_history   = []
    ear_velocity  = []
    current_bbox  = locked_bbox

    with open(output_csv, mode="w", newline="") as csvf:
        writer = csv.DictWriter(csvf, fieldnames=CSV_COLUMNS)
        writer.writeheader()

        for frame_idx, fname in enumerate(frame_files):
            timestamp_ms = int(os.path.splitext(fname)[0])
            if t0_ms is None:
                t0_ms = timestamp_ms
            
            t_sec = (timestamp_ms - t0_ms) / 1000.0

            img = cv2.imread(os.path.join(frame_folder, fname))
            if img is None: continue
            if rotate_180: img = cv2.rotate(img, cv2.ROTATE_180)
            h, w = img.shape[:2]

            if output_video and video_out is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_out = cv2.VideoWriter(os.path.join(session_path, "processed_video.mp4"), fourcc, 30.0, (w, h))

            enh  = img if no_enhance else enhance_frame(img, clahe)
            gray = cv2.cvtColor(enh, cv2.COLOR_BGR2GRAY)

            if current_bbox is not None and frame_idx > 0 and frame_idx % DRIFT_CHECK_FREQ == 0:
                drift_bbox = detect_eye(gray, cascade, scale_factor, min_neighbors)
                if drift_bbox is not None:
                    shift = math.hypot(drift_bbox[0] - current_bbox[0], drift_bbox[1] - current_bbox[1])
                    if shift < DRIFT_MAX_PX:
                        alpha = 0.3
                        current_bbox = (
                            int(current_bbox[0]*(1-alpha) + drift_bbox[0]*alpha),
                            int(current_bbox[1]*(1-alpha) + drift_bbox[1]*alpha),
                            int(current_bbox[2]*(1-alpha) + drift_bbox[2]*alpha),
                            int(current_bbox[3]*(1-alpha) + drift_bbox[3]*alpha)
                        )

            bbox = current_bbox
            if bbox is None:
                bbox = detect_eye(gray, cascade, scale_factor, min_neighbors)
                if bbox is not None: current_bbox = bbox

            row = {
                "timestamp_ms": timestamp_ms, "ear": "", "left_ear": "", "right_ear": "",
                "is_blink_frame": 0, "blink_count": blink_count, "eye_detected": 0,
                "face_detected": 0, "eye_x": "", "eye_y": "", "eye_w": "", "eye_h": "",
            }

            if bbox is not None:
                ex, ey, ew, eh = bbox
                mx = ROI_MARGIN
                x1 = max(0, ex - mx); y1 = max(0, ey - mx)
                x2 = min(w, ex+ew+mx); y2 = min(h, ey+eh+mx)
                
                ear = ellipse_ear(gray[y1:y2, x1:x2])
                if ear == EAR_FALLBACK: ear = 0.0

                ear_base = adaptive_baseline(ear_history)
                ear_history.append(ear)

                ear_velocity.append(ear)
                if len(ear_velocity) > 4: ear_velocity.pop(0)
                ear_drop = max(ear_velocity) - ear if len(ear_velocity) > 0 else 0.0

                # Blink logic
                is_blink_frame = 0
                if t_sec >= WARMUP_SECS:
                    ear_trigger = (ear < ear_base * EAR_RATIO_THRESH)
                    drop_ok = (ear_drop >= ear_blink_drop)
                    is_blink_frame = int(ear_trigger and drop_ok)

                if cooldown_ctr > 0:
                    cooldown_ctr -= 1
                    is_blink_frame = 0
                    blink_frame_ctr = 0
                else:
                    if is_blink_frame:
                        blink_frame_ctr += 1
                    else:
                        if blink_frame_ctr >= blink_consec_min:
                            blink_count += 1
                            cooldown_ctr = BLINK_COOLDOWN_FRAMES
                        blink_frame_ctr = 0

                row.update({
                    "ear": round(ear, 4), "is_blink_frame": is_blink_frame,
                    "blink_count": blink_count, "eye_detected": 1, "face_detected": 1,
                    "eye_x": ex, "eye_y": ey, "eye_w": ew, "eye_h": eh,
                })
                detected_count += 1

                if show or output_video:
                    disp = enh.copy()
                    color = (0, 0, 255) if is_blink_frame else (0, 255, 100)
                    cv2.rectangle(disp, (ex, ey), (ex+ew, ey+eh), color, 2)
                    cv2.putText(disp, f"EAR:{ear:.2f}", (ex, ey - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
                    if is_blink_frame:
                        cv2.putText(disp, "BLINK", (ex, ey - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    elif t_sec < WARMUP_SECS:
                        cv2.putText(disp, "WARMUP", (ex, ey - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    
                    cv2.putText(disp, f"Blinks:{blink_count}", (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
                    cv2.putText(disp, f"t={t_sec:.1f}s", (6, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
                    if output_video and video_out is not None: video_out.write(disp)

            writer.writerow(row)
            processed += 1

            if processed % 200 == 0:
                print(f"[EYE_EAR] Progress: {processed}/{len(frame_files)} ...")

    if video_out is not None:
        video_out.release()
    print(f"\n[EYE_EAR] Total blinks : {blink_count} (Saved to {output_csv})")

# ── Preview contact sheet ─────────────────────────────────────────────────────

def _save_preview_sheet(session_path, frame_folder, csv_path, clahe_clip, clahe_grid, scale_factor, min_neighbors, rotate_180):
    pass 

# ── Entry point ───────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Eye-only EAR blink detection for the ESP32-S3-CAM helmet.\n"
            "Fixed-ROI lock-then-run: Haar runs once at startup to lock the eye\n"
            "position, then fast fixed-crop processing for all remaining frames."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--session", required=True, metavar="SESSION_DIR",
                   help="Path to session directory (must contain frames/ subfolder).")
    p.add_argument("--preview", nargs="?", const=100, type=int, metavar="N",
                   help="Preview mode: process first N frames and save preview_sheet.jpg.")
    p.add_argument("--clahe-clip", type=float, default=CLAHE_CLIP_DEFAULT, metavar="N")
    p.add_argument("--clahe-grid", type=int,   default=CLAHE_GRID_DEFAULT, metavar="N")
    p.add_argument("--scale-factor",  type=float, default=SCALE_FACTOR_DEFAULT,  metavar="F")
    p.add_argument("--min-neighbors", type=int,   default=MIN_NEIGHBORS_DEFAULT,  metavar="N")
    p.add_argument("--ear-blink-thresh", type=float, default=EAR_BLINK_THRESH_DEFAULT, metavar="F",
                   help="Legacy: absolute EAR threshold (adaptive ratio is now preferred).")
    p.add_argument("--ear-blink-drop",   type=float, default=0.0, metavar="F",
                   help="Minimum EAR velocity drop required to count a blink (default: 0.0).")
    p.add_argument("--blink-consec-min", type=int, default=BLINK_CONSEC_MIN_DEFAULT, metavar="N")
    p.add_argument("--rotate-180", action="store_true",
                   help="Rotate 180 degrees before processing (upside-down camera).")
    p.add_argument("--show",         action="store_true", help="Show live preview window.")
    p.add_argument("--output-video", action="store_true", help="Save processed_video.mp4.")
    p.add_argument("--no-enhance",   action="store_true", help="Skip CLAHE enhancement.")
    p.add_argument("--verbose",      action="store_true", help="Print one line per frame.")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    if not os.path.isdir(args.session):
        sys.exit(f"[ERROR] Session directory not found: {args.session}")

    process_folder(
        session_path=args.session,
        clahe_clip=args.clahe_clip,
        clahe_grid=args.clahe_grid,
        scale_factor=args.scale_factor,
        min_neighbors=args.min_neighbors,
        ear_blink_thresh=args.ear_blink_thresh,
        ear_blink_drop=args.ear_blink_drop,
        blink_consec_min=args.blink_consec_min,
        rotate_180=args.rotate_180,
        show=args.show,
        output_video=args.output_video,
        verbose=args.verbose,
        no_enhance=args.no_enhance,
        max_frames=args.preview,
    )


if __name__ == "__main__":
    main()

