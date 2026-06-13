#!/usr/bin/env python3
"""
eye_ear.py — Eye-only EAR detection for helmet-mounted close-up camera
=======================================================================
Designed for the ESP32-S3-CAM helmet setup where the camera is physically
mounted inside the helmet pointing at one eye.  The recording workflow is:

    1. ESP32-S3 records MJPEG + sensor CSV to microSD card.
    2. SD card is copied to PC.
    3. unpack_session.py extracts individual JPEG frames.
    4. eye_ear.py processes frames offline → camera_features.csv
    5. merge_session.py joins sensor + camera data → dataset_merged.csv

Detection pipeline:
    Frame → CLAHE contrast enhance → Haar eye cascade → bbox EAR → CSV

EAR approximation:
    EAR ≈ eye_bbox_height / eye_bbox_width
    This ratio drops when the eye closes (blink) — sufficient for fatigue
    detection without needing 6-point landmark fitting.

Output:
    Same camera_features.csv schema as ear_validation.py — fully compatible
    with merge_session.py.

Usage:
    # Process all frames in a session
    python eye_ear.py --session sessions/session_001
    python eye_ear.py --session sessions/session_001 --show
    python eye_ear.py --session sessions/session_001 --verbose

    # Quick preview: process only first N frames to check camera positioning
    # Record a short 5-10 second test clip, copy to PC, then run:
    python eye_ear.py --session sessions/session_001 --preview
    python eye_ear.py --session sessions/session_001 --preview 50

    # Tune detection sensitivity
    python eye_ear.py --session sessions/session_001 --clahe-clip 4.0
    python eye_ear.py --session sessions/session_001 --scale-factor 1.03 --min-neighbors 2
"""

import argparse
import csv
import os
import sys

import cv2
import numpy as np


# ── EAR / blink thresholds ────────────────────────────────────────────────────
# These are tuned for bbox-ratio EAR (not landmark EAR).
# Landmark EAR typically sits around 0.25–0.30 open; bbox-ratio is
# geometry-dependent but typically 0.30–0.50 open for a close-up eye.
EAR_BLINK_THRESH  = 0.20   # bbox ratio below this → blink frame
BLINK_CONSEC_MIN  = 2      # consecutive blink-frames to count one blink

# ── CLAHE defaults ────────────────────────────────────────────────────────────
CLAHE_CLIP_DEFAULT = 3.0
CLAHE_GRID_DEFAULT = 8

# ── Haar cascade defaults ─────────────────────────────────────────────────────
SCALE_FACTOR_DEFAULT  = 1.05
MIN_NEIGHBORS_DEFAULT = 3
MIN_EYE_SIZE          = (20, 10)   # (width, height) in pixels — filters tiny noise


# ── Image enhancement ─────────────────────────────────────────────────────────

def enhance_frame(bgr: np.ndarray, clahe: cv2.CLAHE) -> np.ndarray:
    """
    Apply CLAHE contrast enhancement to the L-channel (LAB colorspace).
    Returns an enhanced BGR image for detection.
    """
    lab  = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l_eq = clahe.apply(l)
    lab_eq = cv2.merge([l_eq, a, b])
    return cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)


# ── Eye detection ─────────────────────────────────────────────────────────────

def detect_eye(
    gray: np.ndarray,
    cascade: cv2.CascadeClassifier,
    scale_factor: float,
    min_neighbors: int,
) -> tuple[int, int, int, int] | None:
    """
    Run Haar cascade on a grayscale image.
    Returns the bounding box (x, y, w, h) of the largest detected eye,
    or None if no eye was found.
    """
    eyes = cascade.detectMultiScale(
        gray,
        scaleFactor=scale_factor,
        minNeighbors=min_neighbors,
        minSize=MIN_EYE_SIZE,
        flags=cv2.CASCADE_SCALE_IMAGE,
    )
    if len(eyes) == 0:
        return None
    # Pick the largest eye by area (most likely the real one)
    eyes = sorted(eyes, key=lambda e: e[2] * e[3], reverse=True)
    return tuple(eyes[0])


# -- EAR from iris/pupil ellipse fit --------------------------------------------
# Haar cascade always outputs square bboxes so bbox h/w == 1.0 always.
# Instead: crop to the detected eye region, threshold to isolate the dark
# pupil/iris, fit an ellipse to the largest dark contour, and use
#   EAR = ellipse_minor_axis / ellipse_major_axis
# This ratio is 0.4-0.7 when eyes are open and drops toward 0 on a blink.

EAR_BLINK_THRESH_DEFAULT = 0.15   # below this = eye very closed / blink
BLINK_CONSEC_MIN_DEFAULT = 2      # consecutive blink-frames to count one blink
EAR_FALLBACK             = -1.0   # returned when ellipse fit fails


def ellipse_ear(gray_full: np.ndarray, bbox: tuple) -> float:
    """
    Fit an ellipse to the dark iris/pupil inside the Haar-detected eye bbox.
    Returns minor/major axis ratio (0..1), or EAR_FALLBACK on failure.

    Algorithm:
        1. Crop to bbox with a small margin.
        2. Resize crop to a fixed 64x64 for consistent processing.
        3. Otsu threshold (dark regions = iris/pupil).
        4. Find contours; pick the largest that looks like an eye (not tiny noise).
        5. Fit ellipse; return minor/major ratio.
    """
    x, y, w, h = bbox
    img_h, img_w = gray_full.shape[:2]

    # Expand crop slightly for context
    margin = max(4, w // 8)
    x1 = max(0, x - margin)
    y1 = max(0, y - margin)
    x2 = min(img_w, x + w + margin)
    y2 = min(img_h, y + h + margin)
    crop = gray_full[y1:y2, x1:x2]

    if crop.size == 0:
        return EAR_FALLBACK

    # Resize to fixed size for stable contour detection
    crop_rs = cv2.resize(crop, (64, 64), interpolation=cv2.INTER_LINEAR)

    # Invert + Otsu: makes dark iris/pupil into bright foreground
    _, thresh = cv2.threshold(crop_rs, 0, 255,
                              cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # Morphological close to fill small holes in iris
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return EAR_FALLBACK

    # Pick the largest contour that has enough points for ellipse fitting
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    for cnt in contours:
        if len(cnt) < 5:          # fitEllipse needs >= 5 points
            continue
        area = cv2.contourArea(cnt)
        if area < 30:             # too small = noise
            continue
        try:
            (cx, cy), (minor, major), angle = cv2.fitEllipse(cnt)
        except cv2.error:
            continue
        if major < 1e-6:
            continue
        return float(minor) / float(major)   # 0..1, drops on blink

    return EAR_FALLBACK


# ── CSV output schema (compatible with merge_session.py) ─────────────────────

CSV_COLUMNS = [
    "timestamp_ms", "ear", "left_ear", "right_ear",
    "is_blink_frame", "blink_count", "eye_detected",
    # Note: 'face_detected' renamed to 'eye_detected' for accuracy;
    # we also write 'face_detected' as an alias for backward compat.
    "face_detected",
]


# ── Core offline processor ────────────────────────────────────────────────────

def process_folder(
    session_path: str,
    clahe_clip: float,
    clahe_grid: int,
    scale_factor: float,
    min_neighbors: int,
    ear_blink_thresh: float,
    blink_consec_min: int,
    rotate_180: bool,
    show: bool,
    verbose: bool,
    max_frames: int | None = None,
) -> None:
    """
    Process JPEG frames in <session_path>/frames/ and write
    <session_path>/camera_features.csv.
    If max_frames is set, only the first max_frames are processed (preview mode).
    """
    frame_folder = os.path.join(session_path, "frames")
    preview_mode = max_frames is not None
    output_csv   = (
        os.path.join(session_path, "camera_features_preview.csv")
        if preview_mode
        else os.path.join(session_path, "camera_features.csv")
    )

    if not os.path.isdir(frame_folder):
        sys.exit(f"[ERROR] frames/ folder not found: {frame_folder}")

    # Collect and sort frames by timestamp (integer filename stem)
    frame_files = sorted(
        [f for f in os.listdir(frame_folder) if f.lower().endswith(".jpg")],
        key=lambda x: int(os.path.splitext(x)[0]),
    )
    if not frame_files:
        sys.exit(f"[ERROR] No .jpg files in {frame_folder}")

    if preview_mode:
        frame_files = frame_files[:max_frames]
        print(f"[EYE_EAR] *** PREVIEW MODE — processing first {len(frame_files)} frames only ***")

    print(f"[EYE_EAR] Session     : {session_path}")
    print(f"[EYE_EAR] Frames      : {len(frame_files)}")
    print(f"[EYE_EAR] Output CSV  : {output_csv}")
    print(f"[EYE_EAR] CLAHE       : clip={clahe_clip}, grid={clahe_grid}x{clahe_grid}")
    print(f"[EYE_EAR] Haar        : scaleFactor={scale_factor}, minNeighbors={min_neighbors}")

    # Build CLAHE and Haar cascade
    clahe   = cv2.createCLAHE(clipLimit=clahe_clip,
                               tileGridSize=(clahe_grid, clahe_grid))
    cascade = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_eye.xml"
    )
    if cascade.empty():
        sys.exit("[ERROR] Could not load haarcascade_eye.xml")

    blink_count     = 0
    blink_frame_ctr = 0
    detected_count  = 0
    processed       = 0

    with open(output_csv, "w", newline="") as csvf:
        writer = csv.DictWriter(csvf, fieldnames=CSV_COLUMNS)
        writer.writeheader()

        for fname in frame_files:
            timestamp_ms = int(os.path.splitext(fname)[0])
            img_path     = os.path.join(frame_folder, fname)
            img          = cv2.imread(img_path)

            if img is None:
                print(f"[WARN] Cannot read {fname} — skipping.")
                continue

            if rotate_180:
                img = cv2.rotate(img, cv2.ROTATE_180)

            h, w = img.shape[:2]

            # 1. Enhance contrast
            enhanced = enhance_frame(img, clahe)
            gray     = cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY)

            # 2. Detect eye
            bbox = detect_eye(gray, cascade, scale_factor, min_neighbors)

            # 3. Compute EAR and blink state
            row = {
                "timestamp_ms"  : timestamp_ms,
                "ear"           : "",
                "left_ear"      : "",
                "right_ear"     : "",
                "is_blink_frame": 0,
                "blink_count"   : blink_count,
                "eye_detected"  : 0,
                "face_detected" : 0,   # backward compat alias
            }

            if bbox is not None:
                ex, ey, ew, eh = bbox
                ear = ellipse_ear(gray, bbox)

                if ear == EAR_FALLBACK:
                    # Ellipse fit failed — treat same as no detection
                    if verbose:
                        print(f"  [{timestamp_ms:>10}ms]  EYE bbox found but ellipse fit failed")
                else:
                    is_blink_frame = int(ear < ear_blink_thresh)

                    # Blink counting: N consecutive blink frames -> one blink
                    if ear < ear_blink_thresh:
                        blink_frame_ctr += 1
                    else:
                        if blink_frame_ctr >= blink_consec_min:
                            blink_count += 1
                        blink_frame_ctr = 0

                    row.update({
                        "ear"           : round(ear, 4),
                        "is_blink_frame": is_blink_frame,
                        "blink_count"   : blink_count,
                        "eye_detected"  : 1,
                        "face_detected" : 1,  # alias
                    })
                    detected_count += 1

                    if verbose:
                        status = "BLINK" if is_blink_frame else "OPEN "
                        print(f"  [{timestamp_ms:>10}ms]  EAR={ear:.3f}  {status}  "
                              f"eye=({ex},{ey},{ew}x{eh})  blinks={blink_count}")

                    # Optional visual preview
                    if show:
                        disp = enhanced.copy()
                        cv2.rectangle(disp, (ex, ey), (ex+ew, ey+eh), (0, 255, 100), 2)
                        cv2.putText(disp, f"EAR:{ear:.2f}", (ex, ey - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 100), 1)
                        cv2.putText(disp, f"Blinks:{blink_count}", (6, 22),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
                        cv2.putText(disp, f"t={timestamp_ms}ms", (6, 44),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
                        cv2.imshow("eye_ear -- press Q to quit", disp)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            print("[EYE_EAR] Interrupted by user.")
                            break

            else:
                if show:
                    disp = enhanced.copy()
                    cv2.putText(disp, "NO EYE", (6, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 255), 1)
                    cv2.imshow("eye_ear — press Q to quit", disp)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        print("[EYE_EAR] Interrupted by user.")
                        break
                if verbose:
                    print(f"  [{timestamp_ms:>10}ms]  NO EYE")

            writer.writerow(row)
            processed += 1

            # Progress every 200 frames
            if processed % 200 == 0:
                rate = detected_count / processed * 100
                print(f"[EYE_EAR] Progress: {processed}/{len(frame_files)} "
                      f"({rate:.1f}% eye detected so far) ...")

    if show:
        cv2.destroyAllWindows()

    detect_rate = detected_count / processed * 100 if processed else 0
    print(f"\n[EYE_EAR] Processed    : {processed} frames")
    print(f"[EYE_EAR] Eye detected : {detected_count}  ({detect_rate:.1f}%)")
    print(f"[EYE_EAR] No eye       : {processed - detected_count}")
    print(f"[EYE_EAR] Total blinks : {blink_count}")
    print(f"[EYE_EAR] Output       : {output_csv}")

    if preview_mode:
        # Save a contact sheet of detected frames for visual verification
        _save_preview_sheet(session_path, frame_folder, output_csv, clahe_clip,
                            clahe_grid, scale_factor, min_neighbors, rotate_180)
        print(f"\n[PREVIEW] Detection rate: {detect_rate:.1f}%")
        if detect_rate < 20:
            print("[PREVIEW] WARNING: Low detection rate -- camera may not be aimed at the eye.")
            print("          Adjust helmet camera mount and re-record a short test clip.")
        else:
            print("[PREVIEW] OK: Eye visible -- run full processing:")
            print(f"          python eye_ear.py --session {session_path}")
    else:
        print(f"\n[NEXT]  Run: python merge_session.py --session {session_path}")


# ── Preview contact sheet ─────────────────────────────────────────────────────

def _save_preview_sheet(
    session_path: str,
    frame_folder: str,
    csv_path: str,
    clahe_clip: float,
    clahe_grid: int,
    scale_factor: float,
    min_neighbors: int,
    rotate_180: bool,
) -> None:
    """
    Save a side-by-side contact sheet image:
      Left column  = raw frame
      Right column = CLAHE-enhanced frame + detected eye bbox
    Shows up to 6 sample frames (detected and non-detected) so you can
    visually confirm whether the eye is in frame.
    """
    import csv as _csv

    detected_ts = []
    missed_ts   = []
    with open(csv_path, newline="") as f:
        for row in _csv.DictReader(f):
            ts = row["timestamp_ms"]
            if row["eye_detected"] == "1":
                detected_ts.append(ts)
            else:
                missed_ts.append(ts)

    # Pick up to 3 detected + 3 missed frames
    samples = (
        [(ts, True)  for ts in detected_ts[:3]] +
        [(ts, False) for ts in missed_ts[:3]]
    )
    if not samples:
        return

    clahe   = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_grid, clahe_grid))
    cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_eye.xml")

    cell_w, cell_h = 400, 300
    cols = 2  # raw | enhanced
    rows = len(samples)
    sheet = np.zeros((rows * cell_h, cols * cell_w, 3), dtype=np.uint8)

    for row_i, (ts, was_detected) in enumerate(samples):
        img_path = os.path.join(frame_folder, ts + ".jpg")
        img = cv2.imread(img_path)
        if img is None:
            continue

        if rotate_180:
            img = cv2.rotate(img, cv2.ROTATE_180)

        # Left: raw
        raw_resized = cv2.resize(img, (cell_w, cell_h))
        sheet[row_i*cell_h:(row_i+1)*cell_h, 0:cell_w] = raw_resized

        # Right: enhanced + bbox
        enh  = enhance_frame(img, clahe)
        gray = cv2.cvtColor(enh, cv2.COLOR_BGR2GRAY)
        eyes = cascade.detectMultiScale(gray, scale_factor, min_neighbors,
                                        minSize=MIN_EYE_SIZE)
        disp = enh.copy()
        for ex, ey, ew, eh in eyes:
            ear_val = round(float(eh) / float(ew), 3)
            cv2.rectangle(disp, (ex, ey), (ex+ew, ey+eh), (0, 255, 100), 2)
            cv2.putText(disp, f"EAR={ear_val}", (ex, max(ey-6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 100), 1)

        label_color = (80, 220, 80) if was_detected else (80, 80, 220)
        label_text  = "EYE FOUND" if was_detected else "NO EYE"
        cv2.putText(disp, f"{ts}ms  {label_text}", (4, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_color, 1)

        enh_resized = cv2.resize(disp, (cell_w, cell_h))
        sheet[row_i*cell_h:(row_i+1)*cell_h, cell_w:cols*cell_w] = enh_resized

    # Column headers
    cv2.putText(sheet, "RAW", (cell_w//2 - 20, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.putText(sheet, "CLAHE + DETECTION", (cell_w + cell_w//2 - 80, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    out_path = os.path.join(session_path, "preview_sheet.jpg")
    cv2.imwrite(out_path, sheet)
    print(f"[PREVIEW] Contact sheet saved -> {out_path}")
    print("          Inspect it to confirm the eye is visible and the bbox is correct.")


# -- Entry point ---------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Eye-only EAR detection for the ESP32-S3-CAM helmet setup.\n"
            "Processes offline JPEG frames extracted from SD card recordings.\n"
            "No full face required — designed for close-up single-eye view."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    p.add_argument(
        "--session", required=True, metavar="SESSION_DIR",
        help="Path to session directory containing frames/ subfolder "
             "(e.g. sessions/session_001).",
    )
    p.add_argument(
        "--preview", nargs="?", const=100, type=int, metavar="N",
        help="Preview mode: process only the first N frames (default 100) and "
             "save a preview_sheet.jpg to verify camera positioning. "
             "Use this after a short test recording before committing to a full session.",
    )
    p.add_argument(
        "--clahe-clip", type=float, default=CLAHE_CLIP_DEFAULT, metavar="N",
        help=f"CLAHE clip limit — higher = more contrast boost "
             f"(default: {CLAHE_CLIP_DEFAULT}).",
    )
    p.add_argument(
        "--clahe-grid", type=int, default=CLAHE_GRID_DEFAULT, metavar="N",
        help=f"CLAHE tile grid size (default: {CLAHE_GRID_DEFAULT}).",
    )
    p.add_argument(
        "--scale-factor", type=float, default=SCALE_FACTOR_DEFAULT, metavar="F",
        help=f"Haar scaleFactor — lower = more sensitive but slower "
             f"(default: {SCALE_FACTOR_DEFAULT}).",
    )
    p.add_argument(
        "--min-neighbors", type=int, default=MIN_NEIGHBORS_DEFAULT, metavar="N",
        help=f"Haar minNeighbors — lower = more detections, more false positives "
             f"(default: {MIN_NEIGHBORS_DEFAULT}).",
    )
    p.add_argument(
        "--ear-blink-thresh", type=float, default=EAR_BLINK_THRESH_DEFAULT, metavar="F",
        help=f"EAR threshold for blink detection (default: {EAR_BLINK_THRESH_DEFAULT}).",
    )
    p.add_argument(
        "--blink-consec-min", type=int, default=BLINK_CONSEC_MIN_DEFAULT, metavar="N",
        help=f"Consecutive frames below threshold to count as a blink (default: {BLINK_CONSEC_MIN_DEFAULT}).",
    )
    p.add_argument(
        "--rotate-180", action="store_true",
        help="Rotate the image 180 degrees before processing (useful for upside-down cameras).",
    )
    p.add_argument(
        "--show", action="store_true",
        help="Display each enhanced frame with detection overlay in a window "
             "while processing (requires a display).",
    )
    p.add_argument(
        "--verbose", action="store_true",
        help="Print one line per frame.",
    )
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
        blink_consec_min=args.blink_consec_min,
        rotate_180=args.rotate_180,
        show=args.show,
        verbose=args.verbose,
        max_frames=args.preview,
    )


if __name__ == "__main__":
    main()

