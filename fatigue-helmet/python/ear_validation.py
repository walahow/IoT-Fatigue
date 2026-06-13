#!/usr/bin/env python3
"""
EAR Validation Script
======================
Two modes of operation:

1. LIVE (default, no arguments)
   Validates EAR calculation on a live webcam feed BEFORE applying the same
   logic to recorded video from the ESP32-S3-CAM.
   This is a calibration / sanity-check tool, NOT a real-time fatigue detector.
   Press 'q' in the video window to quit.

2. OFFLINE (--input <frames_dir>)
   Processes extracted JPEG frames from unpack_session.py output.
   Reads frames sorted by filename (= timestamp_ms), runs FaceMesh + EAR/blink
   on each, and writes camera_features.csv to the parent session directory.
   This is the post-processing step for Phase 2B dataset collection.

Usage:
    python ear_validation.py                              # live webcam
    python ear_validation.py --input sessions/session_001/frames/
    python ear_validation.py --input sessions/session_001/frames/ --verbose
"""

import argparse
import csv
import os
import time
from collections import deque

import cv2
import mediapipe as mp
import numpy as np

# ── EAR configuration ─────────────────────────────────────────────────────
# MediaPipe FaceMesh landmark indices for each eye
# Order: [outer, top-far, top-near, inner, bottom-near, bottom-far]
LEFT_EYE_IDX  = [362, 385, 387, 263, 373, 380]
RIGHT_EYE_IDX = [33,  160, 158, 133, 153, 144]

EAR_BLINK_THRESH  = 0.20   # EAR below this → blink frame
EAR_OPEN_THRESH   = 0.25   # EAR above this → eyes clearly open
BLINK_CONSEC_MIN  = 2      # consecutive blink-frames required to count one blink
RATE_WINDOW_SEC   = 60     # rolling window for blinks/min calculation
STATS_INTERVAL    = 10     # seconds between terminal log lines


# ── Geometry helpers ──────────────────────────────────────────────────────

def dist(p1, p2) -> float:
    return float(np.linalg.norm(np.subtract(p1, p2)))


def compute_ear(landmarks, indices, w: int, h: int) -> float:
    """
    EAR = (||p2-p6|| + ||p3-p5||) / (2 * ||p1-p4||)
    indices: [p1, p2, p3, p4, p5, p6] (outer→inner across, top/bottom pairs)
    """
    pts = [(landmarks[i].x * w, landmarks[i].y * h) for i in indices]
    p1, p2, p3, p4, p5, p6 = pts
    vertical = dist(p2, p6) + dist(p3, p5)
    horizontal = dist(p1, p4)
    if horizontal < 1e-6:
        return 0.0
    return vertical / (2.0 * horizontal)


def draw_eye_contour(frame, landmarks, indices, w: int, h: int, color):
    pts = np.array(
        [(int(landmarks[i].x * w), int(landmarks[i].y * h)) for i in indices],
        dtype=np.int32,
    )
    cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=1)


# ── Overlay helpers ───────────────────────────────────────────────────────

def put(frame, text: str, pos, scale=0.7, color=(255, 255, 255), thickness=2):
    cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness)


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Cannot open webcam (index 0). Check connection or try index 1.")
        return

    mp_mesh   = mp.solutions.face_mesh
    face_mesh = mp_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    blink_count       = 0
    blink_frame_ctr   = 0
    blink_timestamps  = deque()   # wall-clock time of each counted blink

    fps_frame_ctr = 0
    fps_clock     = time.time()
    current_fps   = 0.0

    last_stats_t = time.time()
    start_time   = time.time()

    # Working values used both inside and outside the face-detection block
    ear        = 0.0
    blink_rate = 0.0

    print("EAR Validation running. Press 'q' to quit.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to read frame from webcam.")
            break

        h, w = frame.shape[:2]
        rgb     = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_mesh.process(rgb)

        now              = time.time()
        status_text      = "NO FACE"
        status_color_bgr = (100, 100, 100)

        if results.multi_face_landmarks:
            lm = results.multi_face_landmarks[0].landmark

            # Draw sparse landmark dots on eye points only (faster than full mesh)
            for idx in LEFT_EYE_IDX + RIGHT_EYE_IDX:
                cx, cy = int(lm[idx].x * w), int(lm[idx].y * h)
                cv2.circle(frame, (cx, cy), 2, (0, 255, 255), -1)

            draw_eye_contour(frame, lm, LEFT_EYE_IDX,  w, h, (0, 200, 255))
            draw_eye_contour(frame, lm, RIGHT_EYE_IDX, w, h, (0, 200, 255))

            left_ear  = compute_ear(lm, LEFT_EYE_IDX,  w, h)
            right_ear = compute_ear(lm, RIGHT_EYE_IDX, w, h)
            ear       = (left_ear + right_ear) / 2.0

            # Blink detection: N consecutive frames below threshold → one blink
            if ear < EAR_BLINK_THRESH:
                blink_frame_ctr += 1
            else:
                if blink_frame_ctr >= BLINK_CONSEC_MIN:
                    blink_count += 1
                    blink_timestamps.append(now)
                blink_frame_ctr = 0

            # Purge blinks outside rolling rate window
            cutoff = now - RATE_WINDOW_SEC
            while blink_timestamps and blink_timestamps[0] < cutoff:
                blink_timestamps.popleft()

            # Scale blink count to per-minute using actual elapsed or full window
            elapsed_window = min(now - start_time, RATE_WINDOW_SEC)
            blink_rate = (len(blink_timestamps) / elapsed_window * 60.0
                          if elapsed_window > 0 else 0.0)

            # Status label with colour-coded bar
            if ear > EAR_OPEN_THRESH:
                status_text      = "EYES OPEN"
                status_color_bgr = (30, 180, 30)
            elif ear < EAR_BLINK_THRESH:
                status_text      = "BLINK"
                status_color_bgr = (30, 30, 210)
            else:
                status_text      = "CLOSING"
                status_color_bgr = (30, 140, 220)

            # Metric overlay (top-left)
            put(frame, f"EAR: {ear:.3f}",          (10, 30))
            put(frame, f"Blinks: {blink_count}",    (10, 60))
            put(frame, f"Rate: {blink_rate:.1f}/min", (10, 90))

        # Status bar at bottom
        cv2.rectangle(frame, (0, h - 42), (w, h), status_color_bgr, -1)
        put(frame, status_text, (10, h - 12), scale=0.9)

        # FPS counter (top-right)
        fps_frame_ctr += 1
        if now - fps_clock >= 1.0:
            current_fps   = fps_frame_ctr / (now - fps_clock)
            fps_frame_ctr = 0
            fps_clock     = now
        put(frame, f"FPS: {current_fps:.1f}", (w - 130, 30), color=(200, 200, 200))

        cv2.imshow("EAR Validation — press Q to quit", frame)

        # Terminal stats every STATS_INTERVAL seconds
        if now - last_stats_t >= STATS_INTERVAL:
            session_s = now - start_time
            print(f"[{session_s:6.0f}s]  EAR: {ear:.3f}  |  "
                  f"Blinks: {blink_count}  |  "
                  f"Rate: {blink_rate:.1f}/min  |  "
                  f"FPS: {current_fps:.1f}")
            last_stats_t = now

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    face_mesh.close()
    print(f"\nSession ended. Total blinks counted: {blink_count}")


# ── Offline mode ──────────────────────────────────────────────────────────────

def process_folder(frame_folder: str, verbose: bool) -> None:
    """
    Offline processing of extracted JPEG frames.
    Reads frames sorted by filename (timestamp_ms), runs the same
    FaceMesh + EAR + blink detection logic as the live mode.
    Writes camera_features.csv to the parent session directory.
    """
    session_dir = os.path.dirname(os.path.abspath(frame_folder))
    output_csv  = os.path.join(session_dir, "camera_features.csv")

    # Collect and sort JPEG files by timestamp (integer filename)
    frame_files = sorted(
        [f for f in os.listdir(frame_folder) if f.lower().endswith(".jpg")],
        key=lambda x: int(os.path.splitext(x)[0])  # sort by timestamp_ms
    )

    if not frame_files:
        print(f"[ERROR] No .jpg files found in {frame_folder}")
        return

    print(f"[OFFLINE] Frame folder : {frame_folder}")
    print(f"[OFFLINE] Frames found : {len(frame_files)}")
    print(f"[OFFLINE] Output CSV   : {output_csv}")

    mp_mesh   = mp.solutions.face_mesh
    face_mesh = mp_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    blink_count     = 0
    blink_frame_ctr = 0
    no_face_count   = 0
    processed       = 0

    CSV_COLUMNS = [
        "timestamp_ms", "ear", "left_ear", "right_ear",
        "is_blink_frame", "blink_count", "face_detected"
    ]

    with open(output_csv, "w", newline="") as csvf:
        writer = csv.DictWriter(csvf, fieldnames=CSV_COLUMNS)
        writer.writeheader()

        for fname in frame_files:
            timestamp_ms = int(os.path.splitext(fname)[0])
            img_path     = os.path.join(frame_folder, fname)
            img          = cv2.imread(img_path)

            if img is None:
                print(f"[WARN] Cannot read {fname} — skipping.")
                no_face_count += 1
                continue

            h, w = img.shape[:2]
            rgb  = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = face_mesh.process(rgb)

            row = {
                "timestamp_ms"   : timestamp_ms,
                "ear"            : "",
                "left_ear"       : "",
                "right_ear"      : "",
                "is_blink_frame" : 0,
                "blink_count"    : blink_count,
                "face_detected"  : 0,
            }

            if results.multi_face_landmarks:
                lm        = results.multi_face_landmarks[0].landmark
                left_ear  = compute_ear(lm, LEFT_EYE_IDX,  w, h)
                right_ear = compute_ear(lm, RIGHT_EYE_IDX, w, h)
                ear       = (left_ear + right_ear) / 2.0

                is_blink_frame = int(ear < EAR_BLINK_THRESH)

                # Blink counting: require BLINK_CONSEC_MIN consecutive blink frames
                if ear < EAR_BLINK_THRESH:
                    blink_frame_ctr += 1
                else:
                    if blink_frame_ctr >= BLINK_CONSEC_MIN:
                        blink_count += 1
                    blink_frame_ctr = 0

                row.update({
                    "ear"           : round(ear, 4),
                    "left_ear"      : round(left_ear, 4),
                    "right_ear"     : round(right_ear, 4),
                    "is_blink_frame": is_blink_frame,
                    "blink_count"   : blink_count,
                    "face_detected" : 1,
                })

                if verbose:
                    status = "BLINK" if is_blink_frame else "OPEN"
                    print(f"[{timestamp_ms:>10}ms]  EAR={ear:.3f}  {status}  "
                          f"blinks={blink_count}")
            else:
                no_face_count += 1
                if verbose:
                    print(f"[{timestamp_ms:>10}ms]  NO FACE")

            writer.writerow(row)
            processed += 1

    face_mesh.close()

    face_rate = (processed - no_face_count) / processed * 100 if processed else 0
    print(f"\n[OFFLINE] Processed   : {processed} frames")
    print(f"[OFFLINE] Face detected: {processed - no_face_count}  "
          f"({face_rate:.1f}%)")
    print(f"[OFFLINE] No face      : {no_face_count}")
    print(f"[OFFLINE] Total blinks : {blink_count}")
    print(f"[OFFLINE] Output       : {output_csv}")
    print(f"\n[NEXT]  Run merge_session.py to combine with sensor_data.csv")


# ── Entry point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    parser = argparse.ArgumentParser(
        description="EAR validation — live webcam or offline JPEG folder"
    )
    parser.add_argument(
        "--input", default=None, metavar="FRAMES_DIR",
        help="Path to extracted frames folder for offline processing. "
             "If omitted, live webcam mode is used."
    )
    parser.add_argument(
        "--verbose", action="store_true",
        help="Print one line per frame in offline mode."
    )
    args = parser.parse_args()

    if args.input:
        if not os.path.isdir(args.input):
            sys.exit(f"[ERROR] --input directory not found: {args.input}")
        process_folder(args.input, args.verbose)
    else:
        main()
