#!/usr/bin/env python3
"""
EAR Validation Script — VALIDATION ONLY
========================================
This script validates that the Eye Aspect Ratio (EAR) calculation and
blink-detection algorithm work correctly on a live webcam feed BEFORE
applying the same logic to recorded video from the ESP32-S3-CAM in
Phase 2 dataset processing.

It is NOT a real-time fatigue detector — it is a calibration and
sanity-check tool. Run it to confirm MediaPipe FaceMesh is working
correctly on your hardware before committing to a recording session.

Usage:
    python ear_validation.py
    Press 'q' in the video window to quit.
"""

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


if __name__ == "__main__":
    main()
