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
    p.add_argument("--no-fill-gaps", action="store_true",
                   help="Skip duplicate-frame gap filling; just encode frames as-is")
    p.add_argument("--codec", default="avc1",
                   help="FourCC codec string (default: avc1 H.264). Fallback: mp4v.")
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
                (OSD_MARGIN + 1, frame.shape[0] - OSD_MARGIN - 1),
                OSD_FONT, OSD_SCALE, OSD_SHADOW, OSD_THICKNESS + 1,
                cv2.LINE_AA)
    # Main text
    cv2.putText(frame, text,
                (OSD_MARGIN, frame.shape[0] - OSD_MARGIN),
                OSD_FONT, OSD_SCALE, OSD_COLOR, OSD_THICKNESS,
                cv2.LINE_AA)


def encode(session_path: str, fps: float, out_path: str, scale: float,
           show_timestamp: bool, fill_gaps: bool, codec: str) -> None:  # noqa

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

    # ── Set up VideoWriter ────────────────────────────────────────────────────
    fourcc = cv2.VideoWriter_fourcc(*codec)
    writer = cv2.VideoWriter(out_path, fourcc, fps, (out_w, out_h))
    if not writer.isOpened():
        # avc1 may not be available on all Windows builds; fall back to mp4v
        print(f"[WARN] Codec '{codec}' unavailable, falling back to mp4v.")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(out_path, fourcc, fps, (out_w, out_h))
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

        # Upscale
        if scale != 1.0:
            img = cv2.resize(img, (out_w, out_h), interpolation=interp)

        # Fill timing gap with previous frame duplicated
        if fill_gaps and prev_frame is not None:
            gap_ms       = ts - prev_ts
            extra_frames = max(0, round(gap_ms / ms_per_frame) - 1)
            if extra_frames > 0:
                fill_img = prev_frame.copy()
                if show_timestamp:
                    draw_osd(fill_img, prev_ts, written + 1, total)
                for _ in range(extra_frames):
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
    )


if __name__ == "__main__":
    main()
