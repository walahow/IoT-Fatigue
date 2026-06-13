#!/usr/bin/env python3
"""
mjpeg_to_mp4.py -- Convert video.mjpeg directly to MP4 using video.idx.
=========================================================================
No intermediate frame files required.
Decodes each JPEG in memory via the exact byte_offset + frame_size from
the index, upscales with Lanczos4, and writes straight to VideoWriter.

Handles timestamp gaps by duplicating the last frame (same as frames_to_mp4.py).
Corrupt / CRC-failed frames are skipped gracefully.

Usage:
    python mjpeg_to_mp4.py --session D:/path/to/session_001
    python mjpeg_to_mp4.py --session D:/path/to/session_001 --scale 3
    python mjpeg_to_mp4.py --session D:/path/to/session_001 --no-timestamp
    python mjpeg_to_mp4.py --session D:/path/to/session_001 --skip-crc
    python mjpeg_to_mp4.py --session D:/path/to/session_001 --out my_clip.mp4

Output:
    <session_dir>/video.mp4  (default, overwrites if exists)
"""

import argparse
import binascii
import os
import sys

import cv2
import numpy as np

# ── Index format ─────────────────────────────────────────────────────────────
IDX_HEADER = "frame_index,timestamp_ms,byte_offset,frame_size,crc32_hex"
IDX_NCOLS  = 5

# ── OSD constants ─────────────────────────────────────────────────────────────
OSD_FONT      = cv2.FONT_HERSHEY_SIMPLEX
OSD_SCALE     = 0.45
OSD_THICKNESS = 1
OSD_COLOR     = (255, 255, 255)
OSD_SHADOW    = (0, 0, 0)
OSD_MARGIN    = 6
# ─────────────────────────────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Convert video.mjpeg directly to MP4 (no frame extraction needed)"
    )
    p.add_argument("--session", required=True,
                   help="Path to session directory (must contain video.mjpeg + video.idx)")
    p.add_argument("--fps", type=float, default=None,
                   help="Output FPS (default: read from metadata.txt or 30)")
    p.add_argument("--scale", type=float, default=2.0,
                   help="Upscale factor (default: 2 = 640x480, use 1 for native 320x240)")
    p.add_argument("--out", default=None,
                   help="Output MP4 path (default: <session>/video.mp4)")
    p.add_argument("--no-timestamp", action="store_true",
                   help="Disable timestamp OSD overlay")
    p.add_argument("--no-fill-gaps", action="store_true",
                   help="Don't duplicate frames to fill timestamp gaps")
    p.add_argument("--skip-crc", action="store_true",
                   help="Skip CRC32 validation (faster)")
    p.add_argument("--codec", default="avc1",
                   help="FourCC codec (default: avc1/H.264, fallback: mp4v)")
    return p.parse_args()


def read_metadata(session_path: str) -> dict:
    meta = {}
    path = os.path.join(session_path, "metadata.txt")
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                if "=" in line:
                    k, v = line.strip().split("=", 1)
                    meta[k.strip()] = v.strip()
    return meta


def draw_osd(frame, timestamp_ms: int, frame_num: int, total: int):
    secs  = timestamp_ms / 1000.0
    mins  = int(secs) // 60
    secs -= mins * 60
    text  = f"{mins:02d}:{secs:05.2f}  [{frame_num}/{total}]"
    y     = frame.shape[0] - OSD_MARGIN
    cv2.putText(frame, text, (OSD_MARGIN + 1, y + 1),
                OSD_FONT, OSD_SCALE, OSD_SHADOW, OSD_THICKNESS + 1, cv2.LINE_AA)
    cv2.putText(frame, text, (OSD_MARGIN, y),
                OSD_FONT, OSD_SCALE, OSD_COLOR, OSD_THICKNESS, cv2.LINE_AA)


def open_writer(out_path: str, codec: str, fps: float, size: tuple):
    """Try requested codec, fall back to mp4v if unavailable."""
    fourcc = cv2.VideoWriter_fourcc(*codec)
    writer = cv2.VideoWriter(out_path, fourcc, fps, size)
    if not writer.isOpened() and codec != "mp4v":
        print(f"[WARN] Codec '{codec}' unavailable, falling back to mp4v.")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(out_path, fourcc, fps, size)
    if not writer.isOpened():
        sys.exit("[ERROR] VideoWriter failed to open. Check codec / path.")
    return writer


def convert(session_path: str, fps: float, scale: float, out_path: str,
            show_ts: bool, fill_gaps: bool, skip_crc: bool, codec: str) -> None:

    mjpeg_path = os.path.join(session_path, "video.mjpeg")
    idx_path   = os.path.join(session_path, "video.idx")

    for p, label in [(mjpeg_path, "video.mjpeg"), (idx_path, "video.idx")]:
        if not os.path.exists(p):
            sys.exit(f"[ERROR] {label} not found in {session_path}")

    # ── Count index lines for progress display ────────────────────────────────
    with open(idx_path) as f:
        total_idx = sum(1 for ln in f if ln.strip()) - 1  # minus header
    print(f"[INFO] Index entries : {total_idx}")

    # ── Determine output dimensions from first valid JPEG ─────────────────────
    src_w = src_h = None
    with open(mjpeg_path, "rb") as vf, open(idx_path) as idxf:
        idxf.readline()  # skip header
        for line in idxf:
            parts = line.strip().split(",")
            if len(parts) != IDX_NCOLS:
                continue
            try:
                offset = int(parts[2])
                size   = int(parts[3])
            except ValueError:
                continue
            vf.seek(offset)
            data = vf.read(size)
            if len(data) != size:
                continue
            arr = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                src_h, src_w = img.shape[:2]
                break

    if src_w is None:
        sys.exit("[ERROR] Could not decode any frame to determine resolution.")

    out_w = int(src_w * scale)
    out_h = int(src_h * scale)
    out_w += out_w % 2   # must be even
    out_h += out_h % 2
    interp = cv2.INTER_LANCZOS4 if scale > 1.0 else cv2.INTER_AREA

    print(f"[INFO] Source size   : {src_w}x{src_h}")
    print(f"[INFO] Output size   : {out_w}x{out_h}  (scale={scale}x, Lanczos4)")
    print(f"[INFO] FPS           : {fps:.1f}")
    print(f"[INFO] Output        : {out_path}")
    print(f"[INFO] Gap fill      : {'yes' if fill_gaps else 'no'}")
    print(f"[INFO] CRC check     : {'no (--skip-crc)' if skip_crc else 'yes'}")

    writer = open_writer(out_path, codec, fps, (out_w, out_h))

    ms_per_frame = 1000.0 / fps

    # ── Main encoding loop ────────────────────────────────────────────────────
    written    = 0
    gap_filled = 0
    skipped    = 0
    crc_fails  = 0
    prev_frame = None
    prev_ts    = None

    with open(mjpeg_path, "rb") as vf, open(idx_path) as idxf:
        idxf.readline()  # skip header

        for lineno, raw in enumerate(idxf, start=2):
            line = raw.strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != IDX_NCOLS:
                skipped += 1
                continue

            try:
                frame_idx   = int(parts[0])
                timestamp   = int(parts[1])
                byte_offset = int(parts[2])
                frame_size  = int(parts[3])
                crc_str     = parts[4]
            except ValueError:
                skipped += 1
                continue

            # ── Read raw JPEG bytes ───────────────────────────────────────────
            vf.seek(byte_offset)
            data = vf.read(frame_size)

            if len(data) != frame_size:
                print(f"[WARN] Frame {frame_idx}: truncated ({len(data)}/{frame_size} B) - skip")
                skipped += 1
                continue

            # ── CRC check ────────────────────────────────────────────────────
            if not skip_crc:
                computed = binascii.crc32(data) & 0xFFFFFFFF
                try:
                    stored = int(crc_str, 16)
                except ValueError:
                    crc_fails += 1
                    continue
                if computed != stored:
                    print(f"[WARN] Frame {frame_idx}: CRC mismatch - skip")
                    crc_fails += 1
                    continue

            # ── Decode JPEG in memory ─────────────────────────────────────────
            arr = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                print(f"[WARN] Frame {frame_idx}: JPEG decode failed - skip")
                skipped += 1
                continue

            # ── Upscale ───────────────────────────────────────────────────────
            if scale != 1.0:
                img = cv2.resize(img, (out_w, out_h), interpolation=interp)

            # ── Fill timing gap ───────────────────────────────────────────────
            if fill_gaps and prev_frame is not None and prev_ts is not None:
                gap_ms    = timestamp - prev_ts
                n_extra   = max(0, round(gap_ms / ms_per_frame) - 1)
                if n_extra > 0:
                    fill = prev_frame.copy()
                    if show_ts:
                        draw_osd(fill, prev_ts, written + 1, total_idx)
                    for _ in range(n_extra):
                        writer.write(fill)
                        written    += 1
                        gap_filled += 1

            # ── OSD + write ───────────────────────────────────────────────────
            if show_ts:
                draw_osd(img, timestamp, written + 1, total_idx)

            writer.write(img)
            written    += 1
            prev_frame  = img.copy()
            prev_ts     = timestamp

            if written % 500 == 0 or frame_idx == total_idx - 1:
                print(f"  ... frame {frame_idx+1}/{total_idx}  "
                      f"({written} written, {gap_filled} gap-filled, "
                      f"{skipped} skipped, {crc_fails} CRC fails)")

    writer.release()
    print(f"\n[DONE] {written} frames -> {out_path}")
    if gap_filled:
        print(f"       {gap_filled} duplicate frames inserted for timing gaps")
    if skipped or crc_fails:
        print(f"       {skipped} skipped (truncated/malformed)  |  {crc_fails} CRC failures")
    file_mb = os.path.getsize(out_path) / (1024 * 1024)
    print(f"       File size : {file_mb:.1f} MB")


def main() -> None:
    args = parse_args()

    session_path = os.path.abspath(args.session)
    if not os.path.isdir(session_path):
        sys.exit(f"[ERROR] Session directory not found: {session_path}")

    fps = args.fps
    if fps is None:
        meta = read_metadata(session_path)
        try:
            fps = float(meta.get("camera_fps", "30"))
        except ValueError:
            fps = 30.0

    out_path = args.out or os.path.join(session_path, "video.mp4")

    print(f"[INFO] Session       : {session_path}")
    convert(
        session_path = session_path,
        fps          = fps,
        scale        = args.scale,
        out_path     = out_path,
        show_ts      = not args.no_timestamp,
        fill_gaps    = not args.no_fill_gaps,
        skip_crc     = args.skip_crc,
        codec        = args.codec,
    )


if __name__ == "__main__":
    main()
