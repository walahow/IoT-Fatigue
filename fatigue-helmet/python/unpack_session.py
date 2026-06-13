#!/usr/bin/env python3
"""
unpack_session.py — Extract JPEG frames from a session's video.mjpeg using video.idx.
======================================================================================
Uses exact byte_offset + frame_size from video.idx — never does blind SOI/EOI scanning.
Each frame is validated against its stored CRC32 before being saved.

One corrupt frame → [WARN] + skip; ALL subsequent frames are unaffected.
Power-loss partial write → detected by len(data) != frame_size; skipped cleanly.

Output: sessions/session_XXX/frames/{timestamp_ms}.jpg
        (same naming convention as the old per-file approach — downstream
         MediaPipe / merge_session.py scripts require NO changes)

Usage:
    python unpack_session.py --session sessions/session_001
    python unpack_session.py --session sessions/session_001 --skip-crc
    python unpack_session.py --session sessions/session_001 --verbose
"""

import argparse
import binascii
import os
import sys

# ── CRC32 parity note ────────────────────────────────────────────────────────
# Firmware uses polynomial 0xEDB88320 (standard Ethernet CRC-32).
# Python binascii.crc32() uses the same polynomial, but returns a signed int
# in some Python versions. The & 0xFFFFFFFF mask guarantees an unsigned result.
# ─────────────────────────────────────────────────────────────────────────────

IDX_HEADER = "frame_index,timestamp_ms,byte_offset,frame_size,crc32_hex"
IDX_NCOLS  = 5


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Extract JPEG frames from video.mjpeg using video.idx"
    )
    p.add_argument("--session",  required=True,
                   help="Path to session directory (e.g. sessions/session_001)")
    p.add_argument("--skip-crc", action="store_true",
                   help="Skip CRC32 validation (faster, but less safe)")
    p.add_argument("--verbose",  action="store_true",
                   help="Print one line per successfully extracted frame")
    return p.parse_args()


def unpack(session_path: str, skip_crc: bool, verbose: bool) -> None:
    mjpeg_path = os.path.join(session_path, "video.mjpeg")
    idx_path   = os.path.join(session_path, "video.idx")
    frames_dir = os.path.join(session_path, "frames")

    # ── Pre-flight checks ────────────────────────────────────────────────────
    for path, label in [(mjpeg_path, "video.mjpeg"), (idx_path, "video.idx")]:
        if not os.path.exists(path):
            sys.exit(f"[ERROR] {label} not found in {session_path}")

    os.makedirs(frames_dir, exist_ok=True)

    # ── Stats counters ───────────────────────────────────────────────────────
    ok        = 0
    skipped   = 0
    crc_fails = 0

    with open(mjpeg_path, "rb") as vf, open(idx_path, "r") as idxf:
        header = idxf.readline().strip()
        if header != IDX_HEADER:
            print(f"[WARN] Unexpected idx header: {header!r}")
            print(f"       Expected            : {IDX_HEADER!r}")
            print("       Proceeding anyway — column order may be wrong.")

        for lineno, raw_line in enumerate(idxf, start=2):  # 2 = after header
            line = raw_line.strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != IDX_NCOLS:
                print(f"[WARN] Line {lineno}: expected {IDX_NCOLS} columns, "
                      f"got {len(parts)} — skipping: {line!r}")
                skipped += 1
                continue

            frame_idx_str, timestamp_str, offset_str, size_str, crc_str = parts

            try:
                frame_idx   = int(frame_idx_str)
                timestamp   = int(timestamp_str)
                byte_offset = int(offset_str)
                frame_size  = int(size_str)
            except ValueError as exc:
                print(f"[WARN] Line {lineno}: cannot parse numeric fields "
                      f"({exc}) — skipping.")
                skipped += 1
                continue

            # ── Seek to exact position and read exact number of bytes ────────
            vf.seek(byte_offset)
            data = vf.read(frame_size)

            if len(data) != frame_size:
                print(f"[WARN] Frame {frame_idx} @ {timestamp}ms: "
                      f"expected {frame_size} B, got {len(data)} B — "
                      f"truncated (power-loss?). Skipping.")
                skipped += 1
                continue

            # ── CRC32 validation ─────────────────────────────────────────────
            if not skip_crc:
                computed = binascii.crc32(data) & 0xFFFFFFFF  # unsigned
                try:
                    stored = int(crc_str, 16)
                except ValueError:
                    print(f"[WARN] Frame {frame_idx}: cannot parse CRC "
                          f"{crc_str!r} — skipping.")
                    crc_fails += 1
                    continue

                if computed != stored:
                    print(f"[WARN] Frame {frame_idx} @ {timestamp}ms: "
                          f"CRC mismatch "
                          f"(stored={stored:08x}, computed={computed:08x}) "
                          f"— corrupt frame, skipping.")
                    crc_fails += 1
                    continue

            # ── Save frame ───────────────────────────────────────────────────
            out_path = os.path.join(frames_dir, f"{timestamp}.jpg")
            with open(out_path, "wb") as out_f:
                out_f.write(data)

            ok += 1
            if verbose:
                print(f"[OK]  {timestamp}.jpg  "
                      f"({frame_size:,} B)  frame={frame_idx}")

    # ── Summary ──────────────────────────────────────────────────────────────
    print(f"\n[DONE] Session : {session_path}")
    print(f"       Frames  : {frames_dir}")
    print(f"       Extracted : {ok}")
    print(f"       Skipped   : {skipped}  (truncated / malformed idx lines)")
    print(f"       CRC fails : {crc_fails}  (corrupt frame data, skipped)")
    if crc_fails > 0:
        print("\n[NOTE] CRC failures usually mean the SD card lost power during a")
        print("       write. The frame was not saved cleanly. All other frames OK.")


def main() -> None:
    args = parse_args()
    unpack(args.session, args.skip_crc, args.verbose)


if __name__ == "__main__":
    main()
