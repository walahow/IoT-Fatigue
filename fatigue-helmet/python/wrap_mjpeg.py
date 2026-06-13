#!/usr/bin/env python3
"""
wrap_mjpeg.py — Wrap raw OV2640 video.mjpeg into a standard AVI for VLC playback.
====================================================================================
OV2640 produces raw concatenated JPEG bytes — not a standard MJPEG container.
VLC may refuse to play or show a black screen when opening video.mjpeg directly.
This script uses ffmpeg to wrap the raw stream into a proper AVI container.

Requires ffmpeg on PATH:
    Windows:  winget install ffmpeg   OR  https://ffmpeg.org/download.html
    Linux:    sudo apt install ffmpeg
    macOS:    brew install ffmpeg

Usage:
    python wrap_mjpeg.py --session sessions/session_001
    python wrap_mjpeg.py --session sessions/session_001 --fps 30
    python wrap_mjpeg.py --session sessions/session_001 --fps 30 --no-open

When to use:
    1. Run unpack_session.py first to extract frames.
    2. Try opening video.mjpeg directly in VLC.
       - If it plays correctly  → this script is optional (add note to README).
       - If VLC shows black screen or error → run this script.
"""

import argparse
import os
import subprocess
import sys


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Wrap raw OV2640 MJPEG stream into AVI for VLC playback"
    )
    p.add_argument("--session", required=True,
                   help="Path to session directory (e.g. sessions/session_001)")
    p.add_argument("--fps", type=int, default=30,
                   help="Recording frame rate used during capture (default: 30)")
    p.add_argument("--no-open", action="store_true",
                   help="Do not print the VLC open command after wrapping")
    return p.parse_args()


def check_ffmpeg() -> bool:
    """Return True if ffmpeg is available on PATH."""
    try:
        result = subprocess.run(
            ["ffmpeg", "-version"],
            capture_output=True, text=True, timeout=5
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def wrap(session_path: str, fps: int, no_open: bool) -> None:
    src = os.path.join(session_path, "video.mjpeg")
    dst = os.path.join(session_path, "video.avi")

    if not os.path.exists(src):
        sys.exit(f"[ERROR] video.mjpeg not found in {session_path}\n"
                 f"        Run unpack_session.py first, or check session path.")

    if not check_ffmpeg():
        sys.exit(
            "[ERROR] ffmpeg not found on PATH.\n"
            "        Install it and try again:\n"
            "          Windows: winget install ffmpeg\n"
            "          Linux:   sudo apt install ffmpeg\n"
            "          macOS:   brew install ffmpeg"
        )

    mjpeg_size_mb = os.path.getsize(src) / (1024 * 1024)
    print(f"[WRAP] Source : {src}  ({mjpeg_size_mb:.1f} MB)")
    print(f"[WRAP] Target : {dst}")
    print(f"[WRAP] FPS    : {fps}")

    cmd = [
        "ffmpeg", "-y",            # overwrite output without asking
        "-f",  "mjpeg",            # treat input as raw MJPEG stream
        "-r",  str(fps),           # input frame rate
        "-i",  src,                # source file
        "-c:v", "copy",            # copy codec — no re-encoding (fast, lossless)
        dst
    ]

    print(f"[WRAP] Running: {' '.join(cmd)}\n")

    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode == 0:
        avi_size_mb = os.path.getsize(dst) / (1024 * 1024)
        print(f"\n[WRAP] Done — {dst}  ({avi_size_mb:.1f} MB)")
        if not no_open:
            print(f"\n[WRAP] Open in VLC:")
            print(f"         vlc \"{dst}\"")
            print(f"\n[NOTE] If video.mjpeg also plays directly in VLC,")
            print(f"       wrap_mjpeg.py is optional. Document in README.md.")
    else:
        print(f"[ERROR] ffmpeg failed (exit code {result.returncode}):")
        print(result.stderr[-2000:])   # last 2000 chars of stderr (most useful)
        sys.exit(1)


def main() -> None:
    args = parse_args()
    wrap(args.session, args.fps, args.no_open)


if __name__ == "__main__":
    main()
