#!/usr/bin/env python3
"""
frame_inject_replay.py
========================
Feeds a folder of already-recorded JPEG frames (named {timestamp_ms}.jpg,
the unpack_session.py/debug_recorder.py convention) to the ESP32 one at a
time over USB serial, where the REAL compiled firmware (flashed with
`pio run -e esp32s3cam_frame_inject -t upload`) runs its actual on-device
processEarFrame() on each one -- the same code path a live capture would
take -- and reports back ROI-lock / blink events.

This is the hardware-in-the-loop counterpart to tools/session_replay
(which re-runs the same algorithm on the PC instead of the chip). Use
both to compare: if they agree, the PC-side replay is a trustworthy stand
-in; if they disagree, something differs between the host build and the
actual ESP32 binary (JPEG decoder, float behavior, etc.).

Protocol per frame (matches frameInjectTask() in main.cpp):
    SOF(4)=AA BB CC DD | ts_ms(u32 LE) | width(u32 LE) | height(u32 LE)
    | jpeg_len(u32 LE) | jpeg_bytes | EOF(4)=DD CC BB AA
The ESP replies "#FRAME_DONE ts=<ts>" once it has finished processing,
which this script waits for before sending the next frame (simple
lockstep flow control -- no need to match real-time pacing since the
firmware's internal timing logic keys off the ts_ms value we send, not
wall-clock time).

Usage:
    python frame_inject_replay.py --frames-dir ../../sessions/session_067/frames --out session_067_esp_result.csv

Output CSV columns match tools/session_replay's output exactly, so the
two can be diffed directly:
    timestamp_ms,processed,ear,is_blink_event,rolling_rate_bpm,roi_x,roi_y,roi_locked,lock_conf
"ear" here is left blank (the firmware only reports it implicitly via lock
status + blink events, not every frame) -- see README note in the script.

--hog-out also records the HOG classifier's blinks ("#STATUS: blink (hog)")
as timestamp_ms,score. Compare against the rows with blink=1 in
`session_replay <frames> <csv> --hog-model=<session>/blink_model.bin
--hog-csv=hog.csv`: the same timestamps mean the chip's JPEG decoder and
float maths reproduce the PC replay the classifier was validated on.
"""

import argparse
import glob
import os
import re
import struct
import sys
import time

import cv2          # only for the JPEG's dimensions; already a project dependency
import numpy as np
import serial

SOF = bytes([0xAA, 0xBB, 0xCC, 0xDD])
EOF_MARKER = bytes([0xDD, 0xCC, 0xBB, 0xAA])
CHUNK = 256          # bytes per serial write; see the send loop's note

LOCK_RE = re.compile(
    r"EAR ROI motion-locked at x=(\d+) y=(\d+) size=(\d+) conf=([\d.]+)"
)
REJECT_RE = re.compile(r"EAR motion lock rejected conf=([\d.]+)")
BLINK_RE = re.compile(r"#STATUS: blink \(glint\) t=(\d+)")
HOG_BLINK_RE = re.compile(r"#STATUS: blink \(hog\) t=(\d+) score=(-?[\d.]+)")
FRAME_DONE_RE = re.compile(r"#FRAME_DONE ts=(\d+)")
TIMING_RE = re.compile(r"#FRAME_DONE ts=\d+ ear=(\d+)us hog=(\d+)us")
ERROR_RE = re.compile(r"#ERROR:")


def list_frames(frames_dir):
    paths = glob.glob(os.path.join(frames_dir, "*.jpg"))
    frames = []
    for p in paths:
        name = os.path.splitext(os.path.basename(p))[0]
        if name.isdigit():
            frames.append((int(name), p))
    frames.sort(key=lambda t: t[0])
    return frames


def read_lines_until(ser, predicate, timeout_s, on_line=None):
    """Read serial lines (best-effort ASCII) until predicate(line) is
    True or timeout_s elapses. Calls on_line(line) for every line seen."""
    deadline = time.time() + timeout_s
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", errors="ignore").strip()
                if not text:
                    continue
                if on_line:
                    on_line(text)
                if predicate(text):
                    return True
    return False


def main():
    ap = argparse.ArgumentParser(description="Replay recorded frames through the real ESP32 firmware")
    ap.add_argument("--frames-dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--hog-out", default=None, help="also write the HOG classifier's blinks here")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="stop after N frames (a whole session takes ~20 min over serial; "
                         "a few hundred frames is enough for the timing numbers)")
    args = ap.parse_args()

    frames = list_frames(args.frames_dir)
    if not frames:
        print(f"[ERROR] No {{timestamp}}.jpg frames found in {args.frames_dir}")
        sys.exit(1)
    if args.max_frames:
        frames = frames[:args.max_frames]
    print(f"Found {len(frames)} frames. First ts={frames[0][0]} Last ts={frames[-1][0]}")

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = args.baud
    ser.dtr = False
    ser.rts = False
    ser.timeout = 0.2
    ser.open()
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Wait for the board's ready banner so we don't race its boot sequence.
    print("Waiting for '#STATUS: Frame-injection mode ready'...")
    got_ready = read_lines_until(
        ser, lambda l: "Frame-injection mode ready" in l, timeout_s=15,
        on_line=lambda l: print(f"  boot: {l}"),
    )
    if not got_ready:
        print("[WARN] Didn't see the ready banner in 15s -- proceeding anyway.")

    events = []          # (ts, kind, payload)
    roi_locked = False
    roi_x = roi_y = 0
    lock_conf = 0.0
    total_blinks = 0
    hog_blinks = []      # (ts, score)
    timings = []         # (ear_us, hog_us) per frame, as measured on the chip

    def handle_line(text):
        nonlocal roi_locked, roi_x, roi_y, lock_conf, total_blinks
        m = TIMING_RE.search(text)
        if m:
            timings.append((int(m.group(1)), int(m.group(2))))
            return
        m = HOG_BLINK_RE.search(text)
        if m:
            hog_blinks.append((int(m.group(1)), float(m.group(2))))
            return
        m = LOCK_RE.search(text)
        if m:
            roi_x, roi_y = int(m.group(1)), int(m.group(2))
            lock_conf = float(m.group(4))
            roi_locked = True
            print(f"  [LOCKED] x={roi_x} y={roi_y} conf={lock_conf:.2f}")
            events.append((None, "lock", (roi_x, roi_y, lock_conf)))
            return
        if REJECT_RE.search(text):
            print(f"  [REJECT] {text}")
            return
        m = BLINK_RE.search(text)
        if m:
            ts = int(m.group(1))
            total_blinks += 1
            print(f"  [BLINK #{total_blinks}] t={ts}")
            events.append((ts, "blink", None))
            return
        if ERROR_RE.search(text):
            print(f"  {text}")

    t0 = time.time()
    for i, (ts, path) in enumerate(frames):
        with open(path, "rb") as f:
            jpeg_bytes = f.read()
        im = cv2.imdecode(np.frombuffer(jpeg_bytes, np.uint8), cv2.IMREAD_UNCHANGED)
        if im is None:
            print(f"[WARN] {path} is not a decodable JPEG -- skipping")
            continue
        h, w = im.shape[:2]

        packet = SOF + struct.pack("<IIII", ts, w, h, len(jpeg_bytes)) + jpeg_bytes + EOF_MARKER
        # Written in chunks, not one burst: the ESP32-S3's USB CDC receive
        # buffer is far smaller than a frame, and a full-speed burst overruns
        # it -- the board then reads the header and stalls waiting for a JPEG
        # that was partly dropped ("#ERROR: frame read failed, resyncing").
        for off in range(0, len(packet), CHUNK):
            ser.write(packet[off:off + CHUNK])
            ser.flush()

        ok = read_lines_until(
            ser,
            lambda l, _ts=ts: bool(FRAME_DONE_RE.search(l)),
            timeout_s=5.0,
            on_line=handle_line,
        )
        if not ok:
            print(f"[WARN] No FRAME_DONE for frame ts={ts} (#{i}) within 5s -- continuing")

        if (i + 1) % 50 == 0:
            print(f"... {i + 1}/{len(frames)} frames sent")

    elapsed = time.time() - t0
    ser.close()

    with open(args.out, "w") as f:
        f.write("timestamp_ms,processed,ear,is_blink_event,rolling_rate_bpm,roi_x,roi_y,roi_locked,lock_conf\n")
        running_blinks = 0
        for ts, kind, payload in events:
            if kind == "blink":
                running_blinks += 1
                f.write(f"{ts},1,,1,{running_blinks}.00,{roi_x},{roi_y},{int(roi_locked)},{lock_conf:.2f}\n")
    if args.hog_out:
        with open(args.hog_out, "w") as f:
            f.write("timestamp_ms,score\n")
            f.writelines(f"{ts},{score:.2f}\n" for ts, score in hog_blinks)

    print("\n=== frame_inject_replay summary ===")
    print(f"Total frames sent       : {len(frames)}")
    print(f"Wall-clock time taken   : {elapsed:.1f}s")
    print(f"ROI locked              : {'yes' if roi_locked else 'no'} "
          f"(x={roi_x} y={roi_y} conf={lock_conf:.2f})" if roi_locked else "ROI locked: no")
    print(f"Total blink events      : {total_blinks}")
    print(f"HOG classifier blinks   : {len(hog_blinks)}")
    if timings:
        # Per-frame cost measured on the ESP32 itself. Compare hog against the
        # camera build's frame interval (~81 ms in session_101) to see what the
        # classifier costs the recording frame rate.
        ear = sorted(t[0] for t in timings)
        hog = sorted(t[1] for t in timings if t[1] > 0)
        med = lambda v: v[len(v) // 2] / 1000.0
        p90 = lambda v: v[int(len(v) * 0.9)] / 1000.0
        print(f"on-chip decode+EAR      : median {med(ear):.1f} ms, p90 {p90(ear):.1f} ms ({len(ear)} frames)")
        if hog:
            print(f"on-chip HOG classifier  : median {med(hog):.1f} ms, p90 {p90(hog):.1f} ms ({len(hog)} frames after lock)")
    print(f"Output CSV              : {args.out}")


if __name__ == "__main__":
    main()
