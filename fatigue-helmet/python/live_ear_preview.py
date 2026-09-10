#!/usr/bin/env python3
"""
live_ear_preview.py
====================
Live camera feed from the ESP32, with the ESP's OWN on-device blink
detector's state drawn on top -- not a PC-side Haar cascade guess.

Requires firmware flashed with `pio run -e esp32s3cam_ear_preview
-t upload`. That build streams QVGA video over USB (like esp32s3cam) but
also runs the real on-device EAR/glint pipeline (normally SD-mode only)
on every frame before sending it, and prints its ROI lock + blink events
as plain-text status lines interleaved with the binary video frames.

This script:
  - decodes the binary video stream (same SOF/EOF framing as live_preview.py)
  - parses "#STATUS: EAR ROI motion-locked at x=.. y=.. size=.. conf=.."
    and draws a box at that exact location (rotated into display space)
  - flashes a red "BLINK" banner for ~500ms after each
    "#STATUS: blink (glint)" event line
  - overlays the live HR and blink-rate numbers from the 1 Hz status block

Usage:
    python live_ear_preview.py --port COM3

Press 'q' to quit.
"""

import argparse
import re
import struct
import sys
import time

import cv2
import numpy as np

try:
    import serial
except ImportError:
    print("[ERROR] pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

MAGIC_SOF = bytes([0xAA, 0xBB, 0xCC, 0xDD])
MAGIC_EOF = bytes([0xDD, 0xCC, 0xBB, 0xAA])
HEADER_LEN = 12
MAX_FRAME_B = 200_000  # QVGA JPEGs are small; generous headroom still kept

# Firmware's native (pre-rotation) frame size for esp32s3cam_ear_preview.
FRAME_W, FRAME_H = 320, 240  # QVGA -- must match CAMERA_FRAMESIZE in platformio.ini

ROI_LOCK_RE = re.compile(
    r"EAR ROI motion-locked at x=(\d+) y=(\d+) size=(\d+) conf=([\d.]+)"
)
ROI_REJECT_RE = re.compile(r"EAR motion lock rejected conf=([\d.]+)")
BLINK_EVENT_RE = re.compile(r"#STATUS: blink \(glint\)")
HR_RE = re.compile(r"HR:\s*(--|\d+)\s*BPM")
BLINK_RATE_RE = re.compile(r"BLINK:([\d.]+)")

BLINK_FLASH_SECONDS = 0.5


def roi_box_in_display_space(x, y, size):
    """Map an ROI box from the ESP's native QVGA frame to the rotated
    display frame (live_preview.py rotates 90deg CCW before showing), by
    rotating a mask instead of hand-deriving the transform."""
    mask = np.zeros((FRAME_H, FRAME_W), dtype=np.uint8)
    x2 = min(x + size, FRAME_W)
    y2 = min(y + size, FRAME_H)
    cv2.rectangle(mask, (x, y), (x2, y2), 255, -1)
    rotated = cv2.rotate(mask, cv2.ROTATE_90_COUNTERCLOCKWISE)
    ys, xs = np.where(rotated > 0)
    if len(xs) == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())


def print_status_line(line, state):
    m = ROI_LOCK_RE.search(line)
    if m:
        x, y, size, conf = int(m.group(1)), int(m.group(2)), int(m.group(3)), float(m.group(4))
        state["roi_box"] = roi_box_in_display_space(x, y, size)
        state["roi_conf"] = conf
        state["searching"] = False
        print(f"[LOCKED] ROI x={x} y={y} size={size} conf={conf:.2f}")
        return
    if ROI_REJECT_RE.search(line):
        state["searching"] = True
        return
    if BLINK_EVENT_RE.search(line):
        state["last_blink_time"] = time.time()
        state["blink_count"] += 1
        print(f"[BLINK] #{state['blink_count']}")
        return
    m = HR_RE.search(line)
    if m:
        state["hr"] = m.group(1)
    m = BLINK_RATE_RE.search(line)
    if m:
        state["blink_rate"] = float(m.group(1))


def draw_overlay(img, state):
    h, w = img.shape[:2]

    if state["roi_box"] is not None:
        x1, y1, x2, y2 = state["roi_box"]
        blinking = (time.time() - state["last_blink_time"]) < BLINK_FLASH_SECONDS
        color = (0, 0, 255) if blinking else (0, 255, 100)
        thickness = 3 if blinking else 2
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
        label = "BLINK!" if blinking else "eye (ESP-locked)"
        cv2.putText(img, label, (x1, max(0, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 2)
    elif state["searching"]:
        cv2.putText(img, "ESP searching for eye ROI...", (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

    hr_txt = f"HR: {state['hr']} BPM" if state["hr"] != "--" else "HR: --"
    cv2.putText(img, hr_txt, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.putText(img, f"Blink rate: {state['blink_rate']:.1f} /min", (10, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.putText(img, f"Live blinks seen: {state['blink_count']}", (10, 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.putText(img, "ESP on-device detection (live_ear_preview)", (10, h - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)


def print_text_clean(data: bytes, state):
    for line in data.split(b"\n"):
        if not line:
            continue
        try:
            text = line.decode("ascii", errors="ignore")
        except UnicodeDecodeError:
            continue
        if text.startswith("#") or text.startswith("["):
            print(text)
            print_status_line(text, state)


def main():
    parser = argparse.ArgumentParser(description="Live ESP-side eye/blink detection preview")
    parser.add_argument("--port", default="COM3", help="Serial port (default: COM3)")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    args = parser.parse_args()

    state = {
        "roi_box": None,
        "roi_conf": 0.0,
        "searching": True,
        "last_blink_time": 0.0,
        "blink_count": 0,
        "hr": "--",
        "blink_rate": 0.0,
    }

    print(f"Opening {args.port} @ {args.baud} baud...")
    try:
        ser = serial.Serial()
        ser.port = args.port
        ser.baudrate = args.baud
        ser.dtr = False
        ser.rts = False
        ser.timeout = 0.05
        ser.open()
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {args.port}: {e}")
        sys.exit(1)

    loading_img = np.zeros((FRAME_W, FRAME_H, 3), dtype=np.uint8)
    cv2.putText(loading_img, "Waiting for camera feed...", (20, FRAME_H // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imshow("ESP Eye/Blink Detection (live)", loading_img)
    cv2.waitKey(1)

    buf = b""
    print("Waiting for camera stream... (flash with 'pio run -e esp32s3cam_ear_preview -t upload')")

    try:
        while True:
            chunk = ser.read(8192)
            if chunk:
                buf += chunk

            while buf:
                sof_idx = buf.find(MAGIC_SOF)

                if sof_idx > 0:
                    print_text_clean(buf[:sof_idx], state)
                    buf = buf[sof_idx:]
                elif sof_idx == -1:
                    nl = buf.rfind(b"\n")
                    if nl >= 0:
                        print_text_clean(buf[: nl + 1], state)
                        buf = buf[nl + 1 :]
                    break

                if len(buf) < HEADER_LEN:
                    break

                timestamp_ms, jpeg_len = struct.unpack_from("<II", buf, 4)

                if jpeg_len > MAX_FRAME_B:
                    next_sof = buf.find(MAGIC_SOF, 1)
                    buf = buf[next_sof:] if next_sof != -1 else b""
                    continue

                total_needed = HEADER_LEN + jpeg_len + 4
                if len(buf) < total_needed:
                    break

                jpeg_data = buf[HEADER_LEN : HEADER_LEN + jpeg_len]
                eof_marker = buf[HEADER_LEN + jpeg_len : total_needed]

                if eof_marker != MAGIC_EOF:
                    next_sof = buf.find(MAGIC_SOF, 1)
                    buf = buf[next_sof:] if next_sof != -1 else b""
                    continue

                np_arr = np.frombuffer(jpeg_data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if img is not None:
                    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    draw_overlay(img, state)
                    cv2.imshow("ESP Eye/Blink Detection (live)", img)

                buf = buf[total_needed:]

            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Exiting...")
                break

    except KeyboardInterrupt:
        pass

    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
