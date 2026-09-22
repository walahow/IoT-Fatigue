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
    python live_ear_preview.py --port COM3 --arm     # pre-flight check

--arm adds the arming check to run before a rider leaves: it shows the
classifier's crop as well as the locked ROI, then scores a two-phase blink
test (hold still, then blink on cue) and prints PASS/FAIL per check with a
READY TO RECORD verdict. Keys: SPACE/R run the test, q quits.

Press 'q' to quit.
"""

import argparse
import re
import struct
import sys
import textwrap
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
    r"EAR ROI (motion-locked|stored|manual) at x=(\d+) y=(\d+) size=(\d+) conf=(-?[\d.]+)"
)
ROI_SAVED_RE = re.compile(r"EAR ROI saved cx=(\d+) cy=(\d+)")
ROI_CLEARED_RE = re.compile(r"EAR ROI cleared")
ROI_REJECT_RE = re.compile(r"EAR motion lock rejected conf=([\d.]+)")
BLINK_EVENT_RE = re.compile(r"#STATUS: blink \(glint\)")
HOG_BLINK_RE = re.compile(r"#STATUS: blink \(hog\)")
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
        source, x, y, size, conf = (m.group(1), int(m.group(2)), int(m.group(3)),
                                    int(m.group(4)), float(m.group(5)))
        state["roi_box"] = roi_box_in_display_space(x, y, size)
        state["roi"] = (x, y, size)
        state["roi_conf"] = conf
        state["roi_source"] = source
        state["searching"] = False
        print(f"[ROI {source}] x={x} y={y} size={size} conf={conf:.2f}")
        return
    if ROI_SAVED_RE.search(line) or ROI_CLEARED_RE.search(line):
        state["roi"] = None          # the device re-applies it on the next frame
        state["roi_box"] = None
        state["searching"] = True
        return
    if ROI_REJECT_RE.search(line):
        state["searching"] = True
        return
    if BLINK_EVENT_RE.search(line):
        state["last_blink_time"] = time.time()
        state["blink_count"] += 1
        print(f"[BLINK] #{state['blink_count']}")
        return
    if HOG_BLINK_RE.search(line):
        # The classifier -- what the arming test scores, and what a future
        # build will drive blink_rate from.
        state["last_blink_time"] = time.time()
        state["hog_count"] += 1
        state["phase_blinks"] += 1
        print(f"[BLINK/classifier] #{state['hog_count']}")
        return
    m = HR_RE.search(line)
    if m:
        state["hr"] = m.group(1)
    m = BLINK_RATE_RE.search(line)
    if m:
        state["blink_rate"] = float(m.group(1))


def draw_overlay(img, state, text=True):
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

    if not text:      # arming mode: keep the picture clear, the panel carries the numbers
        return
    hr_txt = f"HR: {state['hr']} BPM" if state["hr"] != "--" else "HR: --"
    cv2.putText(img, hr_txt, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.putText(img, f"Blink rate: {state['blink_rate']:.1f} /min", (10, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.putText(img, f"Live blinks seen: {state['blink_count']}", (10, 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.putText(img, "ESP on-device detection (live_ear_preview)", (10, h - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)


# ── Arming (--arm): pre-flight check before a rider leaves ──────────────────
# Three things decide whether a session is worth recording, and all three are
# behavioural -- image statistics do not separate good footage from useless
# footage here (session_101's crop and the unusable glasses footage of
# session_102/104 score the same for sharpness, and overlap for brightness):
#   1. the ESP locks the eye at all, with confidence above its own gate
#   2. the classifier's crop fits INSIDE the frame, so it sees both lids
#   3. deliberate blinks are actually detected, and a still open eye is not
# Only extreme brightness is checked from the image, to catch a dead/covered
# camera or a blown-out exposure.
MIN_LOCK_CONF = 2.0        # firmware's EAR_MOTION_MIN_CONF
STILL_SECONDS = 8          # phase 1: hold still, eyes open
BLINK_SECONDS = 20         # phase 2: blink on cue
BLINK_TARGET = 10          # blinks to perform in phase 2
BLINK_MIN_DETECTED = 8     # of BLINK_TARGET
STILL_MAX_FALSE = 2        # detections allowed while holding the eye open
BRIGHT_MIN, BRIGHT_MAX = 35, 200


def display_to_native(px, py):
    """Inverse of the display rotation, by rotating a marker back rather than
    hand-deriving the transform (same trick as roi_box_in_display_space)."""
    mask = np.zeros((FRAME_W, FRAME_H), np.uint8)   # display space
    cv2.circle(mask, (int(px), int(py)), 2, 255, -1)
    back = cv2.rotate(mask, cv2.ROTATE_90_CLOCKWISE)
    ys, xs = np.where(back > 0)
    if len(xs) == 0:
        return None
    return int(xs.mean()), int(ys.mean())


def hog_crop_rect(x, y, size):
    """Mirror of EyeBlinkEAR::hogCropRect() -- the crop the classifier sees."""
    w = size * 15 // 8
    h = 2 * w
    return x + size // 2 - w // 2, y + size // 2 - h // 2, w, h


def crop_fit_status(x, y, size):
    cx, cy, cw, ch = hog_crop_rect(x, y, size)
    margin = min(cx, cy, FRAME_W - (cx + cw), FRAME_H - (cy + ch))
    return margin, (cx, cy, cw, ch)


def arm_tick(state, img_native):
    """Update the arming checks once per frame. img_native is the ESP's frame
    before display rotation, so crop coordinates match the firmware's."""
    now = time.time()
    if state["roi"] is not None:
        x, y, size = state["roi"]
        margin, (cx, cy, cw, ch) = crop_fit_status(x, y, size)
        state["crop_margin"] = margin
        sub = img_native[max(cy, 0):cy + ch, max(cx, 0):cx + cw]
        if sub.size:
            state["crop_bright"] = float(cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY).mean())

    phase, started = state["phase"], state["phase_start"]
    if phase == "still" and now - started >= STILL_SECONDS:
        state["still_false"] = state["phase_blinks"]
        state["phase"], state["phase_start"], state["phase_blinks"] = "blink", now, 0
    elif phase == "blink" and now - started >= BLINK_SECONDS:
        state["blink_detected"] = state["phase_blinks"]
        state["phase"] = "done"


def arm_checks(state):
    """-> list of (ok, label, hint). ok is None while a check can't be judged."""
    roi, margin = state["roi"], state["crop_margin"]
    conf, source = state["roi_conf"], state["roi_source"]
    # A stored/manual ROI was given, not found, so it has no confidence to
    # judge -- the blink test below is what validates it.
    given = source in ("stored", "manual")
    out = [(roi is not None and (given or conf >= MIN_LOCK_CONF),
            f"eye ROI {source}" + ("" if given else f" (confidence {conf:.1f}, needs {MIN_LOCK_CONF})"),
            "click the eye to set it, or let the rider blink while it searches")]
    out.append((margin is not None and margin >= 0,
                f"classifier crop fits in frame (margin {margin} px)" if margin is not None
                else "classifier crop fits in frame (waiting for lock)",
                "move the camera so the yellow box sits fully inside the picture"))
    b = state["crop_bright"]
    out.append((b is not None and BRIGHT_MIN <= b <= BRIGHT_MAX,
                f"exposure sane (eye region {b:.0f})" if b is not None else "exposure (waiting)",
                "too dark or blown out -- check the light and the lens"))
    if state["phase"] == "done":
        d, f = state["blink_detected"], state["still_false"]
        out.append((d >= BLINK_MIN_DETECTED,
                    f"blink test: {d} of {BLINK_TARGET} detected",
                    "the camera sees the eye but not the lids -- reposition, or retrain for this rider"))
        out.append((f <= STILL_MAX_FALSE,
                    f"false alarms while still: {f}",
                    "detections with the eye open: reposition, or retrain for this rider"))
    else:
        out.append((None, "blink test: press SPACE to start", ""))
    return out


def draw_arm_panel(img, state):
    checks = arm_checks(state)
    h, w = img.shape[:2]
    rows = [(ok, label, textwrap.wrap(hint, 52) if (ok is False and hint) else [])
            for ok, label, hint in checks]
    needed = 70 + sum(34 + 16 * len(hint) for _, _, hint in rows) + 40
    panel = np.zeros((max(h, needed), 430, 3), np.uint8)
    panel[:] = (32, 28, 26)
    cv2.putText(panel, "ARMING CHECK", (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    yy = 70
    for ok, label, hint in rows:
        color = (120, 120, 120) if ok is None else ((90, 220, 90) if ok else (60, 60, 240))
        mark = "..." if ok is None else ("PASS" if ok else "FAIL")
        cv2.putText(panel, mark, (14, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.putText(panel, label, (75, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (235, 235, 235), 1)
        for i, part in enumerate(hint):
            cv2.putText(panel, part, (75, yy + 16 + 15 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.38, (170, 170, 170), 1)
        yy += 34 + 16 * len(hint)

    h = panel.shape[0]
    phase = state["phase"]
    if phase == "still":
        left = STILL_SECONDS - (time.time() - state["phase_start"])
        msg, col = f"HOLD STILL, EYES OPEN  {left:.0f}s", (0, 200, 255)
    elif phase == "blink":
        left = BLINK_SECONDS - (time.time() - state["phase_start"])
        msg = f"BLINK {BLINK_TARGET} TIMES  {left:.0f}s  (counted {state['phase_blinks']})"
        col = (0, 200, 255)
    elif phase == "done":
        ready = all(c[0] for c in arm_checks(state))
        msg, col = ("READY TO RECORD" if ready else "NOT READY -- see above"), \
                   ((90, 220, 90) if ready else (60, 60, 240))
    else:
        msg, col = "click eye = set ROI   A = auto   SPACE = blink test   Q = quit", (200, 200, 200)
    cv2.putText(panel, msg, (14, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    canvas = np.zeros((h, w + panel.shape[1], 3), np.uint8)
    canvas[:] = (32, 28, 26)
    canvas[:img.shape[0], :w] = img
    canvas[:, w:] = panel
    return canvas


def draw_crop_box(img, state):
    """The classifier's crop, in display space -- must sit inside the picture."""
    if state["roi"] is None:
        return
    x, y, size = state["roi"]
    cx, cy, cw, ch = hog_crop_rect(x, y, size)
    mask = np.zeros((FRAME_H, FRAME_W), np.uint8)
    cv2.rectangle(mask, (max(cx, 0), max(cy, 0)),
                  (min(cx + cw, FRAME_W - 1), min(cy + ch, FRAME_H - 1)), 255, 2)
    rot = cv2.rotate(mask, cv2.ROTATE_90_COUNTERCLOCKWISE)
    img[rot > 0] = (0, 215, 255)


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
    parser.add_argument("--arm", action="store_true",
                        help="pre-flight arming check: verify the camera sees the eye, and that "
                             "deliberate blinks are detected, before a session is recorded")
    args = parser.parse_args()

    state = {
        "roi_box": None,
        "roi": None,
        "roi_conf": 0.0,
        "roi_source": "searching",
        "searching": True,
        "last_blink_time": 0.0,
        "blink_count": 0,
        "hog_count": 0,
        "hr": "--",
        "blink_rate": 0.0,
        # arming
        "phase": "idle",
        "phase_start": 0.0,
        "phase_blinks": 0,
        "still_false": 0,
        "blink_detected": 0,
        "crop_margin": None,
        "crop_bright": None,
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

    if args.arm:
        # Click the eye: the coordinate goes to the ESP, which stores it in
        # flash and uses it instead of searching -- for this and every later
        # session, until it is cleared with A or replaced by another click.
        def on_mouse(event, px, py, flags, _):
            if event != cv2.EVENT_LBUTTONDOWN or px >= FRAME_H:
                return                                  # clicks on the panel are not the eye
            native = display_to_native(px, py)
            if native is None:
                return
            cx, cy = native
            ser.write(f"ROI:{cx},{cy}\n".encode())
            print(f"[ARM] sent eye coordinate cx={cx} cy={cy}")

        cv2.namedWindow("ESP Eye/Blink Detection (live)")
        cv2.setMouseCallback("ESP Eye/Blink Detection (live)", on_mouse)

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
                    native = img
                    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    draw_overlay(img, state, text=not args.arm)
                    if args.arm:
                        arm_tick(state, native)
                        draw_crop_box(img, state)
                        img = draw_arm_panel(img, state)
                    cv2.imshow("ESP Eye/Blink Detection (live)", img)

                buf = buf[total_needed:]

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("Exiting...")
                break
            if args.arm and key == ord("a"):
                ser.write(b"ROI:auto\n")
                print("[ARM] cleared the stored coordinate -- the ESP will search again")
            if args.arm and key in (ord(" "), ord("r")):
                state.update(phase="still", phase_start=time.time(), phase_blinks=0,
                             still_false=0, blink_detected=0, reported=False)
                print(f"[ARM] hold still with eyes open for {STILL_SECONDS}s, "
                      f"then blink {BLINK_TARGET} times")
            if args.arm and state["phase"] == "done" and not state.get("reported"):
                state["reported"] = True
                print("\n=== arming check ===")
                for ok, label, hint in arm_checks(state):
                    print(f"  [{'PASS' if ok else 'FAIL'}] {label}")
                    if not ok and hint:
                        print(f"         -> {hint}")
                print("READY TO RECORD" if all(c[0] for c in arm_checks(state))
                      else "NOT READY -- fix the failures above and press R to retest")

    except KeyboardInterrupt:
        pass

    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
