#!/usr/bin/env python3
"""
Live terminal dashboard for the IoT Fatigue Helmet (esp32s3cam_sd firmware).

Reads the human-readable status block the firmware prints once per second
(see main.cpp's 1 Hz debug print) and renders it as a single refreshing
view instead of a scrolling log, plus a running blink counter driven by
the "#STATUS: blink (glint)" event lines.

Usage:
    python live_view.py [COM_PORT] [BAUD]

Defaults: COM3, 115200 (matches esp32s3cam_sd's monitor_speed).
Press Ctrl+C to stop.
"""

import re
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

HR_RE = re.compile(
    r"t:(\d+)ms\s+\|\s+HR:\s*(--|\d+)\s*BPM(.*?)PULSE:(\d+)\s+\[([^\]]+)\]"
)
DIFF_RE = re.compile(r"diff\s+([-\d.]+)%")
BLINK_RE = re.compile(
    r"BLINK:([\d.]+)\s+PITCH:([-\d.]+)deg\s+GVAR:(\d+)\s+NOD:([\d.]+)"
)
RISK_RE = re.compile(r"RISK:([\d.]+)%\s+ALERT:(\w+)")
IMU_DISCONNECTED = re.compile(r"IMU:\s*\[DISCONNECTED\]")
IMU_RESTORED = re.compile(r"MPU-6050 I2C connection restored")
BLINK_EVENT = re.compile(r"#STATUS: blink \(glint\)")

state = {
    "t_ms": 0,
    "hr": "--",
    "hr_diff": None,
    "pulse_raw": 0,
    "contact": "NO CONTACT",
    "blink_rate": 0.0,
    "pitch": 0.0,
    "gyro_var": 0,
    "nod": 0.0,
    "risk": 0.0,
    "alert": "safe",
    "imu_ok": True,
    "blink_events": 0,
}

BAR_W = 30


def bar(pct, width=BAR_W):
    pct = max(0.0, min(100.0, pct))
    filled = int(width * pct / 100.0)
    return "#" * filled + "-" * (width - filled)


def render():
    sys.stdout.write("\x1b[2J\x1b[H")  # clear screen, home cursor
    lines = []
    lines.append("=" * 56)
    lines.append("  IoT FATIGUE HELMET -- LIVE VIEW")
    lines.append("=" * 56)
    lines.append(f"  uptime: {state['t_ms']/1000.0:7.1f} s")
    lines.append("")

    hr_str = f"{state['hr']} BPM" if state["hr"] != "--" else "--   "
    diff_str = f" (diff {state['hr_diff']:+.1f}%)" if state["hr_diff"] is not None else ""
    lines.append(f"  HEART RATE   : {hr_str}{diff_str}")
    lines.append(f"    contact    : {state['contact']}   raw={state['pulse_raw']}")
    lines.append("")

    lines.append(f"  BLINK RATE   : {state['blink_rate']:.1f} bl/min")
    lines.append(f"    live blinks seen this session: {state['blink_events']}")
    lines.append("")

    imu_str = "OK" if state["imu_ok"] else "DISCONNECTED"
    lines.append(f"  IMU          : {imu_str}")
    lines.append(f"    pitch={state['pitch']:.1f}deg  gyro_var={state['gyro_var']}  nod={state['nod']:.2f}")
    lines.append("")

    lines.append(f"  FATIGUE RISK : [{bar(state['risk'])}] {state['risk']:.0f}%  ({state['alert'].upper()})")
    lines.append("")
    lines.append("-" * 56)
    lines.append("  Ctrl+C to stop")
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


def main():
    print(f"Connecting to {PORT} @ {BAUD}...")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    last_render = 0.0
    while True:
        try:
            raw = ser.readline()
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            return
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", errors="ignore").rstrip()
        except UnicodeDecodeError:
            continue

        if BLINK_EVENT.search(line):
            state["blink_events"] += 1

        m = HR_RE.search(line)
        if m:
            state["t_ms"] = int(m.group(1))
            state["hr"] = m.group(2)
            diff_m = DIFF_RE.search(m.group(3))
            state["hr_diff"] = float(diff_m.group(1)) if diff_m else None
            state["pulse_raw"] = int(m.group(4))
            state["contact"] = m.group(5).strip()

        m = BLINK_RE.search(line)
        if m:
            state["blink_rate"] = float(m.group(1))
            state["pitch"] = float(m.group(2))
            state["gyro_var"] = int(m.group(3))
            state["nod"] = float(m.group(4))

        m = RISK_RE.search(line)
        if m:
            state["risk"] = float(m.group(1))
            state["alert"] = m.group(2)

        if IMU_DISCONNECTED.search(line):
            state["imu_ok"] = False
        elif IMU_RESTORED.search(line):
            state["imu_ok"] = True

        now = time.time()
        if now - last_render > 0.2:
            render()
            last_render = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
