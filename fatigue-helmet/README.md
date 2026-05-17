# IoT Helmet — Fatigue Detection System (Phase 1)

Phase 1 covers the sensor firmware and Python data-pipeline.
No camera is involved yet — that comes in Phase 2.

**Goal:** Record labelled physiological data (heart rate + head movement)
from a motorcycle helmet to train a fatigue-detection model.

---

## Table of Contents
1. [Hardware](#1-hardware)
2. [Firmware Setup](#2-firmware-setup-platformio)
3. [Python Setup](#3-python-setup)
4. [Recording a Session](#4-recording-a-session)
5. [EAR Validation](#5-ear-validation-webcam)
6. [CSV Dataset Format](#6-csv-dataset-format)
7. [Recording Protocol](#7-recording-protocol)
8. [Sensor Placement](#8-sensor-placement)

---

## 1. Hardware

### Components

| Part | Description | Interface |
|------|-------------|-----------|
| ESP32-S3 dev board | Main microcontroller | — |
| Analog Pulse Sensor | Heart rate (optical, 3-pin) | ADC — GPIO 1 |
| MPU-6050 GY-521 | 3-axis accelerometer + gyroscope | I2C — 0x68 |

MPU-6050 uses I2C; the Pulse Sensor is a simple 3-pin analog device. Total component cost is typically under $10.

### Wiring Diagram

```
                    ┌─────────────────────────────┐
                    │       ESP32-S3 Dev Board     │
                    │                              │
               ┌────┤ 3.3V ──────┬─────────────── │
               │    │            │                 │
               │    │ GND ───────┼─────────────── ─│─ GND rail
               │    │            │                 │
               │    │ GPIO 8 ────┤ SDA (I2C) ───── │─ MPU-6050 only
               │    │   (SDA)    │                 │
               │    │ GPIO 9 ────┤ SCL (I2C) ───── │─ MPU-6050 only
               │    │   (SCL)    │                 │
               │    │ GPIO 1 ────┤ Signal (analog) │─ Pulse Sensor S pin
               │    │   (ADC1)   │                 │
               │    └─────────────────────────────┘
               │
               │    ┌───────────────────┐   ┌───────────────────┐
               │    │  Analog Pulse     │   │    MPU-6050        │
               │    │  Sensor (3-pin)   │   │  (Accel + Gyro)   │
               └────┤ + (VCC)          │   │ VCC ──────────────┤
                    │ - (GND) ─────────┼───┤ GND               │
                    │ S → GPIO 1       │   │ SDA → GPIO 8      │
                    └───────────────────┘   │ SCL → GPIO 9      │
                                            │ AD0 → GND (0x68)  │
                                            └───────────────────┘
```

**Notes:**
- Pulse Sensor is analog-only — connect Signal (S) pin directly to GPIO 1 (ADC1).
- MPU-6050 AD0 must be tied to GND to keep its I2C address at 0x68.
- Pull-up resistors on SDA/SCL are built into the MPU-6050 breakout board.
- Use short wires (<15 cm) inside the helmet to keep I2C and analog signal quality high.

### Pin Summary

| ESP32-S3 GPIO | Connected to |
|---------------|-------------|
| 3.3V | Pulse Sensor + (VCC), MPU-6050 VCC |
| GND | Pulse Sensor − (GND), MPU-6050 GND |
| GPIO 1 (ADC1) | Pulse Sensor S (Signal) — analog in |
| GPIO 8 (SDA) | MPU-6050 SDA |
| GPIO 9 (SCL) | MPU-6050 SCL |

---

## 2. Firmware Setup (PlatformIO)

### Install PlatformIO

```bash
# Option A — VS Code extension (recommended)
# Install "PlatformIO IDE" from the VS Code marketplace.

# Option B — CLI
pip install platformio
```

### Build & Flash

```bash
cd fatigue-helmet/firmware

# Build only
pio run

# Build + upload
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor
```

### Expected Startup Output

```
[ESP32] #STATUS: Keep sensor still for calibration...
[ESP32] #STATUS: Starting in 3s...
[ESP32] #STATUS: Starting in 2s...
[ESP32] #STATUS: Starting in 1s...
[ESP32] #STATUS: Calibrating... keep still
[ESP32] #STATUS: Calibration done. Offsets: ax=42 ay=-18 az=16201 gx=3 gy=-7 gz=1
[ESP32] #STATUS: Pulse sensor on GPIO 1 (ADC1) — analog mode
[ESP32] #STATUS: Sampling at 100Hz, BPM averaged over 5 beats
[ESP32] #HEADER:timestamp_ms,hr_bpm,pulse_raw,ax_g,ay_g,az_g,...
[ESP32] #STATUS: Logging started. Format: CSV
100,0,2341,0.0021,-0.0013,0.0034,0.0012,-0.0008,0.0019,1.0023,1
200,72,2489,0.0019,-0.0011,0.0031,0.0009,-0.0007,0.0016,0.9998,1
```

The built-in LED blinks **3 times** at boot to confirm the firmware loaded.

### Troubleshooting

| Symptom | Fix |
|---------|-----|
| `#ERROR: MPU-6050 not found` | Confirm AD0 → GND; check I2C address |
| `hr_bpm` always 0 | Check `signal_quality` — if 0, sensor not touching skin |
| `pulse_raw` < 500 | No skin contact — press sensor firmly against skin |
| `pulse_raw` > 3500 | Too much ambient light or sensor saturated — cover sensor |
| No serial output | Add `-DARDUINO_USB_CDC_ON_BOOT=1` to `build_flags` |
| Upload fails | Hold BOOT button while uploading, then release |

---

## 3. Python Setup

```bash
cd fatigue-helmet/python

# Create virtual environment (recommended)
python -m venv venv
source venv/bin/activate      # Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

**Python 3.9+ required.** MediaPipe does not support 3.12 on all platforms;
3.10 or 3.11 is safest as of 2024.

---

## 4. Recording a Session

```bash
python serial_logger.py
# → prompts you to pick a COM port if --port is omitted

# Or specify everything:
python serial_logger.py --port COM3 --output rested_morning_01.csv
```

The script prints live stats every 5 seconds:

```
[   30s]  lines=   300  HR= 72 BPM  pulse_raw=2341  signal=OK  duration=0.5 min
[   35s]  lines=   350  HR= 73 BPM  pulse_raw=2489  signal=OK  duration=0.6 min
```

Stop recording with **Ctrl+C** — the CSV is flushed and closed cleanly.

---

## 5. EAR Validation (Webcam)

This script tests blink detection using your laptop webcam. Run it once
before a recording session to confirm MediaPipe is working on your machine.

```bash
python ear_validation.py
```

The video window shows:
- Eye contours highlighted
- EAR value, blink counter, blinks/min
- Colour-coded status bar (green = open, red = blink)

Press **Q** to quit. Terminal prints a summary line every 10 seconds.

Normal EAR range:
- Eyes open: 0.25 – 0.40
- Blink threshold: < 0.20
- Typical blink rate: 12–20/min (drowsy can drop to < 8/min)

---

## 6. CSV Dataset Format

Each row in the output CSV represents one 100 ms sensor snapshot.

| Column | Unit | Description |
|--------|------|-------------|
| `pc_timestamp` | ISO 8601 | Wall-clock time on the PC when the row arrived |
| `timestamp_ms` | ms | ESP32 `millis()` since boot |
| `hr_bpm` | BPM | Heart rate (0 if no valid beat or poor signal) |
| `pulse_raw` | 0–4095 | Raw 12-bit ADC reading from analog Pulse Sensor |
| `ax_g` | g | Calibrated X acceleration |
| `ay_g` | g | Calibrated Y acceleration |
| `az_g` | g | Calibrated Z acceleration |
| `gx_dps` | °/s | Calibrated X angular velocity |
| `gy_dps` | °/s | Calibrated Y angular velocity |
| `gz_dps` | °/s | Calibrated Z angular velocity |
| `head_movement` | g | Resultant acceleration magnitude √(ax²+ay²+az²); ~0 at rest |
| `signal_quality` | 0 or 1 | 1 = clear pulsatile waveform; 0 = no contact or poor placement |
| `label` | string | Fill after session: KSS 1–9 or descriptive tag |

**Calibration note:** `ax_g`, `ay_g`, `az_g` have gravity zeroed out during
the startup calibration. Values represent dynamic movement only.
`head_movement` ≈ 0 at rest and increases with helmet motion.

---

## 7. Recording Protocol

### Session Duration
Minimum **20 minutes** per session. 30–40 minutes is ideal for model training.

### Conditions

| Condition | When to record | Suggested label |
|-----------|---------------|-----------------|
| **A — Rested** | Morning, after ≥7 h sleep | `rested` / KSS 1–3 |
| **B — Fatigued** | After 4+ h of activity, 24 h awake, or post-lunch slump | `fatigued` / KSS 6–9 |

### KSS Scale (Karolinska Sleepiness Scale)

Fill the `label` column after the session using the subject's self-reported KSS score
at the time of recording:

| Score | Description |
|-------|-------------|
| 1 | Extremely alert |
| 2 | Very alert |
| 3 | Alert |
| 4 | Rather alert |
| 5 | Neither alert nor sleepy |
| 6 | Some signs of sleepiness |
| 7 | Sleepy, no effort to stay awake |
| 8 | Sleepy, some effort to stay awake |
| 9 | Very sleepy, great effort to stay awake |

**Tip:** Record a single KSS score for the whole session, or log it in a separate
notes file with timestamps if the condition changed mid-session.

### Labelling Procedure

1. After `serial_logger.py` finishes, open the CSV in a spreadsheet or text editor.
2. Add the KSS score (or descriptive label) to the `label` column for all rows.
3. Save. The file is now ready for model training.

### Recommended Dataset Size

| Condition | Sessions | Approximate rows |
|-----------|----------|-----------------|
| Rested    | 5–10     | 90 000 – 180 000 |
| Fatigued  | 5–10     | 90 000 – 180 000 |

---

## 8. Sensor Placement

```
          Top view of helmet
          ─────────────────
               ┌─────┐
               │ MPU │  ← Mount MPU-6050 at helmet crown, flat to skull
               │6050 │    Orientation: Z-axis pointing up
               └──┬──┘
                  │ wires routed inside padding
          ┌───────┴────────┐
          │                │
          │    HELMET      │
          │                │
          └──────┬─────────┘
                 │
           ┌─────┴──────┐
           │Pulse Sensor│  ← Firm skin contact required
           │ on earlobe │    signal_quality=0 if loose
           └────────────┘
```

**Analog Pulse Sensor (Heart Rate)**
- Requires FIRM skin contact — `signal_quality` = 0 means no contact or poor placement
- Best placement: earlobe or fingertip during recording sessions
- Avoid placement over bone with no underlying blood vessels
- Cover sensor from ambient light — excess light saturates ADC (`pulse_raw` > 3500)
- Secure with medical tape or a finger-clip attachment (3D-printable designs available online)

**MPU-6050 (IMU)**
- Mount at the top/crown of the helmet
- Keep it level and rigid — vibration from the mounting affects readings
- Cable tie or epoxy to helmet inner shell (avoid foam padding which allows movement)
- Note the axis orientation at calibration time; keep it consistent across sessions

**Cable Management**
- Route wires along the inner padding, away from the rider's neck
- Use a small LiPo battery (3.7 V, 500 mAh) or USB power bank inside the helmet
- Secure the ESP32 dev board in the helmet visor area or rear compartment
