# IoT Helmet — Fatigue Detection System

This project covers the full integrated sensor firmware and Python data-pipeline. 
We are using an **ESP32-S3-CAM** (Freenove WROOM CAM) to simultaneously capture both physiological data (heart rate + head movement) and camera data.

**Goal:** Record synchronized physiological data and facial/eye imagery
from a motorcycle helmet to train a fatigue-detection model.

---

## Table of Contents
1. [Hardware](#1-hardware)
2. [Firmware Setup](#2-firmware-setup-platformio)
3. [Firmware Environments](#3-firmware-environments)
4. [Python Setup](#4-python-setup)
5. [Recording a Session](#5-recording-a-session)
6. [Live Monitoring & On-Device Blink Debug Tools](#6-live-monitoring--on-device-blink-debug-tools)
7. [Labeling a Session](#7-labeling-a-session)
8. [EAR Validation](#8-ear-validation-webcam)
9. [CSV Dataset Format](#9-csv-dataset-format)
10. [Recording Protocol](#10-recording-protocol)
11. [Sensor Placement](#11-sensor-placement)

---

## 1. Hardware

### Components

| Part | Description | Interface |
|------|-------------|-----------|
| Freenove ESP32-S3 WROOM CAM | Main microcontroller + OV2640 Camera | — |
| Analog Pulse Sensor | Heart rate (optical, 3-pin) | ADC — GPIO 1 |
| MPU-6050 GY-521 | 3-axis accelerometer + gyroscope | I2C — 0x68 |

MPU-6050 uses I2C; the Pulse Sensor is a simple 3-pin analog device. Total component cost is typically under $10.

### Wiring Diagram

```
                    ┌──────────────────────────────┐
                    │  Freenove ESP32-S3 WROOM CAM │
                    │                              │
               ┌────┤ 3.3V                         │
               │    │                              │
               │    │ GND ─────────────────────────┼─ GND rail
               │    │                              │
               │    │ GPIO 2 (IO2) ───┤ SDA (I2C) ─│─ MPU-6050 only
               │    │                 │            │
               │    │ GPIO 3 (IO3) ───┤ SCL (I2C) ─│─ MPU-6050 only
               │    │                 │            │
               │    │ GPIO 1 (IO1) ───┤ Signal     │─ Pulse Sensor S pin
               │    │                 │            │
               │    └──────────────────────────────┘
               │
               │    ┌───────────────────┐   ┌───────────────────┐
               │    │  Analog Pulse     │   │    MPU-6050       │
               │    │  Sensor (3-pin)   │   │  (Accel + Gyro)   │
               └────┤ + (VCC)           │   │ VCC ──────────────┤
                    │ - (GND) ──────────┼───┤ GND               │
                    │ S → GPIO 1        │   │ SDA → GPIO 2      │
                    └───────────────────┘   │ SCL → GPIO 3      │
                                            │ AD0 → GND (0x68)  │
                                            └───────────────────┘
```

**Notes:**
- Pulse Sensor is analog-only — connect Signal (S) pin directly to GPIO 1 (ADC1).
- MPU-6050 AD0 must be tied to GND to keep its I2C address at 0x68.
- Pull-up resistors on SDA/SCL are built into the MPU-6050 breakout board.
- Use short wires (<15 cm) inside the helmet to keep I2C and analog signal quality high.

### Pin Summary

| Freenove ESP32-S3 GPIO | Connected to |
|------------------------|--------------|
| 3.3V | Pulse Sensor + (VCC), MPU-6050 VCC |
| GND | Pulse Sensor − (GND), MPU-6050 GND |
| GPIO 1 (ADC1) | Pulse Sensor S (Signal) — analog in |
| GPIO 2 | MPU-6050 SDA |
| GPIO 3 | MPU-6050 SCL |

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

## 3. Firmware Environments

`platformio.ini` defines several build environments. Pick one with `pio run -e <env>` /
`pio run -e <env> --target upload`; only one can be flashed at a time.

| Environment | Purpose | Monitor baud |
|-------------|---------|---------------|
| `esp32s3cam` (default) | USB debug mode — camera frames + sensor CSV streamed live to `debug_recorder.py` | 921600 |
| `esp32s3cam_sd` | **Production mode** — everything (CSV + MJPEG video) saved to microSD, no PC needed; GPIO 21 button starts/stops a session. Also runs the phone arming preview (Wi-Fi `HELMET-xxxx`, see section 6) unless the two `PHONE_PREVIEW` lines are removed | 115200 |
| `esp32s3cam_test_sd` | Hardware test mode — validates buzzer + sensors, SD enabled | 115200 |
| `esp32s3cam_ear_preview` | Debug — same USB video stream as `esp32s3cam`, but also runs the real on-device blink/EAR pipeline on each frame so `live_ear_preview.py` can overlay the ESP's own ROI lock + blink events on the live feed | 921600 |
| `esp32s3cam_frame_inject` | Debug — no live camera; the ESP receives pre-recorded JPEG frames one at a time over serial from `frame_inject_replay.py` and runs the real on-device pipeline on each, for validating firmware against reference footage | 921600 |
| `sensor_test` | Standalone 30s-per-sensor stability test (pulse, IMU, camera), independent of `main.cpp` | 921600 |
| `native` | Host-side (PC) unit tests for hardware-independent algorithm code — `pio test -e native -f test_eye_blink_ear -v` | — |

For day-to-day recording use `esp32s3cam_sd` (standalone) or `esp32s3cam` (tethered debug).
The others exist purely to debug the on-device blink detector without needing a full recording.

---

## 4. Python Setup

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

## 5. Recording a Session

Recording depends on which firmware environment is flashed.

**USB debug mode** (`esp32s3cam`) — sensor CSV *and* camera frames stream to the PC:

```bash
python debug_recorder.py
# → defaults to COM4 @ 921600 baud

# Or specify:
python debug_recorder.py --port COM3 --baud 921600
```

Each run creates its own session folder:

```
sessions/session_001/
    metadata.txt
    sensor_data.csv
    frames/{timestamp_ms}.jpg
```

Stop with **Ctrl+C** — files are flushed and closed, then a summary prints:

```
[RECORDER] Session ended after 312.4s
[RECORDER] CSV rows  : 312
[RECORDER] Frames    : 6240
[RECORDER] Bad frames: 0
[RECORDER] Saved to  : sessions/session_001
```

**SD card mode** (`esp32s3cam_sd`) — no PC needed. The ESP32 writes
`sensor_data.csv`, `video.mjpeg`, `video.idx` and `blink_events.csv` (one row per blink: `timestamp_ms,source,score`, source `hog` = the classifier that drives `blink_rate`, `glint` = the old detector, kept for comparison; blinks seen during arming are not included) straight to the card; press the
GPIO 21 button to start/stop a session. Pull the card afterwards and use
`unpack_session.py` or `mjpeg_to_mp4.py` to extract the video.

---

## 6. Live Monitoring & On-Device Blink Debug Tools

These are diagnostic tools for watching or validating the on-device blink detector —
none of them are required for a normal recording session.

**`live_view.py`** — real-time terminal dashboard while `esp32s3cam_sd` (or any env
printing the 1 Hz debug block) is connected over USB:

```bash
python live_view.py [COM_PORT] [BAUD]
# defaults: COM3, 115200
```

Renders heart rate, blink rate, IMU status, and fatigue risk as a single
refreshing screen instead of a scrolling log. Ctrl+C to stop.

**`live_ear_preview.py`** — live camera feed with the ESP's own eye-ROI lock and
blink events drawn on top (not a PC-side guess):

```bash
# flash first:
pio run -e esp32s3cam_ear_preview --target upload
# then:
python live_ear_preview.py --port COM3
```

Press `q` to quit.

**`live_ear_preview.py --arm`** — pre-flight arming check, run before the rider
leaves. On top of the live preview it draws the classifier's crop (yellow) as
well as the locked ROI, and scores a two-phase blink test:

```bash
pio run -e esp32s3cam_ear_preview --target upload
python live_ear_preview.py --port COM3 --arm
```

**Click the eye** in the preview to set the ROI by hand. The coordinate is sent
as `ROI:<cx>,<cy>`, kept in the ESP's NVS (it survives power cycles *and*
reflashing to another build), and applied on the first frame of every later
session — so detection runs from the start instead of after the localizer's
~20 s search, which is also the search that locked 252 px off the pupil in
session_100. `A` sends `ROI:auto`, which clears it and goes back to searching.
`metadata.txt` records `roi_source` (`stored`/`motion`/`manual`) with the
coordinates, so a session whose ROI was pointing at a cheek can be told apart
from a rider who genuinely wasn't blinking.

A stored coordinate is only valid while the camera sits the same way on the
head, and a stale one fails silently — hence the blink test below, every time
the helmet goes on.

Press SPACE, hold still with eyes open for 8 s, then blink 10 times. It reports
PASS/FAIL for: eye locked above the firmware's own confidence gate; the
classifier's crop fully inside the frame (it must see both lids); sane
exposure; blinks detected; and no false alarms while the eye is open. Verdict is
READY TO RECORD or the failures with what to change. Re-test with R.

The eye lock is taken once and frozen, so after moving the camera, reset the
board before re-testing. Note that the checks are behavioural on purpose:
image sharpness does NOT separate good footage from unusable footage here
(session_101 and the glasses sessions 102/104 score the same), so only the
blink test can tell you the mount will actually work.

**`frame_inject_replay.py`** — replays a folder of previously recorded frames
(`{timestamp_ms}.jpg`, as produced by `debug_recorder.py`/`unpack_session.py`)
through the real compiled firmware to check its blink/ROI output against
reference footage:

```bash
pio run -e esp32s3cam_frame_inject --target upload
python frame_inject_replay.py --frames-dir ../../sessions/session_067/frames --out session_067_esp_result.csv
```

Compare its output CSV against `tools/session_replay`'s PC-side replay of the
same algorithm — if they disagree, something differs between the host build
and the actual ESP32 binary.

This build needs **no camera**: frames arrive over USB, so a bare ESP32-S3 dev
board stands in for the ESP32-S3-CAM, provided it has PSRAM (the decode buffers
are ~380 KB; `board_build.arduino.memory_type` must match the module — `qio_opi`
for octal/R8, `qio_qspi` for quad/R2). Camera/MPU/SD init errors on boot are
expected there and harmless. Each `#FRAME_DONE` line carries the chip's own
per-frame cost (`ear=` decode + localizer + classifier, `hog=` the classifier's
share), which the script summarises — this is how the classifier's cost is
measured without the camera board. It does not exercise live capture, SD
writes, or the sensor tasks.

**`train_blink_classifier.py`** — trains the firmware's per-frame HOG blink
classifier (`HogBlinkDetector` in `EyeBlinkEAR.h`) from hand labels. Needs
`<session>/blink_labels.csv` (`timestamp_ms,frame_idx,state`, state =
`open`/`closed`/`squint`/`unsure`) and a built `tools/session_replay`
(`g++ -O2 -std=gnu++14 -o session_replay.exe session_replay.cpp`):

```bash
python train_blink_classifier.py --session ../../sessions/session_101
```

Features are computed by `session_replay --hog-dump` (the firmware's own code),
the cross-validation is scored by running `session_replay --hog-model` (the
firmware's scoring and blink counting), and the final model is written to
`<session>/blink_model.bin` and, as the same numbers, to
`firmware/src/BlinkWeights.h` for the next flash. The weights are rider- and
mount-specific.

On the device the classifier runs alongside the glint detector and prints
`#STATUS: blink (hog) t=... score=...`; the glint detector still drives
`blink_rate`. To check the chip reproduces the PC replay (JPEG decoder, float
maths), inject the same frames and compare blink timestamps:

```bash
pio run -e esp32s3cam_frame_inject --target upload
python frame_inject_replay.py --frames-dir ../../sessions/session_101/frames \
    --out esp_101.csv --hog-out esp_101_hog.csv
../firmware/tools/session_replay/session_replay.exe ../../sessions/session_101/frames pc_101.csv \
    --hog-model=../../sessions/session_101/blink_model.bin --hog-csv=pc_101_hog.csv
```

**`tools/session_replay/sensor_sim`** — the firmware's 1 Hz loop on the PC:
rebuilds the `sensor_data.csv` the ESP would have written for a recorded
session, with `blink_rate`, `risk_pct` and `alert_level` recomputed by the
firmware's own `FuzzyFatigue.h` from any blink source (e.g. the `--hog-csv`
above). Without `--blinks` it uses the recorded blink rate, which must
reproduce the recorded risk — that is the tool's self-check. The ESP forms the
HR baseline partly during arming, before the CSV starts, so pass `--baseline`
if the first-20-rows default doesn't reproduce the recording (86 for session_101).

```bash
cd ../firmware/tools/session_replay
g++ -O2 -std=gnu++14 -Ihost_shim -o sensor_sim.exe sensor_sim.cpp
./sensor_sim.exe ../../../../sessions/session_101/sensor_data.csv sim.csv --baseline=86 --blinks=pc_101_hog.csv
```

### Phone arming preview (no laptop)

The `esp32s3cam_sd` build brings up its own Wi-Fi, `HELMET-xxxx`. Its password
is read at build time from the `HELMET_AP_PASS` environment variable (8+
characters, letters/digits/dashes); it is not in the repo, which is public.
Set it once, then open a new terminal (restart VS Code if you build from the
PlatformIO extension) before building:

```bash
setx HELMET_AP_PASS your-pass-here
```

Without it, building `esp32s3cam_sd` stops with a message saying so. Join the
network from the phone (if the phone says it has no internet, choose to stay
connected) and open **http://192.168.4.1**:

- **Banner** — `IDLE`, `ARMING n / 60 s`, `RECORDING`.
- **Picture** — the helmet camera, rotated as in `live_ear_preview.py`, with
  the ESP's eye box (green) and the classifier's crop (yellow). Red flash on
  each detected blink.
- **Tap the eye** to place the eye box — the same `ROI:<cx>,<cy>` the PC tool
  sends, stored in NVS the same way. **Auto eye** clears it. Setting it during
  ARMING restarts the eye check (3 blinks) from the new box; it can't be
  changed while recording.
- **ARM / ABORT / STOP** — one more way to press the GPIO 21 button. The
  button itself works as before, phone or no phone — except for a moment
  (about 0.1–0.3 s, logged as `took N ms`) when Wi-Fi switches off 15 s into a
  recording and when it comes back after you stop: a very quick tap then can
  be missed, so press and hold briefly.
- **Arming** checklist — the firmware's own gate: IMU, HR baseline, eye check.
- **Blink test** (optional, as SPACE in `live_ear_preview.py --arm`) — hold
  still 8 s, then blink 10 times; READY / NOT READY. Arming never waits for it.

Wi-Fi goes off 15 s into a recording, so the ride records exactly as without
it, and comes back when the session stops. Stop a recording with the button.
If `HELMET-xxxx` never appears after boot (the serial log shows
`#ERROR: Phone preview: ...`), press the button twice (arm, then abort) to
retry, or power-cycle. If the page won't load on a phone that has used
192.168.4.1 for something else (a router, another ESP), clear that site's
data in the browser: the helmet rejects requests with more than 1 KB of
headers. The page's pure logic is checked with
`node tools/phone_page_test.js`.

---

## 7. Labeling a Session

Standalone SD-card sessions (`esp32s3cam_sd`) write `sensor_data.csv` straight
from the firmware with no `label` column. Attach a KSS score after the ride:

```bash
python label_session.py --session ../../sessions/session_023 --kss 3
python label_session.py --session ../../sessions/session_023 --kss 3 --condition rested
```

Writes a new `sensor_data_labeled.csv` (raw file untouched) with the KSS value
repeated on every row, and records `kss=`/`condition=` in `metadata.txt`.

---

## 8. EAR Validation (Webcam)

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

## 9. CSV Dataset Format

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

## 10. Recording Protocol

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

1. After a session finishes, open its `sensor_data.csv` in a spreadsheet or text editor.
2. Add a `label` column and fill in the KSS score (or descriptive label) for all rows.
   The firmware does not emit this column — it is added by hand after recording.
3. Save. The file is now ready for model training.

**Note:** drop rows where `imu_valid` is `0` before computing IMU statistics. Those
rows carry frozen last-known-good values held while IMU reads were suspended
(buzzer sounding, or bus down), not fresh measurements — counting them treats one
reading as several. Sessions recorded before that column existed have 17 columns
and no validity flag.

`blink_valid` (19th column) is `0` while the on-device blink channel is offline:
eye not locked, the arming eye check failed or is pending (fewer than 3 blinks
within 30 s of the lock), the first 60 s of the rolling window, or no frames being
processed. On those rows `blink_rate` is not evidence of anything -- a 0 means
"cannot see", not "eyes closed" -- and the fuzzy model ignores blink (HR and IMU
decide alone), so `risk_pct`/`alert_level` there rest on HR and IMU only. Drop or
down-weight `blink_valid == 0` rows before using `blink_rate` as a feature.
Sessions with 17/18 columns have no such flag (unknown, not assumed valid).
`metadata.txt` records `eye_check=pass|fail|pending` and `eye_check_blinks`. The
check proves the classifier is running on a locked ROI and saw blinks; it cannot
prove the classifier suits the rider (it is trained on one rider without glasses).

`alert_gated` (20th column) is `alert_level` after the dwell/release gate
(`AlertGate.h`): Warning needs 5 s and Critical 8 s of sustained raw alert, and a
level clears after 3 s below it. It is what the buzzer follows (Critical: 2 s beep
then 30 s silence; Warning: 1 s beep then 3 s). `alert_level` stays the raw model
output, so both are available for analysis.

### Recommended Dataset Size

| Condition | Sessions | Approximate rows |
|-----------|----------|-----------------|
| Rested    | 5–10     | 90 000 – 180 000 |
| Fatigued  | 5–10     | 90 000 – 180 000 |

---

## 11. Sensor Placement

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
