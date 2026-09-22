# IoT-Fatigue Project Overview

## Project Summary

**IoT-Fatigue** is an integrated **firmware + data-pipeline system** for a motorcycle helmet-based fatigue detection platform. The system captures synchronized physiological data (heart rate, head movement via IMU) and camera footage to train ML models for detecting rider drowsiness.

**Hardware:** Freenove ESP32-S3-CAM microcontroller with analog pulse sensor + MPU-6050 6-axis IMU  
**Core Goal:** Record labeled training datasets across rested and fatigued states to enable real-time fatigue detection on-device

---

## Project Evolution

### Phase 1: Foundation (Early Development)
- **Scope:** Basic sensor integration and firmware scaffolding
- **Delivered:**
  - ESP32 firmware reading pulse sensor (ADC) and MPU-6050 (I2C)
  - CSV logging at 100 Hz with calibration routine
  - Python PC-side serial recorder for debug/USB mode (`debug_recorder.py`)
  - Wiring diagrams, sensor placement guides, hardware documentation

### Phase 2: Data Collection Pipeline
- **Scope:** End-to-end session recording and data organization
- **Delivered:**
  - SD card mode (`esp32s3cam_sd`) for standalone recording without PC tether
  - Session folder structure (metadata, sensor_data.csv, frames/)
  - MediaPipe-based eye tracking (`ear_validation.py`, `eye_ear.py`)
  - Eye Aspect Ratio (EAR) blink detection with baseline & cooldown logic
  - Fuzzy logic fatigue model simulation (`fuzzy_model.py`)
  - Frame-to-MP4 pipeline for post-session video assembly
  - KSS (Karolinska Sleepiness Scale) labeling protocol for dataset

### Phase 3: On-Device Blink Detection *(Current)*
- **Scope:** Reduce reliance on PC-side processing; enable edge-based blink counting
- **Key Achievements:**
  - **Otsu threshold binarization** for pupil isolation on ESP32
  - **Median ROI boot lock** to stabilize region-of-interest tracking
  - **Moment-based EAR computation** (centroid, principal axis) running natively on microcontroller
  - **Drift correction** with runtime calibration
  - **Baseline + cooldown** blink detector wired into firmware
  - **Glint detection** as a 3-cue fallback for eye localization (replaces older shape-based approach)
  - **Unit testing** via native PlatformIO environment for host-side algorithm validation
  - **Sensor test suite** (`SENSOR_TEST_GUIDE.md`, `analyze_sensor_test.py`) for pre-flight validation

---

## Current Project State

### Firmware (`fatigue-helmet/firmware/`)

| Component | Status | Notes |
|-----------|--------|-------|
| **Core Sensor Reading** | ✅ Stable | Pulse (ADC1) + MPU-6050 (I2C) with calibration |
| **CSV Logging** | ✅ Stable | 100 Hz output, timestamp syncing, signal quality tracking |
| **Camera Integration** | ✅ Stable | OV2640 frame capture, MJPEG streaming to PC or SD card |
| **SD Card Mode** | ✅ Stable | Standalone recording, no PC required; GPIO 21 button control |
| **On-Device Blink Detection** | ✅ Implemented | Otsu binarization, centroid tracking, glint detection, baseline blink rate |
| **PSRAM Management** | ✅ Fixed | Fallback logic for boards with/without PSRAM; no more init crashes |

**Environment Configurations:**
- `esp32s3cam` — USB debug mode (PC tethered)
- `esp32s3cam_sd` — SD card mode (standalone)
- `esp32s3cam_test_sd` — Test variant with debug output
- `sensor_test` — Host-side unit tests for algorithm validation

### Python Data Pipeline (`fatigue-helmet/python/`)

| Script | Status | Purpose |
|--------|--------|---------|
| `debug_recorder.py` | ✅ Stable | Stream USB sensor + camera frames to disk |
| `ear_validation.py` | ✅ Stable | Webcam EAR validation before recording sessions |
| `eye_ear.py` | ✅ Stable | MediaPipe-based blink detection, EAR computation, visualization |
| `merge_session.py` | ✅ Latest | Merge partial/interrupted sessions; realign timestamps |
| `frames_to_mp4.py` | ✅ Stable | High-quality video assembly from frame sequences |
| `mjpeg_to_mp4.py` | ✅ Stable | Extract video from SD card MJPEG/IDX format |
| `fuzzy_model.py` | ✅ Implemented | Fuzzy logic drowsiness classifier (post-session analysis) |
| `live_preview.py` | ✅ Implemented | Real-time video + overlay preview during recording |
| `visualize_tracking.py` | ✅ Stable | Debug eye tracking and blink detection results |
| `unpack_session.py` | ✅ Stable | Extract MJPEG from SD card sessions |

**Dataset Format:**
- Sensor CSV: `pc_timestamp`, `timestamp_ms`, `hr_bpm`, `pulse_raw`, `ax_g`, `ay_g`, `az_g`, `gx_dps`, `gy_dps`, `gz_dps`, `head_movement`, `signal_quality`, `label`
- Minimum session: **20 minutes** (~12,000 rows @ 100 Hz)
- Recommended: 5–10 rested + 5–10 fatigued sessions per model iteration

### Documentation & Protocols

| File | Content |
|------|---------|
| `README.md` | Hardware, firmware setup, Python setup, recording procedures |
| `SENSOR_TEST_GUIDE.md` | Pre-flight sensor validation checklist |
| `SENSOR_TEST_QUICKSTART.txt` | Quick reference for running hardware tests |
| `implementation_plan.md` | High-level firmware architecture & on-device algorithm roadmap |
| `thresholds_walkthrough.md` | EAR thresholds, blink rate normalization, filtering logic |
| `system_diagrams.md` | Component interaction diagrams, data flow, timing |
| `fuzzy_walkthrough.md` | Fuzzy logic model design, membership functions, rule base |
| `handoff.md` | Previous session notes & pending items |

### Datasets & Artifacts

| Location | Contents |
|----------|----------|
| `dataset & teory/` | Academic papers, reference datasets (e.g. DryAD eye-tracking benchmarks) |
| `sessions/` | Python-recorded sessions (USB mode) with CSV + frame folders |
| `docs/` | Architecture sketches, protocol documentation, measurement notes |
| `3D/` | 3D printable mounting brackets, sensor holders for helmet integration |

---

## Recent Improvements (Last 10 Commits)

1. **Glint detection overhaul** — Replaced shape-based blink detection with 3-cue glint detector (more robust to lighting)
2. **Drift check refinement** — Fixed noise in occlusion detection; currently disabled pending optimization
3. **Session replay validation** — Added host-side tool for verifying on-device blink detection output
4. **Percentile-based pupil localization** — Added threshold flexibility for varying lighting conditions
5. **Windowed darkest-density tracking** — Better ROI stability in high-contrast scenes
6. **PSRAM fallback robustness** — Fixed crash-loop on partial memory init
7. **Blink detector wiring** — Integrated baseline + cooldown logic into main firmware loop
8. **EAR computation** — Moment-based (centroid + axis) implementation on ESP32
9. **Median ROI boot lock** — Stabilizes initial eye region detection
10. **Otsu threshold + unit tests** — Host-side validation suite for binarization algorithm

---

## Architecture Highlights

### Firmware Design
```
┌─ Main Loop (100 Hz) ─────────────────────────────────┐
│                                                       │
├─ ADC Sampling (Pulse Sensor) → Heart Rate Filter    │
├─ I2C Read (MPU-6050) → Accel/Gyro Calibration      │
├─ Camera Capture (OV2640) → MJPEG encode             │
├─ On-Device Blink Detection                          │
│   ├─ Otsu Binarization                              │
│   ├─ Glint Detection (3-cue fallback)                │
│   ├─ Centroid Tracking + EAR Computation            │
│   └─ Baseline Detector + Cooldown Logic              │
├─ CSV Row Assembly & Write (SD or USB)               │
└─ Status LED + Button Control (GPIO 21)              │
```

### Data Flow
```
[ESP32 Sensors] ──┬──→ [CSV Log]           (sensor_data.csv)
                   ├──→ [Camera Frames]    (MJPEG or JPG seq)
                   └──→ [Blink Detection]  (on-device output)
                        │
                        ├──→ [SD Card Storage] (standalone mode)
                        └──→ [USB Serial]      (debug mode)
                             │
                             └──→ [Python PC] (debug_recorder.py)
                                  ├──→ Merge Session
                                  ├──→ EAR Validation
                                  └──→ MP4 Assembly + Labeling
```

### ML Pipeline
```
[Labeled CSV + Video] ──→ [Feature Extraction]
                          ├─ Blink rate (blink/min)
                          ├─ Heart rate variability
                          ├─ Head movement magnitude
                          └─ EAR statistics
                             │
                             ├──→ [Traditional ML] (Fuzzy, SVM, RF)
                             └──→ [Deep Learning] (CNN, LSTM on video)
```

---

## Known Limitations & Technical Debt

| Issue | Impact | Status |
|-------|--------|--------|
| **Drift correction** | Eye tracking drift in long sessions | 🔧 Re-enabled 2026-09-22, motion-based (reuses the boot lock instead of the old Otsu/darkness method); not yet run on a real ride with the new code |
| **Camera frame drops** | Occasional lost frames on high SD write load | ⏳ Buffering optimization in progress |
| **Low-light blink detection** | Otsu fails on very dark scenes | ⚠️ Glint fallback partially mitigates |
| **Heart rate filtering** | Noisy signal with poor skin contact | 📖 Documented in troubleshooting; user error mostly |
| **SD card I2C collision** | Rare lockups when simultaneous SD + I2C access | ⏳ Mutex lock planned |
| **No on-device drowsiness classification** | Blink detection only; classification still PC-side | 🎯 Future phase |
| **Limited model deployment** | Fuzzy model is reference only; no TensorFlow Lite yet | 🎯 Next phase |

---

## Next Steps & Future Work

### Short Term (1–2 weeks)
- [ ] **Validate drift correction on a real ride** — re-enabled and reasoned from the boot lock's own validated behaviour (see `earDriftTick()` in `main.cpp`), but no session has been recorded with it running yet; check `#STATUS: EAR drift corrected` lines against the video, and an `-DEAR_PROFILE` build's `drift=` timing, the way session_116 was checked by hand
- [ ] **Optimize frame buffering** — Investigate SD write stalls; potentially add ring buffer
- [ ] **Test glint fallback** — Validate 3-cue detector in various lighting conditions
- [ ] **Session merge improvements** — Handle edge cases (clock resets, incomplete frames)
- [ ] **Hardware stress test** — 4+ hour continuous recording to catch memory leaks

### Medium Term (1 month)
- [ ] **Collect labeled training dataset** — 10 rested + 10 fatigued sessions minimum (20 hrs total)
- [ ] **Evaluate blink-rate features** — Compare ML performance: blink rate alone vs. multimodal
- [ ] **TensorFlow Lite port** — Deploy lightweight classifier on ESP32 (LSTM or shallow CNN)
- [ ] **On-device inference** — Integrate drowsiness prediction into firmware alert logic
- [ ] **Optimize FLASH usage** — Current model leaves ~500 KB available; TFL might need pruning

### Long Term (2+ months)
- [ ] **Real-time alerts** — ESP32-side buzzer/LED warning when drowsiness detected
- [ ] **Bluetooth data sync** — Stream session metadata to companion app (smartphone)
- [ ] **Battery life profiling** — Measure LiPo drain under sustained recording
- [ ] **Multi-sensor fusion** — Integrate secondary modality (e.g., steering input) if available
- [ ] **Production firmware hardening** — Error recovery, watchdog timers, graceful shutdown
- [ ] **Compliance & safety** — Legal review for rider safety product status

---

## How to Contribute

### Recording Sessions
1. Check `SENSOR_TEST_GUIDE.md` to validate hardware (pulse signal quality, IMU axis orientation)
2. Decide condition: **Rested** (morning after sleep) or **Fatigued** (after activity/long awake)
3. Flash firmware: `pio run --target upload` (use `esp32s3cam_sd` for standalone)
4. Start recording via GPIO 21 button; run 20–40 minutes
5. Pull SD card, extract with `unpack_session.py` or `mjpeg_to_mp4.py`
6. **Label CSV:** add `label` column with KSS score (1–9) for all rows
7. Move to `sessions/` folder and tag with condition + timestamp

### Firmware Development
1. Read `implementation_plan.md` for architecture & rationale
2. Make changes in `fatigue-helmet/firmware/src/`
3. Run `pio run` to build; `pio run --target upload` to flash
4. Test on hardware first; then run host-side unit tests: `pio run -e sensor_test`
5. Check serial output: `pio device monitor` (115200 baud)
6. Commit with descriptive message; reference any PRs or issues

### Python Pipeline Enhancement
1. Check `requirements.txt` for dependencies; add new packages if needed
2. Scripts use MediaPipe (eye detection) and OpenCV (frame processing)
3. Add test data to `sessions/` for reproducibility
4. Test with `python script_name.py --help` and sample data
5. Document any new CLI arguments in the script's docstring

---

## Key References

### Hardware
- **ESP32-S3 Datasheet:** https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
- **MPU-6050 Datasheet:** https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
- **OV2640 Camera:** https://www.uctronics.com/download/cam_module/OV2640DS.pdf

### ML & Fatigue Detection
- **Karolinska Sleepiness Scale (KSS):** Åkerstedt & Gillberg (1990)
- **Eye Aspect Ratio (EAR):** Soukupová & Terzopoulos (2016) — "Real-Time Eye Blink Detection"
- **Reference Dataset:** DryAD eye-tracking benchmark (included in `dataset & teory/`)

### Tools & Libraries
- **PlatformIO:** https://platformio.org/
- **MediaPipe:** https://mediapipe.dev/
- **OpenCV:** https://opencv.org/
- **Fuzzy Logic (scikit-fuzzy):** https://scikit-fuzzy.github.io/

---

## Session Log

| Date | Duration | Condition | Notes |
|------|----------|-----------|-------|
| *To be populated during data collection* | — | — | — |

---

## Contact & Support

- **Project Lead:** Latif Hamzah (latifhamzah106@gmail.com)
- **Hardware Issues:** Check `README.md` troubleshooting table; validate with `SENSOR_TEST_GUIDE.md`
- **Code Issues:** See git history and inline comments for rationale
- **ML Training:** Refer to `fuzzy_model.py` for example; ready for scikit-learn pipeline integration
