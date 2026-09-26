# IoT-Fatigue — agent handoff

Motorcycle-helmet fatigue detector. An ESP32-S3-CAM in the helmet records heart rate, head motion,
and an eye camera, detects blinks on-device, and raises Safe/Warning/Critical alerts via a fuzzy
inference system (FIS). Goal: a labelled dataset (rested vs. fatigued rides), then reliable
on-device detection. Owner: Latif (solo). Repo: `github.com/walahow/IoT-Fatigue` (**public**, so
never commit secrets).

State as of 2026-09-26, carried over from ~25 Claude Code sessions.

## Read these first
- `CLAUDE.md`: architecture and phase history. Partly stale: blink detection is now the HOG
  classifier, not glint/EAR.
- **`docs/LABELING.md`: how to label a new session.** It covers the fatigue/KSS label (every ride) and the
  per-frame blink labels, with the exact conventions, the human GUI path, the AI contact-sheet path
  (`python/sweep_sheets.py`), committing labels, and retraining.
- **`docs/TRAINING.md`: how to train the blink classifier and tune the fuzzy model.** It covers setup, the command,
  the adoption bar (LOSO F1 > 0.693), shipping steps, and a table of ideas already tried that failed.
- **`docs/ROADMAP.md`: further development, prioritized.** It covers the fatigue dataset, sensors/optics, per-rider
  baselines, windowed features, and the untried blink-model ideas.
- `handout.md`: older blink-classifier handout (2026-09-23/24); its verification notes are still useful.
- `docs/claude-handoff/lab-notebook.md`: detailed findings per session (numbers, what failed, why).
- `docs/claude-handoff/artifacts/*.html`: session readout charts (101–104, 023), PPG trace.
- `fatigue-helmet/README.md`, `fatigue-helmet/firmware/SENSOR_TEST_GUIDE.md`, `fuzzy_walkthrough.md`,
  `thresholds_walkthrough.md`.

## Current state
- **Git:** `main` is pushed to GitHub as of 2026-09-27, including the current blink model (`c9b0a3799`) and
  these handoff docs. `sessions/` is gitignored (hundreds of MB); only small label files are force-added.
  Many stale feature branches exist; `main` is the truth.
- **On the helmet (flashed 2026-09-26):** `esp32s3cam_sd` with the newest blink model
  (`BLINK_HOG_BGSUB 1`, threshold −0.25). Boots cleanly; not yet used for a real ride.
- **Blink detection:** HOG (32×64 crop) + linear SVM in `firmware/src/EyeBlinkEAR.h`, weights in
  `firmware/src/BlinkWeights.h`. Latest variant scores `[frame, frame − running open-eye average]`
  (EMA α=0.05, ~2 s, 15-frame warm-up). Trained on sessions 119+121+122 with eye-aligned crops.
  Leave-one-session-out F1 0.693 (previous model 0.637). The glint detector still runs, log-only.
- **Phone arming preview** is on `main`: Wi-Fi AP `HELMET-A974`, page at `http://192.168.4.1`
  (arming checklist, live eye box, tap-to-set eye, optional blink test, ARM/ABORT/STOP). Wi-Fi turns
  off after arming. Builds need env var `HELMET_AP_PASS` (8–63 chars, already set on this PC).
  Its hardware bench test (Task 5 in `firmware/docs/superpowers/plans/2026-09-23-phone-arming-preview.md`) is **not done**.
- **Drift correction** (`earDriftTick()` in `main.cpp`, motion-based): validated in PC replay on
  session 116 and bit-identical on the real chip for the first ~183 s (8.7 ms/frame). The later
  big correction is confirmed in PC replay only.
- Native tests: `pio test -e native` gave **68 of 71 passing on 2026-09-26**. The 2 failures are
  `test_glint_blink_counted_on_recovery` and `test_glint_rolling_rate_evicts_old_blinks`, which were
  already failing before recent work, so they aren't regressions.

## Hardware
- Freenove ESP32-S3-WROOM CAM (OV2640, 8 MB PSRAM) on **COM3** (native USB 303A:1001). A plain
  ESP32-S3 N16R8 devkit (no camera) is also around for bench tests.
- Pulse sensor: analog, `PULSE_PIN 1` (ADC1). Contact is the weak point. A floating ADC still
  produces "valid" 72–119 BPM, so HR can be phantom.
- MPU-6050 (GY-521 **clone**): SDA=GPIO2, SCL=GPIO3, AD0→GND (0x68). `WHO_AM_I` = 0x74 (another unit
  read 0x38), so skip the ID check; library `testConnection()`/`begin()` fail on it. Gyro X bias ~+4 dps
  (calibrate), temperature reading is garbage, gyro noise ~100× datasheet. Details: `D:\proj\atttts\imu_test\IMU_INTEGRATION.md`.
- Buzzer `BUZZER_PIN 14` via BC547 transistor; start/stop button `BUTTON_PIN 21`. 1000 µF cap across
  3V3/GND (fine). The buzzer once sat beside the IMU and its EMI caused I2C dropouts.
- Serial needs `-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1`. PSRAM needs **both**
  `board_build.arduino.memory_type = qio_opi` and `-DBOARD_HAS_PSRAM`, or it silently falls back to QVGA.

## Firmware envs (`fatigue-helmet/firmware/platformio.ini`)
`esp32s3cam_sd` (production: SD recording + phone preview) · `esp32s3cam` (USB-tethered debug) ·
`esp32s3cam_ear_preview` (stream the chip's own eye/blink view to `live_ear_preview.py`) ·
`esp32s3cam_frame_inject` (feed recorded frames through the real chip via `frame_inject_replay.py`) ·
`esp32s3cam_test_sd` · `native` / `sensor_test` (host unit tests).

```bash
C:/Users/latif/.platformio/penv/Scripts/pio.exe run -e esp32s3cam_sd -t upload --upload-port COM3
```

## Recording procedure
1. Helmet on, no glasses, pulse sensor seated. Eye centre should sit at y≈100–130 in the 320×240
   frame, or the classifier crop falls off the bottom.
2. The eye coordinate is **stored in NVS and survives reflashing**. After reseating the camera it may be
   stale. Check the box on the phone page and re-tap the pupil. `ROI:auto` over serial clears it.
3. Press GPIO21 → arming. The IMU needs ~2 s of stillness. The HR baseline is the median of 20 valid readings
   after skipping 10 (needs ~30 valid s; times out at 60 s and forms later). The eye check needs ≥3
   classifier blinks within 30 s of lock.
4. Record ≥20 min. Press again to stop and wait for `Session closed — safe to remove SD card`.
5. Label it by following `docs/LABELING.md`. At minimum run
   `python label_session.py --session ../../sessions/session_XXX --kss N --condition rested|fatigued`.

Session folder: headerless `sensor_data.csv` (20 cols; col 19 `blink_valid`, col 20 `alert_gated`,
`alert_level` stays raw), `video.mjpeg` + index, `metadata.txt` (`armed_*`, `eye_check`,
`roi_source`), `blink_events.csv` (`timestamp_ms,source,score`, recording only, not arming).

## Tools
- `fatigue-helmet/python/`: `unpack_session.py`, `mjpeg_to_mp4.py` (overlays blink events),
  `frames_to_mp4.py` (pass `--codec mp4v`; needs a headered CSV / `dataset_merged.csv`),
  `session_summary.py` (quick verdict from CSV alone), `label_ground_truth.py` (eye/blink labelling GUI),
  `make_roi_track.py` (template-track the eye → `roi_track.csv`; **run before training on a new session**),
  `train_blink_classifier.py` / `train_blink_bgsub.py` / `train_blink_pooled.py`, `live_ear_preview.py --arm`,
  `frame_inject_replay.py`, `fuzzy_model.py`, `merge_session.py`.
- `fatigue-helmet/firmware/tools/session_replay/`: `session_replay.cpp` runs the firmware's eye code on a
  recorded session (`--hog-dump`, `--hog-model`, `--roi-track`, `--hog-jitter=N,J`). `sensor_sim.cpp`
  replays the 1 Hz FIS on a recorded CSV (`--baseline=`, `--blinks=`). `drift_test.cpp` replays drift correction.

## Fuzzy alert system
`firmware/src/FuzzyFatigue.h` mirrors `python/fuzzy_model.py`. **Change both together** and check
they agree with `sensor_sim`. Inputs: HR change vs. arming baseline, blink rate (3-min smoothed),
gyro variance, pitch/limp, nod score. Fixes already in place:
- Blink Low MF is (−∞,−∞,3,7). The old one returned 0 at blink=0.
- A `blink_valid` flag drops blink rules when the channel is offline.
- Rules below firing strength 0.25 are cut (`FIS_MIN_FIRING`).
- Nod only counts when the gyro is stable (so road bumps are not nods). `NodDetector.h` needs a ≥10° swing and ≥3 alternations.
- `AlertGate.h` enforces dwell times: Warning 5 s, Critical 8 s, 3 s release, 30 s buzzer cooldown after Critical.

## Data you should know
- Sessions 101, 119, 121, 122 (and 100, 116) are **the same rider**, with different camera placement.
  Cross-session blink accuracy is driven by **crop alignment**, not by rider.
- 101: fully hand-labelled (71 blinks). It is not yet aligned with the 121-based template (eye ≈ (252,100)).
- 102–104: rider wears **eyeglasses**. Blink detection is impossible there; it's an optics problem
  (needs a camera between lens and eye, or IR + NoIR). Don't tune anything on them. 104 has
  self-report ground truth (`sessions/session_104_ground_truth.xlsx`).
- 116: first real ride on the fixed firmware, fully hand-labelled (92 blinks, very bursty). Naively
  pooling all of 116 with 101 **made the linear SVM collapse**. Curate/align before pooling.
- The rider's true riding blink rate is ~4.6–7.5/min, which is already inside the literature's "fatigued" band.
  Absolute thresholds can't fit them, so per-rider baselines are needed.
- The recorded `alert_level` in 101–104 is an artifact of the old bugs. **Never use it as a fatigue label.**

## Open problems / next steps (full, prioritized list: `docs/ROADMAP.md`)
1. Ride with the new model, label the ride (`docs/LABELING.md`), and compare the on-device blink count with the truth.
2. Nod detector is unvalidated on real nods (sessions are 1 Hz and unlabelled). Record deliberate nods.
3. HR: contact noise and a floating ADC give phantom BPM. Needs bench data on real PPG amplitude vs. noise.
4. Fatigue labels: KSS at start and end, a 2-min stationary baseline, matched rested/tired rides on the same route.
   Log beat-to-beat intervals for HRV (1 Hz BPM can't give HRV).
5. Phone-preview bench test (frame rate with page open, pulse noise with Wi-Fi on/off, memory).
6. Optics: refocus the OV2640 for close range and keep the mount stable. Glare/hair/dark stretches can't be trained away.
7. Alerts are research data, **not a safety device**. Don't design tests that need a fatigued rider on a real road.

The user runs everything on this PC with the helmet on USB. For hardware steps, tell them what to
do physically and wait; capture serial yourself.
