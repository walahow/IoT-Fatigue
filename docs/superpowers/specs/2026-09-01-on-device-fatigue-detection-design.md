# On-device fatigue detection + PC live view

Date: 2026-09-01
Status: approved by user, pending implementation plan

## 1. Problem

The rider-facing goal of this project is a helmet that detects fatigue and
alerts the rider on its own, untethered from a PC. Today it doesn't do that:

- `blink_rate`, one of the three inputs to the on-device `FuzzyFatigue` risk
  engine ([firmware/src/main.cpp:1110-1137](../../../fatigue-helmet/firmware/src/main.cpp)),
  is only ever set by parsing a `BLINK:<float>\n` line off the serial port.
  If nothing sends one, it silently defaults to `13.0` (dead-center
  "normal") forever.
- The only code that computes a real blink rate from camera frames is
  [`eye_ear.py`](../../../fatigue-helmet/python/eye_ear.py) — and it only
  runs **offline**, after a session, on JPEG frames already saved to disk.
  Nothing computes it live today. The only script that ever writes
  `BLINK:` to the serial port live is `test_firmware_serial.py`, a manual
  test-value injector, not a real detector.
- SD-card mode (the field-deployment env, `esp32s3cam_sd`) saves camera
  frames straight to the card and never streams them anywhere, so there is
  also no way to watch the system work from a PC while it records.

So in every real recording session so far, the fatigue alert has only ever
reacted to HR and IMU — the blink input was a constant.

## 2. Goals

1. The ESP32 computes a **real, live blink rate from the camera itself**,
   with no PC required, and feeds it into the existing fuzzy risk engine —
   so the buzzer genuinely reacts to real blinks during an untethered ride.
2. A PC can optionally watch the camera feed and the live telemetry
   (HR/IMU/blink/risk) **while SD recording continues unaffected**, for
   development and verification — not as a requirement for the detection
   itself to work.
3. Zero regression to the existing dataset-recording path: `esp32s3cam_sd`'s
   file formats (`video.mjpeg`, `video.idx`, `sensor_data.csv`) and the USB
   debug workflow (`esp32s3cam`, `debug_recorder.py`, offline `eye_ear.py`)
   are unchanged.

## 3. Non-goals

- On-device deep learning / ESP-WHO / esp-dl. No bundled eye-state model
  exists for this hardware; training one is a separate, much larger
  project and isn't needed — the EAR algorithm already in `eye_ear.py` is
  classical image processing, not ML, and is what gets ported instead.
- Changing the production dataset video quality (VGA @ 20fps stays as-is
  in `esp32s3cam_sd`).
- NVS-based runtime ROI calibration UI. A compile-time override is enough
  for now (see §5.4); a friendlier runtime calibration flow is a possible
  future enhancement, not part of this spec.

## 4. Architecture overview

```
Camera capture (unchanged, core 0)
        |
        +----------------------------> Save to SD (unchanged: video.mjpeg, video.idx)
        |
        v
On-device EAR pipeline (new)
  decode JPEG -> crop to locked ROI -> Otsu threshold -> accumulate
  moments (m20, m02, m11) -> eigenvalue ratio (EAR) -> blink detector
  (baseline + cooldown) -> 60s rolling blink rate
        |
        v
FuzzyFatigue engine (unchanged) <---- HR diff %, IMU (gyro var, pitch, nod)
        |
        v
Risk % / alert level -> buzzer (unchanged)
                      -> optional: serial line -> PC live view (new, debug only)
```

The on-device EAR pipeline is core firmware logic present in **both** SD
environments (`esp32s3cam_sd` and the new `esp32s3cam_sd_live`) — it is not
something that only runs when a PC is attached. The PC live view is a
verification tool layered on top, not the mechanism itself.

## 5. On-device EAR pipeline

### 5.1 Why moments instead of contour + ellipse fit

`eye_ear.py`'s EAR is "minor-axis / major-axis of an ellipse fit to the
thresholded eye blob," normally computed via contour extraction +
`cv2.fitEllipse`. Porting contour tracing and ellipse fitting to plain C is
real work and easy to get subtly wrong at the edges (blob touching the crop
border, degenerate point sets, etc).

The same minor/major ratio can be computed directly from the blob's
second-order **image moments** — no contour tracing needed:

1. Threshold the eye crop (Otsu) into a binary mask.
2. One pass over the mask accumulates three running sums, relative to the
   blob's centroid `(x̄, ȳ)`:
   `μ20 = Σdx²`, `μ02 = Σdy²`, `μ11 = Σdx·dy`.
3. These three numbers form a 2×2 covariance matrix. Its eigenvalues (a
   closed-form quadratic, no iterative solver) are the squared axis
   lengths of the "ellipse with the same second moments" — the same
   geometric quantity `cv2.fitEllipse` reports, computed a different way:

   ```
   trace = μ20 + μ02
   diff  = sqrt((μ20 - μ02)² + 4·μ11²)
   λ_major = (trace + diff) / 2
   λ_minor = max((trace - diff) / 2, 0)   // clamp for float noise
   EAR = sqrt(λ_minor) / sqrt(λ_major)
   ```

This is one linear pixel-scan pass plus a handful of scalar ops — bounded
cost, no dynamic allocation, no boundary-following logic, well-defined for
any nonempty mask (just guard `μ00 == 0` → "no eye detected").

### 5.2 Getting pixels: JPEG decode

`esp32-camera`'s `img_converters` component ships `jpg2rgb888()` — decode
the same JPEG frame already captured for SD/dataset use, then crop to the
ROI before running the threshold/moment steps. One capture, two uses — no
second camera stream or sensor reconfiguration needed. This is a known
pattern (Espressif's own face-detection examples decode a JPEG-mode
capture locally the same way).

### 5.3 Processing cadence

EAR runs on a **throttled subset of frames** (target ~7-10 Hz), not every
captured frame — full-resolution decode + threshold + moments on every
frame at the SD env's capture rate would compete too heavily with the SD
write path on core 0. 7-10 Hz is still enough temporal resolution to catch
a 100-400ms blink (matches what the offline pipeline effectively achieves).
**The exact throttle divisor needs on-hardware profiling** — this is
flagged as an open validation item, not a promised number (see §8).

### 5.4 ROI: lock at boot, drift-correct at runtime, no Haar/ML

The camera is bolted rigidly inside the helmet pointed at one eye, so
there's no need to run a trained detector to *find* the eye every session.
But a pure hardcoded-forever rectangle can't handle the helmet shifting
slightly over a real 20-40 minute ride, which the PC pipeline's Haar-based
lock + drift correction ([eye_ear.py:147-176, 263-289](../../../fatigue-helmet/python/eye_ear.py))
does handle. This design replicates both mechanisms using the same moment
primitive already needed for EAR — no Haar cascade or ML model on-device:

- **Boot lock**: run Otsu-threshold + centroid (`x̄, ȳ` from `μ00, μ10, μ01`)
  over a generously sized search window for the first ~20-30 captured
  frames, take the **median** centroid across them (robust to an
  occasional bad single-frame read), and place the ROI there. This mirrors
  the shape of the existing `calibrateMPU()` startup sequence
  ([main.cpp:728](../../../fatigue-helmet/firmware/src/main.cpp)) — a
  short "hold still" phase at boot — so it fits the firmware's existing UX
  pattern rather than introducing a new one.
- **Runtime drift check**: every few seconds (not every frame — cheap
  because it's rare), re-run the same centroid search over a small window
  around the *current* ROI. If the new centroid is within a small pixel
  threshold of the current one, blend it in with an EMA (mirrors the PC
  pipeline's `alpha=0.3`); if it jumps further than that, ignore it
  (probably mid-blink or a shadow, not real drift) — same gating idea as
  `DRIFT_MAX_PX` in `eye_ear.py`.
- **Manual override escape hatch**: a compile-time `ROI_MANUAL_X/Y/W/H`
  build-flag override (sentinel value = "use auto-lock instead"), for a
  physical build where the heuristic auto-lock repeatedly misbehaves.
  Mirrors `eye_ear.py`'s own `--roi` manual override, which exists for the
  same reason.

**Known limitation, stated honestly**: "find the darkest blob in a window"
is a cheaper heuristic than a trained Haar cascade and could occasionally
latch onto the wrong dark region (eyelash shadow, glare), especially at
the very first boot-time lock over a wider area. The median-of-N-frames
lock and the manual override both exist specifically to absorb that risk.
This needs real on-hardware validation — the offline pipeline's tuned
constants (window sizes, drift thresholds) do not automatically transfer,
since this is a different (simpler) detection method than Haar.

### 5.5 Blink detection & rolling rate

Once EAR is computed per processed frame, reuse the same shape of state
machine `eye_ear.py` already validates: an adaptive baseline (rolling
window, e.g. 75th percentile), a blink fires when EAR drops below
`baseline * 0.70`, gated by a short warm-up period and a cooldown window to
avoid double-counting one blink. Blink *events* accumulate into a 60-second
rolling rate (blinks/minute) — matching the definition already in
[fuzzy_walkthrough.md §2.2](../../../fuzzy_walkthrough.md).

### 5.6 Integration with the fuzzy engine

`g_blinkRate`'s fallback-when-nothing-live-received currently hardcodes
`13.0` ([main.cpp:1136](../../../fatigue-helmet/firmware/src/main.cpp)).
That becomes the on-device-computed rolling rate instead. The existing
`BLINK:<float>` serial parser is **kept, unchanged**, as a manual override
— it already has "prefer live input, fall back after a timeout" logic
built in ([main.cpp:1110-1137](../../../fatigue-helmet/firmware/src/main.cpp)),
which now naturally reads as "prefer a manual test injection if one is
actively arriving, otherwise trust the on-device detector" — useful for
`test_firmware_serial.py`'s existing FIS edge-case testing role.

### 5.7 Where this lives in the codebase

New header-only module, matching the existing `FuzzyFatigue.h` pattern
(heap-free, STL-free): `firmware/src/EyeBlinkEAR.h` — holds the threshold
+ moment + eigenvalue-ratio math, the blink/cooldown state struct, and the
ROI-lock/drift struct (same shape as the existing `NodDetector` /
`GyroVarBuf` ring-buffer structs already in `main.cpp`). The JPEG decode
call site stays in `cameraTask()` in `main.cpp`, since that's where the
frame buffer already lives — it calls into `EyeBlinkEAR.h` with a decoded
crop.

Compiled in whenever `STORAGE_MODE_SD` is defined — both `esp32s3cam_sd`
and the new `esp32s3cam_sd_live` get it. `esp32s3cam` (USB debug env)
is untouched; its workflow stays "record now, run `eye_ear.py` offline
later," which still works for building/validating the dataset.

## 6. PC live view (debug/verification only)

### 6.1 New PlatformIO env: `esp32s3cam_sd_live`

Added alongside (not replacing) `esp32s3cam_sd` in `platformio.ini`:
- `-DSTORAGE_MODE_SD` (same SD-writing logic, unchanged) + `-DSTREAM_TO_PC`
- `monitor_speed = 921600` (native USB CDC)
- Camera defaults to **CIF @ 15 FPS** instead of VGA @ 20 FPS for this env
  only — capturing one JPEG and both writing it to SD *and* chunking it
  over serial is extra work per frame; CIF/15fps leaves headroom. The
  untouched `esp32s3cam_sd` keeps VGA@20fps for real dataset recording.

### 6.2 `main.cpp` changes for streaming (targeted)

- `cameraTask()`: when `STREAM_TO_PC` is defined, also call the existing
  `sendJpegFrame()` (already used by USB mode) after `saveJpegToSD()`,
  guarded by `if (Serial)` so nothing is wasted when no PC is attached.
- The existing machine-readable CSV serial line currently only prints
  under `#if defined(STORAGE_MODE_USB)`
  ([main.cpp:1316](../../../fatigue-helmet/firmware/src/main.cpp)) — widen
  that guard to `STORAGE_MODE_USB || STREAM_TO_PC` so the live env also
  emits one clean CSV line per second (same columns, including the now-real
  `blink_rate`) alongside the human-readable block that already prints in
  every mode.
- SD-writing paths, the session button, buzzer/alert logic: **untouched**.
  The button still starts/stops both SD recording and the live view's "●
  REC" state together, since it's the same session.

### 6.3 New Python: `live_view_sd.py`

- Demuxes the same `0xAABBCCDD`-framed binary JPEG protocol used by
  `debug_recorder.py`/`live_preview.py`. Since this is the third script
  parsing that exact protocol, extract the shared demux loop into
  `serial_frame_protocol.py` and have all three import it — avoids a third
  copy of the same parsing logic/bugs.
- Displays the camera frame in an OpenCV window with a telemetry HUD:
  HR, pulse signal quality, IMU (pitch/nod/gyro-var), **blink rate as
  actually computed on-device now** (no longer a stub), risk % and alert
  level (color-coded SAFE/WARNING/CRITICAL), and a REC indicator.
- **Display-only** — writes no files. The SD card remains the single
  source of truth for the dataset; `unpack_session.py`, `eye_ear.py`,
  `merge_session.py` keep working unmodified on the pulled card.

## 7. What stays exactly as-is

- `esp32s3cam_sd` (real field/dataset env): SD writing logic, file
  formats, camera resolution — zero changes beyond gaining the EAR module
  described in §5, which is additive, not a rewrite of anything existing.
- `esp32s3cam` (USB debug env), `debug_recorder.py`, offline `eye_ear.py`
  pipeline — unchanged.
- `FuzzyFatigue` engine itself (rules, membership functions) — unchanged.
  Only the *source* of one of its three inputs changes.
- `test_firmware_serial.py`'s manual BLINK injection for FIS edge-case
  testing — still works, still takes precedence while actively sending.

## 8. Open validation items (not guaranteed numbers)

These need real on-hardware measurement before the constants in this spec
are treated as final — flagged explicitly rather than assumed:

1. JPEG-decode + threshold + moments cost per processed frame on the
   ESP32-S3, and the resulting safe throttle rate (target 7-10 Hz) that
   doesn't starve SD writes or IMU sampling on core 0/1.
2. Whether the "darkest blob in a window" boot-lock and drift-check
   heuristic reliably tracks the pupil/iris under real helmet lighting —
   window sizes and drift thresholds need tuning on the actual camera/eye
   setup, not assumed from the offline pipeline's Haar-based constants.
3. Retrospective accuracy check: run the on-device algorithm's logic
   (or a Python port of it) over already-recorded session frames and
   compare blink counts against `eye_ear.py`'s existing offline output,
   before trusting it to drive real alerts.

## 9. Documentation follow-ups (implementation-time, not part of this spec)

- `fuzzy_walkthrough.md` §2.2 currently states blink rate is "Computed
  from camera stream via Python" — needs updating once the on-device path
  lands, since that's no longer accurate for SD-mode operation.
- `README.md` §4 (Recording a Session) needs a new subsection for
  `esp32s3cam_sd_live` alongside the existing USB/SD mode descriptions.
