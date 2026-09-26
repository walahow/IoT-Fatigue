# Where the helmet can go next

Written 2026-09-27 from everything learned in ~25 Claude sessions. It's ordered by what unblocks what.
The project's real goal is **detecting fatigue**. Blink detection is one input to that, and it's now "good enough to
collect data with". The biggest gap is no longer the models: it's **fatigue-labelled data** and a few sensor
weaknesses.

## Where things stand
| Part | State |
|---|---|
| Recording pipeline (arming, SD, phone preview, drift correction) | Works end to end on the bench and on real rides |
| Blink detection (on-device HOG + running template) | Held-out F1 0.69. On a device-like fixed crop the blink rate came out −24% / +14% / +16% on 119/121/122 (in-sample). Rider- and mount-specific, and not yet checked on a new ride |
| Heart rate | Unreliable: contact noise, and a floating input still yields plausible-looking BPM |
| IMU / head motion | Works. The nod detector is **unvalidated** on real nods |
| Fuzzy alerts | Plumbing fixed (blink-unknown state, alpha-cut, gating, bump-proof nods). Thresholds are **not calibrated** to any fatigue label |
| Fatigue ground truth | Essentially none: one self-report (session 104). No KSS-labelled rested vs. fatigued pairs |

---

## Now (next 1–2 weeks)
1. **Do a real ride on the new model** (pushed 2026-09-27 as `c9b0a3799`). Label it (`LABELING.md`) and compare the on-device
   blink rate (`blink_events.csv`) with the truth. That's the first real test of the fixed-crop device behaviour.
   Drift correction was only confirmed on the chip for the first 183 s.
2. **Start the fatigue dataset now. Everything below depends on it.** For every ride:
   - KSS at start and end
   - a 2-minute stationary baseline
   - time-block notes of how the ride felt
   - route, time and sleep notes
   
   Aim for matched pairs: the same route at the same time of day, once rested and once tired.
   Target from `CLAUDE.md`: ≥10 rested + ≥10 fatigued sessions of ≥20 min.
3. **Arming feedback without the phone.** One beep = ready, two beeps = not ready (eye check failed / no HR),
   on the existing buzzer (GPIO 14). Offered before, not built. Small `main.cpp` change.
4. **Save the arming HR baseline** (and the final eye ROI) to `metadata.txt`. Replays can't reproduce a ride's
   alerts exactly without it (see `TRAINING.md` §5). It's a one-line firmware change with a big payoff for tuning.
5. **Phone-preview bench test** (Task 5 in `fatigue-helmet/firmware/docs/superpowers/plans/2026-09-23-phone-arming-preview.md`):
   eye-detection fps with the page open, pulse noise with Wi-Fi on/off, how long start/stop block `loop()`, memory over rides.

## Sensors and hardware (biggest lever on data quality)
- **Eye camera optics.**
  - Refocus the OV2640's twist lens for the short eye distance, using the phone preview while adjusting.
  - Make the mount rigid: blink accuracy depends mostly on consistent eye framing.
  - Next step up: an **850/940 nm IR LED + NoIR OV2640 + IR-pass filter**, the same approach as car driver-monitoring cameras. It kills sky/glare
    reflections, works at night, and would make **glasses riders** possible (102–104 were unusable). A camera between lens and eye also helps.
- **Frame rate.**
  - Eye frames run at ~10 fps, with 17–66 gaps over 300 ms per ride, and ~80% of blinks appear in a single frame.
  - Blink *duration* and PERCLOS are among the strongest fatigue markers, and they need ≥20–30 fps on the eye.
  - Investigate the SD write stalls. Consider storing only the eye crop, or a lower resolution for the eye pipeline.
- **Heart rate.**
  - Bench-measure real PPG amplitude against the floating-input noise floor, then add a contact-quality gate
    (amplitude + beat-interval regularity) so noise is reported as "no contact", not BPM.
  - Consider moving the sensor to a better spot inside the helmet (forehead/temple/ear), or a MAX30102-class I2C sensor
    with ambient-light cancellation.
- **Log beat-to-beat intervals (IBI).** The firmware already detects beats. With IBIs you get HRV (RMSSD/SDNN),
  which separates rested from tired far better than 1 Hz BPM.
- **Higher-rate IMU logging for nods.** The CSV is ~1 row/s, so real nods can't be seen or labelled. Log pitch/gyro at ≥10 Hz
  (a separate file like `blink_events.csv`), record deliberate nods safely (seated or stationary), and validate
  `NodDetector.h` against them.
- Battery-life profiling under sustained recording, and power-loss-safe file closing.

## Fatigue detection (the actual goal; needs the dataset above)
1. **Per-rider baselines in the FIS.**
   - Replace absolute blink-rate thresholds (Low < 7/min) with a **relative drop from the rider's own baseline**,
     measured in the arming or first minutes. The literature reports fatigue as a 40–56% relative drop.
   - Do the same for HR (already baseline-relative) and pitch (neutral differs 6–12.6° by mount).
   - Edit `FuzzyFatigue.h` + `fuzzy_model.py` together and check with `sensor_sim` (`TRAINING.md` §5).
2. **Windowed features (1–2 min), not per-second.** Session 104 showed per-second values carry almost no signal
   while 2-minute medians separate states clearly (gyro_var AUC 0.08 vs. 0.42). Candidates:
   - blink rate relative to baseline, and blink duration (once fps allows)
   - HR trend and HRV
   - gyro-variance trend, pitch relative to neutral, nod events
3. **Learn from labels once enough rides exist.** Compare the FIS against simple models (logistic regression, a
   small decision tree) on windowed features, leave-one-session-out, with KSS/phase labels as targets.
   Anything this small runs on the ESP32 as plain C. TensorFlow Lite isn't needed at this size.
   Keep the FIS as the interpretable baseline.
4. Only then recalibrate the alert levels and dwell times against labelled data.

## Blink model (after more clean sessions)
- **More clean labelled sessions of the same rider on a stable mount** is the biggest lever. Every model tweak so far
  moved F1 by ≤0.06, and footage quality moved it by ~0.3.
- Re-sort and re-check session 119's labels (stored as 14 out-of-order blocks, ambiguous partial blinks).
- Align session 101 with its own template (`make_roi_track.py --ref session_101`) so it can be pooled.
- Untried and plausible:
  - a **HOG block-mask search** with strictly nested CV (the one place a search/"swarm" idea might help);
  - a **tiny MLP** (756→16→1, ~12k multiply-adds, 1–8 ms) once there are ≳1000 labelled blinks;
  - an **adaptive per-ride threshold** targeting a plausible blink-rate range (a calibration fix, not accuracy).
- A **per-rider profile** in NVS (stored eye tap + blink baseline, later the model) for multiple riders.
- Already tried and rejected: see `TRAINING.md` §6. Older approaches were also replaced for good reasons:
  glint detector (missed ~95%), EAR/Otsu moments, Haar, contour+ellipse, darkness-based drift correction.

## System and code health
- Delete the log-only glint detector and its 2 permanently failing tests, or fix them. The failures hide real regressions.
- Known technical debt:
  - a cross-core race on the EAR ROI;
  - the SD/I2C access collision (mutex planned);
  - `CLAUDE.md`'s phase history, which is partly stale;
  - many stale feature branches.
- Sync sessions over Wi-Fi to the phone or PC instead of pulling the SD card, plus a post-ride summary page on the phone.
- Watchdog and graceful recovery for long rides.

## Safety
The alerts are research data, **not a safety device**. Don't collect "fatigued" data by riding dangerously
tired on public roads. Prefer natural end-of-day rides, short routes, or controlled or stationary protocols.
