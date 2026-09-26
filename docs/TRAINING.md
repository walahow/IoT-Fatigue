# Training and tuning the models

Written 2026-09-27. Commands run from `fatigue-helmet/python/` unless stated. Labelling comes first:
see [`LABELING.md`](LABELING.md).

The helmet has two "models":

| | Blink classifier | Fuzzy fatigue model (FIS) |
|---|---|---|
| What | HOG features + linear SVM with a running open-eye template. Decides "closed eye" per frame. | Membership functions + rules turning HR, blink rate, head motion and nods into Safe / Warning / Critical |
| Learned? | **Yes**, from hand-labelled frames | **No**, hand-tuned, then checked by replaying recorded rides |
| Lives in | `firmware/src/BlinkWeights.h` (generated, never hand-edit) | `firmware/src/FuzzyFatigue.h` **and** `python/fuzzy_model.py` (keep identical) |

There is **no learned fatigue classifier yet**. It needs fatigue-labelled rides first (see `ROADMAP.md`).

---

## 1. One-time setup on a machine
```bash
pip install -r requirements.txt            # numpy, opencv-python, pandas, scipy, matplotlib, ...
cd ../firmware/tools/session_replay
g++ -O2 -std=gnu++14 -o session_replay.exe session_replay.cpp          # trainers call this; not in git
g++ -O2 -std=gnu++14 -Ihost_shim -o sensor_sim.exe sensor_sim.cpp       # FIS replay (section 5)
```
`session_replay.exe` compiles the **same `EyeBlinkEAR.h` the ESP32 uses**, so training features are
exactly what the device computes. Rebuild it whenever `EyeBlinkEAR.h` or `session_replay.cpp` changes.
g++ on this PC is WinLibs MinGW (on PATH).

Session footage isn't in git. Copy `sessions/session_XXX/` folders between machines. Each training
session needs `frames/`, `blink_labels.csv`, `roi_track.csv` and `metadata.txt`.

## 2. How the blink model works (enough to not break it)
- **Input:** a crop around the eye (stored tap / drift-corrected ROI) → resized → **756 HOG values** `f`.
- **Score:** `f·w1 + (f − B)·w2 + bias`. `B` is a running average of recent frames (α = 0.05, ≈2 s): the
  rider's current open-eye look. On the device this collapses to `f·(w1+w2) − s + bias`, with
  `s ← (1−α)s + α(f·w2)`, so it costs two dot products and one float of state. No detections for the first 15 frames.
- **Blink event:** fires on the first frame whose score crosses the threshold after ≥3 non-blink frames.
- **Labels:** only `closed` frames are positives. `unsure` frames are dropped. Frames within 2 of a closed frame
  aren't used as negatives.
- **The score that matters is leave-one-session-out, event-level, on *device-like* crops.** Train on the other
  sessions, test on the held-out one, with the crop fixed at its stored tap (what the helmet does). A detection
  within 2 frames of a labelled blink counts as a hit. In-sample or eye-tracked-crop numbers flatter the model.
  Session 101 once "scored" 0.99 that way.

## 3. Train
Current shipped model: sessions 119+121+122, **mean LOSO F1 0.693** (119: 0.52, 121: 0.80, 122: 0.76; detected/true rate 0.8 / 1.1 / 1.1), threshold −0.25.
Re-running the command below on 119+121+122 reproduces the shipped `BlinkWeights.h` exactly (checked 2026-09-27).
```bash
python train_blink_bgsub.py \
    --sessions ../../sessions/session_119 ../../sessions/session_121 ../../sessions/session_122 ../../sessions/session_XXX \
    --header ../firmware/src/BlinkWeights.pooled_bgsub_XXX.h \
    --model ../../sessions/session_XXX/bgsub_model.bin
```
- Takes ~15–25 min. It dumps HOG features for each session through `session_replay.exe`, cached under
  `sessions/<s>/blink_classifier/`.
- It prints leave-one-session-out results at three thresholds (−0.50, −0.25, 0.00):
  per-session F1, the mean, and **detected ÷ true blink rate** per session.
- **Adopt the new model only if:**
  - the mean F1 at −0.25 beats **0.693**;
  - no session drops badly;
  - detected/true stays near **1.0** (0.8–1.2);
  - F1 at −0.50 and 0.00 is close to the −0.25 value (no "cliff").
- The header's comment records the measured score. Before 2026-09-27 it was a hardcoded "~0.69", so trust the printed log over old headers.
- Options: `--alpha` (template speed, default 0.05; 0.02–0.15 tested, 0.05 was best), `--threshold` (−0.25),
  `--no-loso` (skip evaluation; use only for a final fit you've already evaluated).

Other trainers in `python/`, kept for history. Don't use them for the shipped model:
- `train_blink_devicelike.py`: single-frame, sweeps class weight. It produced the previous model (0.637).
- `train_blink_pooled.py`: aligned crops, class weight 20. This gave the **threshold cliff**: 1.0 → 16 blinks/min,
  1.25 → 3/min on a 6/min session.
- `train_blink_classifier.py`: one session plus ROI jitter. **Its default `--header` is `BlinkWeights.h`, so it
  overwrites the shipped model.** Always pass `--header`.

## 4. Ship a new model
1. **Keep a rollback:** the shipped model is also saved as `BlinkWeights.pooled_bgsub.h`. Then
   `cp ../firmware/src/BlinkWeights.pooled_bgsub_XXX.h ../firmware/src/BlinkWeights.h`.
2. **Simulate the helmet on a labelled session** (fixed crop at the stored tap, like the device). Run from the repo root:
   `python fatigue-helmet/python/make_sim_video.py <any_scratch_dir> 122` →
   `sessions/session_122/sim_devicelike_model.mp4` (overwrites). Watch it. It prints the model rate vs.
   hand-label rate on screen. (Fixed 2026-09-27: before that it couldn't read the running-template model format.)
3. **Tests and build:**
   `cd ../firmware && pio test -e native` should give 68/71, with the 2 known `test_glint_*` failures. Then
   `pio run -e esp32s3cam_sd`, which needs the `HELMET_AP_PASS` env var.
4. **Optional check on the real chip, no ride needed:** flash `esp32s3cam_frame_inject`, then
   `python frame_inject_replay.py --frames-dir ../../sessions/session_122/frames --out inj122 --hog-out inj122_hog.csv`.
   Its blinks should match `session_replay.exe ... --hog-csv` on the same frames. They matched exactly on session 116.
5. **Flash and ride:** `pio run -e esp32s3cam_sd -t upload --upload-port COM3`. Re-tap the eye on the phone page first.
   After the ride, label it and compare `blink_events.csv` (source `hog`) with the truth.
6. Commit the header, and name the sessions and score in the commit message.

## 5. Tuning the fuzzy fatigue model
- Edit **both** `firmware/src/FuzzyFatigue.h` and `python/fuzzy_model.py`, and update `fuzzy_walkthrough.md`.
  Constants that must match: membership-function edges, rules, `FIS_MIN_FIRING` / `MIN_FIRING` (0.25).
- **Correctness check after any edit:** replay a ride recorded with the *current* FIS code without new
  inputs, and the tool must reproduce the recorded alerts. It prints `vs recorded CSV: alert differs on N rows`.
  Verified 2026-09-27: session 122 → 3 of 1035 rows differ.
  ```bash
  ../firmware/tools/session_replay/sensor_sim.exe ../../sessions/session_122/sensor_data.csv out.csv
  ```
  - The device forms its HR baseline during arming, before the CSV starts, and doesn't save it. The tool
    estimates it from the CSV, and a wrong estimate shows up as many differing rows. Pass `--baseline=<BPM>`
    (121: 64 rows differ with the estimate, 8 with `--baseline=88`).
  - Only sessions recorded after the 2026-09-22 FIS changes can match. Older rides ran older FIS code.
  - **Rebuild `sensor_sim.exe` after pulling.** A build from before the 20-column CSV silently reads **0 rows**,
    which is what the copy here did until 2026-09-27.
- **Evaluate a change:** replay rides with `--blinks=<session_replay --hog-csv output>` and compare the gated
  Safe/Warning/Critical shares between rested and fatigued rides. A change is good only if alerts **track the
  labels** (KSS, felt-phase notes), not merely if it produces fewer alerts. Native tests: `test_fis_hr`, `test_nod_alert`.
- Known facts:
  - This rider's normal riding blink rate (4.6–7.5/min) sits inside the literature's "fatigued" band, so absolute blink
    thresholds can't fit them and a per-rider baseline is needed.
  - The nod detector is unvalidated on real nods.
  - 2-minute medians separate states much better than per-second values. In session 104, gyro_var had AUC 0.08 windowed vs. 0.42 per-row.

## 6. Already tried: don't repeat without a new reason
All tested 2026-09-25/26, leave-one-session-out on 119/121/122, device-like crops:

| Idea | Result vs. single-frame 0.637 |
|---|---|
| Position jitter ±8/12/16 px, scale/blur/contrast augmentation | 0.58–0.62 (worse, rate calibration drifts) |
| Temporal windows (3, 5, past-only frames) | 0.59–0.62 (worse; ~80% of blinks are one frame at 10 fps) |
| LDA, logistic regression instead of SVM | 0.62–0.64 single-frame; 0.66–0.68 with the template (SVM 0.69) |
| SVM C sweep 0.01–1.0 | 0.1 is at the peak |
| Class weight 20 | score cliff (see above) |
| Pooling unaligned or heterogeneous sessions (101 + all of 116) | collapse (recall 42% → 16%) |
| Counting brief partial closures as "closed" (119) | harder, not better |
| **Running open-eye template** | **0.693, adopted** |

Ideas that haven't been tried are in [`ROADMAP.md`](ROADMAP.md).
