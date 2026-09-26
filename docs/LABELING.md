# Labelling a new session

Written 2026-09-26 from how sessions 101, 116, 119, 121 and 122 were labelled, including the mistakes.
Every command runs from `fatigue-helmet/python/`. `XXX` is the new session number.

A session gets two kinds of label:

| Label | What | Needed for | Effort |
|---|---|---|---|
| **A. Fatigue** (per session) | KSS 1–9 and how the ride felt over time | **every** ride: this is the dataset's purpose | 2 minutes |
| **B. Blinks** (per frame) | open / closed / squint / unsure for every video frame | only sessions used to train or evaluate the blink classifier | hours |

---

## 0. During the ride (without this, label A is weak)
- Before starting: note the **wall-clock start time** and your **KSS** (scale below). Then sit still for
  **2 minutes** (engine off, looking ahead). This gives a resting HR, pitch and stillness level to compare across days.
- Right after the ride, while it's fresh: **KSS again**, plus rough time blocks of how you felt,
  e.g. `0–10 min Normal / 10–20 min menguap, mata berat / 20+ min lelah`. This is the same idea as
  `sessions/session_104_ground_truth.xlsx`.
- Also note route, time of day, hours slept, and caffeine. Rides are only comparable when these match.
  Ideally record one rested ride and one tired ride on the same route at the same time of day.

KSS: 1 extremely alert · 2 very alert · 3 alert · 4 fairly alert · 5 neither · 6 some signs of sleepiness ·
7 sleepy, no effort to stay awake · 8 sleepy, some effort · 9 extremely sleepy, fighting sleep.

## 1. Copy and check the session
1. Wait for `Session closed — safe to remove SD card`, then copy the card's session folder to `sessions/session_XXX`.
2. Open `metadata.txt` and check: `armed_imu=1`, `armed_hr=1` (0 = pulse timed out), `armed_eye=1`,
   `eye_check=pass`, `roi_source`, `roi_x/roi_y/roi_size`. Check that `blink_events.csv` exists.
3. Run a quick health check from the CSV alone:
   ```bash
   python session_summary.py ../../sessions/session_XXX
   ```
   A flat blink channel or near-zero pulse contact means the session is still usable for label A but may be useless for B.

## 2. Label A: fatigue (always)
```bash
python label_session.py --session ../../sessions/session_XXX --kss 6 --condition fatigued   # or rested
```
- Use the **end-of-ride** KSS. The script writes `sensor_data_labeled.csv` (a header plus a `label` column) and
  records the label in `metadata.txt`. The raw `sensor_data.csv` is never modified.
- Write the start KSS, time blocks and conditions from step 0 into `sessions/session_XXX/notes.txt`.
  `label_session.py` holds only one number.
- **Never** use the recorded `alert_level` / `alert_gated` columns as a fatigue label. They are the
  system's own guess, and in 101–104 they were bug artifacts.

## 3. Label B: blinks (only for training/eval sessions)

### Is this session worth it?
Skip B if the rider wore **glasses** (102–104: lens reflections, blinks can't be seen), if the eye leaves
the frame for long stretches, or if most of it is too dark. Clean and stable beats long.

### 3a. Unpack frames and track the eye
```bash
python unpack_session.py --session ../../sessions/session_XXX
python make_roi_track.py --ref ../../sessions/session_121 --ref-centre 222,125 --session ../../sessions/session_XXX
```
- `unpack_session.py` writes `frames/<timestamp_ms>.jpg`. **`frame_idx` = the frame's position in
  `frames/` sorted by integer timestamp.** Every label file and trainer uses that index.
- `make_roi_track.py` writes `roi_track.csv`, which records where the eye is through the ride. The trainers need it:
  HOG crops must be aligned to the eye, or cross-session training falls apart.
  If it drops many weak windows or the cx/cy ranges look wild, the camera view differs too much from
  121's (like session 101). In that case use the new session as its own reference: `--ref ../../sessions/session_XXX
  --ref-centre <x,y of the eye centre in a typical frame>`.

### 3b. The labelling rules (the convention the current model was trained on)
| State | Mark it when | Training treats it as |
|---|---|---|
| `closed` | The lids are shut: the white of the eye and the dark iris are gone, and only lid skin or lashes show. Also mark a lid that is obviously shutting. At ~10 fps most blinks are **1 frame**, some 2, a few longer. | blink |
| `squint` | The lid stays narrowed for many frames (eye still visible as a slit). **Not a blink.** | not a blink |
| `unsure` | Before the eye lock (~first 200 frames), dark or exposure dropouts, hand/hair/glare over the eye, eye outside the crop, closures longer than ~0.5 s (rubbing eyes, looking down), stretches too thin to judge | **excluded** from training and scoring |
| `open` | everything else | not a blink |

- **Look at every frame, with no model in the loop.** On 121/122, labelling only the frames a model had
  flagged found about **half** the blinks (60 → 130). Labels shaped by a model teach the next model the same blind spots.
- When unsure between a faint partial blink and nothing, use `unsure`, not `closed`. A wrong `closed` label hurts more
  than a missing one. (Session 119 was once relabelled with every brief lid narrowing marked closed. That
  made it *harder* to train, because single frames of partial blinks and squints look alike.)
- Label the **whole** session. `train_blink_classifier.py` can handle windowed labels, but the pooled
  trainers compute per-minute rates over the full ride span.
- Sanity check on this rider: true riding rate is **~4.6–6.6 blinks/min**. Much lower usually means blinks
  were missed. Above ~15/min, check for squints or glare marked as closed.

### 3c. Doing it, option 1: you, with the GUI (most reliable)
```bash
python label_ground_truth.py --session ../../sessions/session_XXX --mode blink \
    --roi <roi_x>,<roi_y>,<roi_size> --out ../../sessions/session_XXX/blink_labels.csv
```
`--roi` values come from `metadata.txt`. Keys: `o` open, `c` closed, `s` squint, `u` unsure,
`b` back, `z` undo, `q` save and quit. It resumes where you stopped, so you can do a 20-min ride in
chunks (`--start-ms` / `--end-ms` limit a sitting to a time window).

### 3d. Doing it, option 2: an AI agent sweeps contact sheets (how Claude labelled 121/122)
```bash
python sweep_sheets.py sheets --session ../../sessions/session_XXX
```
This writes `sessions/session_XXX/sweep/sheet_000.png …`: 192 eye tiles per sheet (16×12), each stamped
with its `frame_idx`, about 60 sheets for a 20-min ride. Then:
1. Look at **every sheet in order.** Keep a running list in `sweep/notes.md` of closed frame indices,
   squint stretches and unsure stretches, so the work can resume if interrupted.
2. **Re-check every closed frame and every doubtful spot zoomed in** before recording it. Dense grids cause
   column miscounts, which happened on session 116.
   ```bash
   python sweep_sheets.py sheets --session ../../sessions/session_XXX --start 5100 --end 5130 \
       --cols 7 --scale 3 --out-dir ../../sessions/session_XXX/sweep/check
   ```
3. Write the file. Every frame not listed becomes `open`, and the script refuses to overwrite without `--force`:
   ```bash
   python sweep_sheets.py write --session ../../sessions/session_XXX \
       --closed 5113,5127,6004-6005 --squint 6528-6720 --unsure 0-199,7872-8448
   ```
   It prints blinks, closed frames, squint and unsure counts. Check the rate against the sanity range above.
4. Spot-check: render zoomed strips around ~10 random `closed` frames and a few long `open` stretches.
5. Tell the user these labels were made by an AI on small tiles. Claude's 119 labels had errors of exactly this kind.

Verified: rebuilding session 121's labels through `sweep_sheets.py write` reproduces the hand-made
file byte for byte.

### 3e. Optional: eye-position ground truth
Only needed to benchmark drift correction:
`python label_ground_truth.py --session ../../sessions/session_XXX --mode eye --step 20 --out ../../sessions/session_XXX/eye_gt.csv`

## 4. Commit the labels (not the footage)
`sessions/` is gitignored because the frames are hundreds of MB. The small label files are force-added so
training can be reproduced on another machine, the same way as for 119/121/122:
```bash
git add -f sessions/session_XXX/blink_labels.csv sessions/session_XXX/roi_track.csv
```
Also consider force-adding `metadata.txt` and `notes.txt`, since the trainers read `roi_x/roi_y/roi_size` from metadata.

## 5. Retrain, only once the labels are done
The shipped model (`firmware/src/BlinkWeights.h`, identical to `BlinkWeights.pooled_bgsub.h`) is the
running-open-eye-template model trained on 119+121+122, with leave-one-session-out mean **F1 0.693**. To test adding the new session:
```bash
python train_blink_bgsub.py \
    --sessions ../../sessions/session_119 ../../sessions/session_121 ../../sessions/session_122 ../../sessions/session_XXX \
    --header ../firmware/src/BlinkWeights.pooled_bgsub_XXX.h --model ../../sessions/session_XXX/bgsub_model.bin
```
- Each session needs `frames/`, `blink_labels.csv`, `roi_track.csv` and `metadata.txt`.
- **Compare the printed leave-one-session-out F1 per session to the current one.** Only if it's better, copy the new header over
  `BlinkWeights.h`, then build, flash, and confirm the on-chip blink count on a real ride (or via
  `frame_inject_replay.py`).
- Don't pool glasses sessions, dark sessions, or sessions without an aligned `roi_track.csv`. Naively
  pooling session 116 made the model collapse (recall 42% → 16%).
- Changing the camera mount changes the crop. Check first how the current model does on the new session
  (`session_replay.exe <session>/frames rep.csv --hog-model=... --hog-csv=hog.csv`, see `handout.md`)
  before deciding whether it needs retraining.
