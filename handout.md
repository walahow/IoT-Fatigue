# Blink classifier — handout (2026-09-23)

For continuing this work from another device: what changed, what it found, and
the exact steps to record → label → retrain from here.

## Branch

Everything below is on `main` now (fast-forwarded from `blink-jitter-116-labels`,
which was branched off `phone-arming-preview` since that's where the HOG
classifier code lived before this merge). Just pull main on the new device:

```bash
git pull origin main
```

## Verification status — read this before trusting any of it

What I actually tested this session, with before/after numbers on real
footage: the blink classifier changes only (`session_replay.cpp`'s jitter
flag, `train_blink_classifier.py`, the regenerated `BlinkWeights.h`). Those
I'd stand behind.

What I did **not** verify, and neither did anyone else as far as the repo
shows:

- **The actual embedded firmware build.** Every host-side test I ran
  compiles `EyeBlinkEAR.h` in isolation with plain `g++` — that is not the
  same as building `main.cpp` against the real Arduino/ESP32 framework.
  I never ran `pio run` for `esp32s3cam` or `esp32s3cam_sd` (no PlatformIO
  in the environment I had). `PhonePreview.h`, `phone_page.h`, the Wi-Fi/
  HTTP server code — none of that got compiled by me. **Run `pio run` for
  both environments before flashing anything.**
- **The phone-arming-preview hardware bench test.** It was already pending
  before this merge — frame rate with the arming page open, pulse-sensor
  noise with Wi-Fi on/off, how long start/stop blocks the main loop, memory
  over a full ride. Merging it into `main` didn't test any of that; it just
  moved the code to where it'll get tested. Treat the phone preview as
  unbenched until someone actually runs that checklist (it's in
  `fatigue-helmet/firmware/docs/superpowers/plans/2026-09-23-phone-arming-preview.md`).
- **Known pre-existing test failures.** 2 tests in `test_eye_blink_ear` were
  already failing before today, not caused by this work but also not fixed.
  Run `pio test -e sensor_test` (or the equivalent native env) to see current
  status before assuming a clean baseline.
- **`label_ground_truth.py`.** Committed because it was sitting there
  uncommitted and matches the labelling schema already in use — I read it
  fully but never ran it myself this session. Should work; hasn't been
  exercised by me.

## What's in this branch

- `fatigue-helmet/firmware/tools/session_replay/session_replay.cpp`: new
  `--hog-jitter=N,J` flag. Dumps N extra HOG-feature copies per labelled
  training frame, each from the ROI shifted by a random ±J px (deterministic
  seed, reproducible). Training-time only — scoring/eval is untouched.
- `fatigue-helmet/python/train_blink_classifier.py`: `--jitter-n`/`--jitter-px`
  (default 4, 16) thread through to that flag.
- `fatigue-helmet/firmware/src/BlinkWeights.h`: regenerated on session_101
  with jitter on. This is what's currently shipped.
- `fatigue-helmet/python/label_ground_truth.py`: interactive hand-labelling
  tool (click the eye / step-through blink states), was sitting uncommitted
  from an earlier session — committed here. Use this to label new sessions
  instead of the throwaway scratchpad scripts used to build 116's labels.

**Not in git:** `sessions/` is gitignored (raw frames/video are too big to
version). Hand labels (`blink_labels.csv`) live inside each session folder
and don't travel with `git pull` — move session folders between machines by
copying the SD card's contents / the `sessions/<name>/` folder directly.

## What we found (full detail: session-116-blink-eval memory, this machine only —
## won't be on a different device, so the essentials are here)

- The shipped classifier is very position-brittle: an 8px ROI shift alone
  swung recall from 9/10 to 4/10 blinks. Jitter augmentation is the fix for
  that specifically, and it measurably works: on a full unseen 19-minute
  ride (session_116, hand-labelled all 12665 frames, never in training),
  recall 35%→41%, false alarms 7.3→3.5/min, vs. the pre-jitter model.
- Naively pooling session_116 into training (with or without retuning the
  class-weight ratio) made it *worse* — recall down to 16%. Two sessions'
  HOG feature distributions are different enough that one linear model
  doesn't fit both; this isn't a quick imbalance fix. Don't retry it without
  a real plan (curating/balancing what goes in, or a different model).
- Drift correction (the moving ROI box) only successfully re-centers 6 times
  in a 19-minute ride — its confidence gate (2.0) rejects 57/63 cycles. We
  checked whether loosening that gate would help: **no** — of the rejected
  cycles, roughly as many would have jumped to a *wrong* spot (20/57) as
  would have been a real fix (13/57). Don't lower `EAR_MOTION_MIN_CONF`
  blindly. Disabling drift correction entirely is worse, not better (tested:
  25% recall / 8.3 FA-min without it, vs 41% / 3.5 with it, imperfect as it
  is) — leave it on.
- Even with the box correctly on a clearly-shut eye, the model sometimes
  just doesn't fire (confirmed by inspection, not just the aggregate
  number) — a genuine cross-session appearance gap (lighting/exposure/skin
  rendering), not a framing artifact. This is the actual ceiling right now.
- Camera physically reseated since this analysis (better rigidity, and the
  OV2640's lens focus may need manual adjustment for the short eye-distance
  — it's a twist-focus lens, not fixed; use the phone-arming-preview live
  view at 192.168.4.1 to check while adjusting). This is a new, untested
  mount position — expect the current model to need re-evaluation on it.

## Workflow for a new session (do this next)

1. **Record.** `esp32s3cam_sd` build, GPIO 21 to arm/start, 15-20 min is
   plenty (session_101's whole training set was 9.4 min / 70 blinks).
   Firmware/Python setup details are in `fatigue-helmet/README.md`.
2. **Unpack.** `python unpack_session.py <session dir>` to get `frames/`.
3. **Test the current model cold, before labelling anything.** Build the
   replay tool once (`g++ -O2 -std=gnu++14 -o session_replay.exe
   session_replay.cpp` inside `firmware/tools/session_replay/`, or add
   `-std=c++17` — either works, `-std=gnu++14` is what README.md documents),
   then:
   ```bash
   session_replay.exe <new_session>/frames rep.csv \
     --hog-model=sessions/session_101/blink_model.bin --hog-csv=hog.csv
   ```
   Eyeball `hog.csv`'s `blink` column against the footage. This alone tells
   you whether the physical fixes closed the gap before spending time
   labelling.
4. **Label.** `python label_ground_truth.py --session <new_session> --mode eye ...`
   then `--mode blink ...` (see its docstring for exact flags) — writes
   `blink_labels.csv` in the same schema session_101 and session_116 use.
5. **Retrain**, same recipe, jitter on by default:
   ```bash
   python train_blink_classifier.py --session <new_session>
   ```
   This overwrites `BlinkWeights.h` and `<new_session>/blink_model.bin`. If
   the cold-test in step 3 looked reasonable, consider whether this new
   session should replace or join 101 as the training set — given the
   pooling result above, don't just concatenate blindly; re-run the same
   held-out-fold check `train_blink_classifier.py` already does and look at
   the numbers before trusting it.
6. **Verify** against real firmware, not just the PC replay: flash
   `esp32s3cam_sd` (or use `FRAME_INJECT_MODE` / `frame_inject_replay.py` to
   feed the same session to the real chip) and confirm the on-chip blink
   count roughly matches.

## Open question this hands off

Should the reseated camera get its own trained model, or does it turn out
close enough to 101's original mount that the existing one holds up? Step 3
above answers that directly — do it before deciding anything else.
