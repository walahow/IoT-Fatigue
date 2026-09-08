#!/usr/bin/env python3
"""
session_review_video.py — annotated review video for a recorded helmet session.
===============================================================================
Renders camera frames alongside the sensor stream and the on-device blink
pipeline, so a session can be judged by eye instead of by reading CSVs.

Layout (1280x720):
  left   : camera frame at 2x, with the locked ROI, the pinned ROI, and a
           blink flash
  right  : live sensor readouts (HR, head movement, pitch, gyro var, nod, risk)
  bottom : glint signal with blink marks, heart rate, and a playhead

Inputs, all produced by existing tools:
  frames/                  unpack_session.py
  sensor_data.csv          firmware
  pc_replay.csv            session_replay.exe <frames> <csv>
  pc_replay_pinned.csv     session_replay.exe <frames> <csv> --roi=X,Y
The two replay CSVs are optional; lanes are omitted if absent.

Usage:
    python session_review_video.py sessions/session_100 out.mp4
"""

import cv2, numpy as np, glob, os, sys, csv as csvmod

sess    = sys.argv[1]
out_mp4 = sys.argv[2]

# Frame geometry is read from the footage, not assumed: sessions exist at both
# QVGA (320x240) and VGA (640x480), and an earlier version hardcoded QVGA and
# silently produced a broken canvas for the VGA ones.
_probe = sorted(glob.glob(os.path.join(sess, "frames", "*.jpg")),
                key=lambda p: int(os.path.splitext(os.path.basename(p))[0]))
if not _probe:
    raise SystemExit("no frames in %s -- run unpack_session.py first" % sess)
_im = cv2.imread(_probe[0])
if _im is None:
    raise SystemExit("could not read %s" % _probe[0])
FR_H, FR_W = _im.shape[:2]
SC = max(1, int(round(640.0 / FR_W)))      # QVGA -> 2x, VGA -> 1x
VID_W, VID_H = FR_W * SC, FR_H * SC
W, H       = VID_W + 640, max(720, VID_H + 240)
TL_Y       = VID_H
TL_H       = H - TL_Y

# ROI edge length in source pixels, matching EAR_ROI_SIZE in the firmware.
# Override when replaying with a different value (e.g. 96 for VGA).
ROI_PX = int(sys.argv[3]) if len(sys.argv) > 3 else 48

# Keep every Nth frame. Output fps is unchanged, so a stride > 1 plays the
# session back that many times faster -- session_023 is nine minutes at
# 20 fps, which is neither watchable nor a sane file size at 1:1. The
# timeline lanes are still drawn from the FULL data, so nothing is hidden.
STRIDE = int(sys.argv[4]) if len(sys.argv) > 4 else 1

# ── palette (BGR) ────────────────────────────────────────────────────
BG      = (24, 20, 18)
PANEL   = (34, 29, 26)
INK     = (238, 240, 244)
INK2    = (150, 158, 168)
INK3    = (96, 104, 114)
RULE    = (58, 52, 48)
SIG     = (214, 120, 42)     # blue  (locked/ESP)
GOOD    = (122, 175, 27)     # green (pinned/eye)
FLAG    = (52, 104, 235)     # orange
HOT     = (72, 72, 227)      # red
HRCOL   = (196, 123, 232)

def put(img, txt, org, scale=0.5, col=INK, thick=1, font=cv2.FONT_HERSHEY_SIMPLEX):
    cv2.putText(img, txt, org, font, scale, col, thick, cv2.LINE_AA)

# ── load frames ──────────────────────────────────────────────────────
# Title comes from the session folder, not a constant -- the first version of
# this script hardcoded "SESSION 100" and mislabelled every other session.
SESSION_LABEL = os.path.basename(os.path.normpath(sess)).replace("_", " ").upper()

files = _probe
fts = [int(os.path.splitext(os.path.basename(p))[0]) for p in files]
print("frames:", len(files))

# ── load sensor csv (headerless, 18 cols) ────────────────────────────
COLS = ["ts","hr","pulse","ax","ay","az","gx","gy","gz","mov","sq",
        "blink_rate","pitch","gvar","nod","risk","alert","imu_valid"]
sensor = []
with open(os.path.join(sess, "sensor_data.csv")) as f:
    for row in csvmod.reader(f):
        if len(row) < 18 or not row[0].lstrip("-").isdigit():
            continue
        d = dict(zip(COLS, row))
        d["ts"] = int(d["ts"])
        sensor.append(d)
sensor.sort(key=lambda d: d["ts"])
print("sensor rows:", len(sensor))

def sensor_at(ts):
    best, bd = None, 10**9
    for d in sensor:
        dd = abs(d["ts"] - ts)
        if dd < bd:
            best, bd = d, dd
    return best

# ── load replay csvs ─────────────────────────────────────────────────
def load_replay(path):
    m = {}
    if not os.path.exists(path):
        return m
    with open(path) as f:
        for row in csvmod.DictReader(f):
            ts = int(row["timestamp_ms"])
            ear = row["ear"]
            m[ts] = {
                "ear": float(ear) if ear not in ("", None) else None,
                "blink": row["is_blink_event"] == "1",
                "rate": float(row["rolling_rate_bpm"] or 0),
                "rx": int(row["roi_x"]), "ry": int(row["roi_y"]),
                "locked": row["roi_locked"] == "1",
            }
    return m

rep_lock = load_replay(os.path.join(sess, "pc_replay.csv"))
rep_pin  = load_replay(os.path.join(sess, "pc_replay_pinned.csv"))
print("replay rows: locked=%d pinned=%d" % (len(rep_lock), len(rep_pin)))

blink_ts      = [t for t, v in sorted(rep_pin.items())  if v["blink"]]   # reference
blink_ts_lock = [t for t, v in sorted(rep_lock.items()) if v["blink"]]   # what shipped
print("blinks -- locked ROI: %d, pinned reference: %d"
      % (len(blink_ts_lock), len(blink_ts)))

t0, t1 = fts[0], fts[-1]
span = max(1, t1 - t0)

# ── static timeline background ───────────────────────────────────────
tl = np.full((TL_H, W, 3), PANEL, np.uint8)
cv2.line(tl, (0, 0), (W, 0), RULE, 1)

PAD_L, PAD_R = 62, 18
PLOT_W = W - PAD_L - PAD_R

def tx(ts):
    return int(PAD_L + (ts - t0) / span * PLOT_W)

# --- lane 1: glint signal + blink marks ---
L1_Y, L1_H = 22, 96
put(tl, "GLINT SIGNAL  (ROI pinned on eye)", (PAD_L, L1_Y - 6), 0.42, INK2)
ear_vals = [(t, v["ear"]) for t, v in sorted(rep_pin.items()) if v["ear"] is not None]
if ear_vals:
    mx = max(1.0, max(v for _, v in ear_vals))
    pts = [(tx(t), int(L1_Y + L1_H - (v / mx) * L1_H)) for t, v in ear_vals]
    for i in range(1, len(pts)):
        cv2.line(tl, pts[i - 1], pts[i], SIG, 1, cv2.LINE_AA)
    put(tl, "%.0f" % mx, (PAD_L - 30, L1_Y + 8), 0.36, INK3)
    put(tl, "0", (PAD_L - 14, L1_Y + L1_H + 4), 0.36, INK3)
cv2.line(tl, (PAD_L, L1_Y + L1_H), (W - PAD_R, L1_Y + L1_H), RULE, 1)
# Two blink series so the cost of an imperfect lock is visible: what the
# shipped pipeline actually detected at its locked ROI, against what the same
# detector finds with the ROI placed on the pupil by hand.
for bt in blink_ts:
    x = tx(bt)
    cv2.line(tl, (x, L1_Y + L1_H - 14), (x, L1_Y + L1_H), GOOD, 2, cv2.LINE_AA)
    cv2.circle(tl, (x, L1_Y + L1_H - 17), 3, GOOD, -1, cv2.LINE_AA)
for bt in blink_ts_lock:
    x = tx(bt)
    cv2.line(tl, (x, L1_Y), (x, L1_Y + L1_H), FLAG, 1, cv2.LINE_AA)
    cv2.circle(tl, (x, L1_Y - 3), 4, FLAG, -1, cv2.LINE_AA)
put(tl, "detected at locked ROI", (PAD_L + 232, L1_Y - 6), 0.4, FLAG)
put(tl, "same detector, ROI pinned on pupil", (PAD_L + 402, L1_Y - 6), 0.4, GOOD)

# --- lane 2: heart rate ---
L2_Y, L2_H = 152, 62
put(tl, "HEART RATE  (bpm)", (PAD_L, L2_Y - 6), 0.42, INK2)
hrs = [(d["ts"], float(d["hr"])) for d in sensor if d["hr"] and float(d["hr"]) > 0]
if hrs:
    lo = min(v for _, v in hrs) - 3
    hi = max(v for _, v in hrs) + 3
    rng = max(1e-6, hi - lo)
    pts = [(tx(t), int(L2_Y + L2_H - (v - lo) / rng * L2_H)) for t, v in hrs]
    for i in range(1, len(pts)):
        cv2.line(tl, pts[i - 1], pts[i], HRCOL, 2, cv2.LINE_AA)
    put(tl, "%.0f" % hi, (PAD_L - 34, L2_Y + 8), 0.36, INK3)
    put(tl, "%.0f" % lo, (PAD_L - 34, L2_Y + L2_H + 4), 0.36, INK3)
cv2.line(tl, (PAD_L, L2_Y + L2_H), (W - PAD_R, L2_Y + L2_H), RULE, 1)

# time ticks
for k in range(0, int(span / 1000) + 1, 10):
    x = tx(t0 + k * 1000)
    cv2.line(tl, (x, TL_H - 16), (x, TL_H - 11), INK3, 1)
    put(tl, "%ds" % k, (x - 9, TL_H - 3), 0.36, INK3)

# ── writer ───────────────────────────────────────────────────────────
fps = len(files) / (span / 1000.0)   # source rate; stride speeds playback
vw = cv2.VideoWriter(out_mp4, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
print("fps: %.2f" % fps)

BLINK_HOLD_MS = 220

render_list = list(zip(files, fts))[::STRIDE]
print("rendering %d of %d frames (stride %d)" % (len(render_list), len(files), STRIDE))

for i, (path, ts) in enumerate(render_list):
    canvas = np.full((H, W, 3), BG, np.uint8)
    frame = cv2.imread(path)
    if frame is None:
        continue

    pin = rep_pin.get(ts)
    lok = rep_lock.get(ts)
    recent_blink = any(0 <= ts - bt <= BLINK_HOLD_MS for bt in blink_ts_lock)

    # ── video panel ──────────────────────────────────────────────
    big = cv2.resize(frame, (VID_W, VID_H), interpolation=cv2.INTER_NEAREST)

    if lok and lok["locked"]:
        x, y = lok["rx"] * SC, lok["ry"] * SC
        cv2.rectangle(big, (x, y), (x + ROI_PX * SC, y + ROI_PX * SC), HOT, 2)
        put(big, "localizer lock", (x + 4, y + ROI_PX * SC + 16), 0.44, HOT)
    if pin:
        x, y = pin["rx"] * SC, pin["ry"] * SC
        cv2.rectangle(big, (x, y), (x + ROI_PX * SC, y + ROI_PX * SC), GOOD, 2)
        put(big, "pinned on eye", (x + 4, y - 8), 0.44, GOOD)

    if recent_blink:
        cv2.rectangle(big, (0, 0), (VID_W - 1, VID_H - 1), FLAG, 6)
        put(big, "BLINK", (VID_W - 96, 30), 0.7, FLAG, 2)

    canvas[0:VID_H, 0:VID_W] = big

    # ── sensor panel ─────────────────────────────────────────────
    px0 = VID_W
    cv2.rectangle(canvas, (px0, 0), (W, VID_H), PANEL, -1)
    cv2.line(canvas, (px0, 0), (px0, VID_H), RULE, 1)

    s = sensor_at(ts)
    yy = 34
    put(canvas, SESSION_LABEL, (px0 + 22, yy), 0.62, INK, 2)
    yy += 22
    put(canvas, "t = %6.2f s     frame %d/%d%s" % ((ts - t0) / 1000.0,
            i * STRIDE + 1, len(files), "" if STRIDE == 1 else "  (%dx)" % STRIDE),
        (px0 + 22, yy), 0.44, INK2)
    yy += 26
    cv2.line(canvas, (px0 + 22, yy), (W - 22, yy), RULE, 1)
    yy += 30

    def stat(label, value, col=INK, sub=""):
        global yy
        put(canvas, label, (px0 + 22, yy), 0.40, INK3)
        put(canvas, value, (px0 + 200, yy), 0.56, col, 2)
        if sub:
            put(canvas, sub, (px0 + 330, yy), 0.40, INK2)
        yy += 32

    if s:
        sq = s["sq"] == "1"
        stat("HEART RATE", "%s bpm" % s["hr"], INK if sq else INK3,
             "contact OK" if sq else "no contact")
        stat("PULSE RAW", s["pulse"], INK2)
        stat("HEAD MOVE", "%.2f g" % float(s["mov"]), INK2)
        stat("PITCH", "%.1f deg" % float(s["pitch"]), INK2)
        stat("GYRO VAR", "%.1f" % float(s["gvar"]), INK2)
        stat("NOD SCORE", "%.2f" % float(s["nod"]), INK2)
        alert = int(s["alert"])
        acol = (GOOD, FLAG, HOT)[min(alert, 2)]
        stat("RISK", "%.0f %%" % float(s["risk"]), acol,
             ("safe", "WARNING", "CRITICAL")[min(alert, 2)])

    yy += 6
    cv2.line(canvas, (px0 + 22, yy), (W - 22, yy), RULE, 1)
    yy += 28
    put(canvas, "BLINK DETECTION", (px0 + 22, yy), 0.46, INK, 1)
    yy += 28

    lok_rate = lok["rate"] if lok else 0.0
    put(canvas, "at locked ROI", (px0 + 22, yy), 0.40, INK3)
    put(canvas, "%.0f blinks/min" % lok_rate, (px0 + 200, yy), 0.52, FLAG, 2)
    yy += 28
    pin_rate = pin["rate"] if pin else 0.0
    put(canvas, "ROI pinned on pupil", (px0 + 22, yy), 0.40, INK3)
    put(canvas, "%.0f blinks/min" % pin_rate, (px0 + 200, yy), 0.52, GOOD, 2)
    yy += 30
    nl = sum(1 for bt in blink_ts_lock if bt <= ts)
    nb = sum(1 for bt in blink_ts if bt <= ts)
    put(canvas, "blinks so far:  %d locked  /  %d pinned" % (nl, nb),
        (px0 + 22, yy), 0.44, INK2)

    # ── timeline with playhead ───────────────────────────────────
    lane = tl.copy()
    x = tx(ts)
    cv2.line(lane, (x, 10), (x, TL_H - 18), INK, 1, cv2.LINE_AA)
    cv2.circle(lane, (x, 10), 4, INK, -1, cv2.LINE_AA)
    canvas[TL_Y:H, 0:W] = lane

    vw.write(canvas)
    if i % 100 == 0:
        print("  %d/%d" % (i, len(render_list)))

vw.release()
print("wrote", out_mp4)
