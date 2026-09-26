"""
sweep_sheets.py -- contact sheets for labelling blinks by eye, and a writer for the result.

How sessions 121/122 were hand-labelled (no model in the loop): every frame is cropped around the
eye, tiled into numbered grids, and each sheet is looked at in order. Closed frames are written down
by frame_idx, suspicious ones re-checked at a larger zoom, then everything becomes blink_labels.csv.

  sheets  python sweep_sheets.py sheets --session ../../sessions/session_123
          -> <session>/sweep/sheet_000.png ... (default 16x12 tiles, 64x60 px each, frame_idx on every tile)
          Re-check a stretch big: --start 5040 --end 5060 --cols 7 --scale 3 --out-dir <session>/sweep/check
  write   python sweep_sheets.py write --session ../../sessions/session_123 \
              --closed 1203,1507-1508 --squint 6528-6720 --unsure 0-199
          -> <session>/blink_labels.csv (timestamp_ms,frame_idx,state; every unlisted frame = open)

frame_idx = position of the frame in frames/ sorted by integer timestamp (same as label_ground_truth.py
and the trainers). Eye centre per frame comes from roi_track.csv if present, else the centre of the
metadata box (roi_x/roi_y + roi_size/2).
"""
import argparse, bisect, csv, os, sys
import cv2
import numpy as np


def load_frames(session):
    d = os.path.join(session, "frames")
    if not os.path.isdir(d):
        sys.exit(f"[ERROR] no frames/ in {session} -- run unpack_session.py first")
    return d, sorted((int(os.path.splitext(f)[0]), f) for f in os.listdir(d) if f.endswith(".jpg"))


def eye_centre_fn(session):
    track = os.path.join(session, "roi_track.csv")
    if os.path.exists(track):
        rows = sorted((int(float(r[0])), float(r[1]), float(r[2]))
                      for r in csv.reader(open(track)) if r and r[0][0].isdigit())
        ts = [r[0] for r in rows]
        return lambda t: rows[max(0, bisect.bisect_right(ts, t) - 1)][1:]
    meta = dict(l.strip().split("=", 1) for l in open(os.path.join(session, "metadata.txt")) if "=" in l)
    half = int(meta.get("roi_size", 48)) / 2                 # roi_x/roi_y are the box's top-left corner
    x, y = float(meta["roi_x"]) + half, float(meta["roi_y"]) + half
    return lambda t: (x, y)


def parse_ranges(spec):
    """'5,7-9' -> {5,7,8,9}"""
    out = set()
    for part in filter(None, (spec or "").split(",")):
        a, _, b = part.partition("-")
        out.update(range(int(a), int(b or a) + 1))
    return out


def cmd_sheets(a):
    d, frames = load_frames(a.session)
    centre = eye_centre_fn(a.session)
    tw, th = 64 * a.scale, 60 * a.scale
    lo, hi = a.start or 0, min(a.end if a.end is not None else len(frames) - 1, len(frames) - 1)
    out = a.out_dir or os.path.join(a.session, "sweep")
    os.makedirs(out, exist_ok=True)
    per = a.cols * a.rows
    for s, first in enumerate(range(lo, hi + 1, per)):
        idx = range(first, min(first + per, hi + 1))
        sheet = np.zeros((-(-len(idx) // a.cols) * th, a.cols * tw, 3), np.uint8)
        for k, i in enumerate(idx):
            ts, name = frames[i]
            img = cv2.imread(os.path.join(d, name))
            cx, cy = centre(ts)
            x0, y0 = int(cx - a.crop_w / 2), int(cy - a.crop_h / 2)
            img = cv2.copyMakeBorder(img, a.crop_h, a.crop_h, a.crop_w, a.crop_w, cv2.BORDER_CONSTANT)
            crop = img[y0 + a.crop_h:y0 + 2 * a.crop_h, x0 + a.crop_w:x0 + 2 * a.crop_w]
            tile = cv2.resize(crop, (tw, th), interpolation=cv2.INTER_AREA)
            cv2.putText(tile, str(i), (2, 10 * a.scale), cv2.FONT_HERSHEY_PLAIN, 0.7 * a.scale, (0, 255, 255), 1)
            r, c = divmod(k, a.cols)
            sheet[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = tile
        cv2.imwrite(os.path.join(out, f"sheet_{s:03d}.png"), sheet)
    print(f"{s + 1} sheets, frames {lo}-{hi} -> {out}")


def cmd_write(a):
    _, frames = load_frames(a.session)
    state = ["open"] * len(frames)
    for name in ("squint", "closed", "unsure"):          # later wins: unsure overrides everything
        for i in parse_ranges(getattr(a, name)):
            state[i] = name
    path = os.path.join(a.session, "blink_labels.csv")
    if os.path.exists(path) and not a.force:
        sys.exit(f"[ERROR] {path} exists -- pass --force to overwrite (move the old one aside first)")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp_ms", "frame_idx", "state"])
        w.writerows((ts, i, st) for i, ((ts, _), st) in enumerate(zip(frames, state)))
    closed = sorted(parse_ranges(a.closed))
    events = sum(1 for j, i in enumerate(closed) if j == 0 or i - closed[j - 1] > 2)
    print(f"{path}: {len(frames)} frames, {events} blinks ({len(closed)} closed frames), "
          f"{state.count('squint')} squint, {state.count('unsure')} unsure")


if __name__ == "__main__":
    assert parse_ranges("5,7-9") == {5, 7, 8, 9} and parse_ranges("") == set()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("sheets")
    s.add_argument("--session", required=True)
    s.add_argument("--start", type=int); s.add_argument("--end", type=int)
    s.add_argument("--cols", type=int, default=16); s.add_argument("--rows", type=int, default=12)
    s.add_argument("--scale", type=int, default=1, help="tile zoom (use 3 with --cols 7 to re-check)")
    s.add_argument("--crop-w", type=int, default=128); s.add_argument("--crop-h", type=int, default=120)
    s.add_argument("--out-dir")
    w = sub.add_parser("write")
    w.add_argument("--session", required=True)
    w.add_argument("--closed", default=""); w.add_argument("--squint", default=""); w.add_argument("--unsure", default="")
    w.add_argument("--force", action="store_true")
    a = p.parse_args()
    cmd_sheets(a) if a.cmd == "sheets" else cmd_write(a)
