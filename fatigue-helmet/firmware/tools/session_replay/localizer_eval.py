#!/usr/bin/env python3
"""
localizer_eval.py — measure an eye-localizer version against recorded sessions.
==============================================================================
Builds session_replay.cpp once per localizer version, runs it over every
session that has unpacked frames, and reports how far each version's ROI lock
landed from the eye, plus how many blinks it went on to detect.

This exists so a new localizer version is accepted or rejected on evidence.
The history it came from is worth knowing: v2 began as an elaborate
second-difference burst detector, 41 compiled configurations of which all
failed the session it was written for. The actual defect was a lock window too
short to contain more than one blink. Only a harness like this makes that
distinction visible -- a single session, or an argument, would not have.

GROUND TRUTH
Eye position per session is derived from whole-session motion energy: over a
full recording the eye is by far the most consistently moving thing, and the
peak is checked visually once (see --save-maps). That is a fair reference
because the localizer never sees the whole session -- it only gets a short
window at the start, which is exactly what is being tested. Override any
session with --truth session_100=211,193 if the automatic pick is wrong.

USAGE
    python localizer_eval.py                       # compare v1 and v2
    python localizer_eval.py --versions 1 2 3      # once a v3 exists
    python localizer_eval.py --sessions ../../../../sessions/session_100
    python localizer_eval.py --save-maps out/      # write ground-truth images
    python localizer_eval.py --define EAR_MOTION_MIN_FRAMES=120

Requires g++ on PATH, and opencv-python + numpy for ground truth.
"""

import argparse
import glob
import math
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "session_replay.cpp")
DEFAULT_SESSIONS = os.path.normpath(os.path.join(HERE, "..", "..", "..", "..", "sessions"))


def build(version, defines, workdir):
    exe = os.path.join(workdir, "replay_v%s.exe" % version)
    cmd = ["g++", "-O2", "-std=gnu++14",
           "-DEAR_LOCALIZER_VERSION=%s" % version]
    cmd += ["-D%s" % d for d in defines]
    cmd += ["-o", exe, SRC]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        raise SystemExit("build failed for version %s" % version)
    return exe


def ground_truth(sess, save_dir=None):
    """Whole-session motion-energy peak: the eye, in full-frame coordinates."""
    import cv2
    import numpy as np

    files = sorted(glob.glob(os.path.join(sess, "frames", "*.jpg")),
                   key=lambda p: int(os.path.splitext(os.path.basename(p))[0]))
    if not files:
        return None
    step = max(1, len(files) // 400)          # cap work on long sessions
    use = files[::step]
    first = cv2.imread(use[0])
    if first is None:
        return None
    acc = np.zeros(first.shape[:2], np.float64)
    prev = cv2.cvtColor(first, cv2.COLOR_BGR2GRAY).astype(np.int16)
    for p in use[1:]:
        im = cv2.imread(p)
        if im is None:
            continue
        g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).astype(np.int16)
        acc += np.abs(g - prev)
        prev = g
    acc = cv2.GaussianBlur(acc, (0, 0), 5)
    my, mx = np.unravel_index(np.argmax(acc), acc.shape)

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        mid = cv2.imread(use[len(use) // 2]).copy()
        cv2.circle(mid, (int(mx), int(my)), 9, (0, 255, 0), 2)
        cv2.putText(mid, "%s eye %d,%d" % (os.path.basename(sess), mx, my),
                    (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imwrite(os.path.join(save_dir, os.path.basename(sess) + "_truth.png"), mid)
    return int(mx), int(my)


def run(exe, sess, out_csv, extra):
    r = subprocess.run([exe, os.path.join(sess, "frames"), out_csv] + extra,
                       capture_output=True, text=True)
    txt = r.stdout
    m = re.search(r"lock(?:ed)?[^\n]*?x=(\d+) y=(\d+) size=(\d+)", txt)
    conf = re.search(r"conf=([\d.]+)", txt)
    blinks = re.search(r"Total blink events\s*:\s*(\d+)", txt)
    if not m:
        return None
    return {"x": int(m.group(1)), "y": int(m.group(2)), "size": int(m.group(3)),
            "conf": float(conf.group(1)) if conf else float("nan"),
            "blinks": int(blinks.group(1)) if blinks else -1}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--versions", nargs="+", default=["1", "2"])
    ap.add_argument("--sessions", nargs="+", default=None)
    ap.add_argument("--define", action="append", default=[],
                    help="extra -D passed to every build, e.g. EAR_MOTION_MIN_FRAMES=120")
    ap.add_argument("--truth", action="append", default=[],
                    help="override ground truth, e.g. session_100=211,193")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="stop each run after N frames (lock is decided early)")
    ap.add_argument("--save-maps", default=None)
    args = ap.parse_args()

    sessions = args.sessions or sorted(glob.glob(os.path.join(DEFAULT_SESSIONS, "session_*")))
    sessions = [s for s in sessions if os.path.isdir(os.path.join(s, "frames"))]
    if not sessions:
        raise SystemExit("no sessions with unpacked frames; run unpack_session.py first")

    overrides = {}
    for t in args.truth:
        name, xy = t.split("=")
        overrides[name] = tuple(int(v) for v in xy.split(","))

    print("resolving ground truth ...")
    truth = {}
    for s in sessions:
        name = os.path.basename(s.rstrip("/\\"))
        truth[name] = overrides.get(name) or ground_truth(s, args.save_maps)
        print("  %-16s eye at %s" % (name, truth[name]))

    extra = ["--max-frames=%d" % args.max_frames] if args.max_frames else []
    work = tempfile.mkdtemp(prefix="localizer_eval_")
    exes = {v: build(v, args.define, work) for v in args.versions}

    print("\n%-16s %-4s %-18s %7s %8s %7s" %
          ("session", "ver", "roi centre", "conf", "dist(px)", "blinks"))
    print("-" * 68)
    totals = {v: [] for v in args.versions}
    for s in sessions:
        name = os.path.basename(s.rstrip("/\\"))
        t = truth.get(name)
        if not t:
            continue
        for v in args.versions:
            res = run(exes[v], s, os.path.join(work, "%s_v%s.csv" % (name, v)), extra)
            if not res:
                print("%-16s v%-3s no lock reported" % (name, v))
                totals[v].append((False, 999.0))
                continue
            cx = res["x"] + res["size"] // 2
            cy = res["y"] + res["size"] // 2
            dist = math.hypot(cx - t[0], cy - t[1])
            ok = dist <= res["size"] / 2.0     # is the eye inside the ROI at all
            totals[v].append((ok, dist))
            print("%-16s v%-3s (%3d,%3d)%9s %7.2f %8.1f %7d %s" %
                  (name, v, cx, cy, "", res["conf"], dist, res["blinks"],
                   "HIT" if ok else "MISS"))

    print("\n=== totals ===")
    for v in args.versions:
        r = totals[v]
        if not r:
            continue
        print("  v%s: %d/%d locks on the eye | mean error %.1f px"
              % (v, sum(a for a, _ in r), len(r), sum(d for _, d in r) / len(r)))


if __name__ == "__main__":
    main()
