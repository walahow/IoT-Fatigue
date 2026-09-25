#!/usr/bin/env python3
"""Measure where the eye sits in each frame window of a session, for aligned HOG training crops.

The firmware's ROI (motion lock / stored tap) and the replay tool's own lock put the crop on different
parts of the eye in different sessions, and the eye drifts ~15 px within a session. The HOG classifier is
position-sensitive, so mixed framings wreck cross-session training. This tracks one anatomical point
(the eye's centre, defined by a reference session's median image) through every session by matching the
edge image of a per-window median frame against that reference template, and writes
<session>/roi_track.csv ("timestamp_ms,cx,cy") for `session_replay --roi-track=`.

Usage: python make_roi_track.py --ref sessions/session_121 --ref-centre 222,125 --session sessions/session_119 [...]
"""
import argparse, os
import cv2, numpy as np

MIN_CORR = 0.55   # below this the match is not the eye

def frames(session):
    d = os.path.join(session, "frames")
    return d, sorted(os.listdir(d), key=lambda p: int(p[:-4]))

def median_gray(d, files, idx):
    return np.median(np.stack([cv2.cvtColor(cv2.imread(f"{d}/{files[i]}"), cv2.COLOR_BGR2GRAY) for i in idx]), axis=0).astype(np.uint8)

def edges(g):
    g = cv2.GaussianBlur(g, (5, 5), 0)
    return cv2.magnitude(cv2.Sobel(g, cv2.CV_32F, 1, 0), cv2.Sobel(g, cv2.CV_32F, 0, 1))

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ref", required=True); ap.add_argument("--ref-centre", required=True)
    ap.add_argument("--session", nargs="+", required=True)
    ap.add_argument("--window", type=int, default=250, help="frames per median window")
    ap.add_argument("--search", type=int, default=40, help="max +-px from the reference centre")
    a = ap.parse_args()
    rcx, rcy = map(int, a.ref_centre.split(","))
    d, f = frames(a.ref); n = len(f)
    T = edges(median_gray(d, f, np.linspace(int(n * .1), int(n * .9) - 1, 120).astype(int)))[rcy - 55:rcy + 55, rcx - 30:rcx + 30]
    for s in a.session:
        d, f = frames(s); n = len(f)
        rows = []
        for lo in range(0, n, a.window):
            hi = min(n, lo + a.window)
            if hi - lo < a.window // 2: break
            E = edges(median_gray(d, f, np.linspace(lo, hi - 1, min(60, hi - lo)).astype(int)))
            r = cv2.matchTemplate(E, T.astype(np.float32), cv2.TM_CCOEFF_NORMED)
            # restrict to +-search px around the reference centre (top-left of the template = centre - (30,55))
            mask = np.full_like(r, -1); y0, x0 = max(0, rcy - 55 - a.search), max(0, rcx - 30 - a.search)
            mask[y0:rcy - 55 + a.search + 1, x0:rcx - 30 + a.search + 1] = r[y0:rcy - 55 + a.search + 1, x0:rcx - 30 + a.search + 1]
            _, mv, _, ml = cv2.minMaxLoc(mask)
            rows.append([int(f[(lo + hi - 1) // 2][:-4]), ml[0] + 30, ml[1] + 55, mv])
        rows = np.array(rows, dtype=float)
        # A window where the eye is hidden/blinking-dominated/dark matches badly and snaps to a wrong spot: drop it
        # (session_replay interpolates across the gap) rather than trust it. Then a 5-window median removes any
        # remaining single-window outlier without flattening the real ~15 px drift.
        weak = rows[:, 3] < MIN_CORR
        rows = rows[~weak]
        for c in (1, 2):
            p = np.pad(rows[:, c], 2, mode="edge"); rows[:, c] = np.median(np.stack([p[i:len(rows) + i] for i in range(5)]), axis=0)
        np.savetxt(os.path.join(s, "roi_track.csv"), rows[:, :3], fmt="%d,%.1f,%.1f")
        print(f"{s}: dropped {int(weak.sum())} weak windows")
        print(f"{s}: {len(rows)} windows, cx {rows[:,1].min():.0f}-{rows[:,1].max():.0f}, cy {rows[:,2].min():.0f}-{rows[:,2].max():.0f}, "
              f"match corr min/med {rows[:,3].min():.2f}/{np.median(rows[:,3]):.2f}")

if __name__ == "__main__":
    main()
