#!/usr/bin/env python3
"""Train the pooled HOG blink classifier for the DEVICE, not for an idealised crop.

Why not train_blink_pooled.py: it scores held-out data with the eye-tracked (aligned) crop, which
the helmet can't reproduce (it crops at the stored/locked ROI), and it uses class weight 20, which
squeezes the scores into a narrow band just above the SVM margin. A model fitted on all the data then
sits on a cliff: threshold 1.0 gave 16 blinks/min on a 6/min session, 1.25 gave 3/min.

This script (1) also dumps features at each session's fixed stored ROI (metadata roi_x/roi_y + size/2),
(2) leave-one-session-out: train on the other sessions, test on the held-out one's fixed-ROI crops,
sweeping class weight / training mix / threshold, (3) trains the final model with the chosen settings.

Usage (needs each session's blink_labels.csv, roi_track.csv, frames/ and metadata.txt):
  python train_blink_devicelike.py --sessions sessions/session_119 sessions/session_121 sessions/session_122 \
         --header ../firmware/src/BlinkWeights.pooled_devicelike.h
"""
import argparse, csv, os, subprocess
import numpy as np, cv2
import train_blink_classifier as T

HOG_LEN = T.HOG_LEN
DT = [("ts", "<u4"), ("mean", "<f4"), ("f", "<f4", (HOG_LEN,))]


def train(X, y, cw, C=0.1):
    svm = cv2.ml.SVM_create(); svm.setType(cv2.ml.SVM_C_SVC); svm.setKernel(cv2.ml.SVM_LINEAR)
    svm.setC(C); svm.setClassWeights(np.array([1.0, float(cw)])); svm.train(X, cv2.ml.ROW_SAMPLE, y)
    rho, alpha, _ = svm.getDecisionFunction(0)
    return -(alpha[0, 0] * svm.getSupportVectors()[0]).astype(np.float32), np.float32(rho)


def dump(session, name, *flags):
    path = os.path.join(session, "blink_classifier", name); os.makedirs(os.path.dirname(path), exist_ok=True)
    subprocess.run([T.REPLAY, os.path.join(session, "frames"), os.path.join(session, "blink_classifier", "replay.csv"),
                    "--hog-dump=" + path, *flags], capture_output=True, check=True)
    return np.fromfile(path, dtype=DT)


def load(session):
    rows = list(csv.DictReader(open(os.path.join(session, "blink_labels.csv"))))
    idx = {int(r["timestamp_ms"]): int(r["frame_idx"]) for r in rows}
    st = np.full(max(idx.values()) + 1, "", dtype="<U10")
    for r in rows: st[int(r["frame_idx"])] = r["state"]
    track = os.path.join(session, "roi_track.csv")   # crop follows the eye (make_roi_track.py); --roi pins the lock to frame 0
    first = open(track).readline().split(",")
    aligned = dump(session, "aligned.bin", f"--roi={int(float(first[1]))},{int(float(first[2]))}", "--roi-track=" + track)
    meta = dict(l.strip().split("=", 1) for l in open(os.path.join(session, "metadata.txt")) if "=" in l)
    cx, cy = int(meta["roi_x"]) + int(meta["roi_size"]) // 2, int(meta["roi_y"]) + int(meta["roi_size"]) // 2
    fixed = dump(session, "devicelike.bin", f"--roi={cx},{cy}")   # crop pinned where the device's stored tap put it
    assert (aligned["ts"] == fixed["ts"]).all()
    keep = np.array([t in idx for t in aligned["ts"]]); aligned, fixed = aligned[keep], fixed[keep]
    fi = np.array([idx[t] for t in aligned["ts"]]); o = np.argsort(fi); aligned, fixed, fi = aligned[o], fixed[o], fi[o]
    s_ = st[fi]; closed = np.where(st == "closed")[0]; closed = closed[closed >= fi[0]]
    near = np.array([np.abs(closed - i).min() <= T.TOL for i in fi])
    return dict(session=session, Xa=aligned["f"], Xd=fixed["f"], fi=fi, st=st, y=(s_ == "closed").astype(np.int32),
                usable=(s_ != "unsure") & ((s_ == "closed") | ~near),
                blinks=np.split(closed, np.where(np.diff(closed) > 2)[0] + 1),
                mins=(aligned["ts"][-1] - aligned["ts"][0]) / 60000)


def events(sc, th, R=3):
    out, run = [], R          # same rule as HogBlinkDetector: count on the first closed frame, 3-frame refractory
    for i, v in enumerate(sc):
        if v <= th: run = min(R, run + 1)
        else:
            if run >= R: out.append(i)
            run = 0
    return out


def score(D, sc, th):
    tp, fp, _ = T.score_events([D["fi"][i] for i in events(sc, th)], D["st"], D["blinks"])
    n = len(D["blinks"]); p = tp / max(tp + fp, 1); r = tp / n
    return 2 * p * r / max(p + r, 1e-9), (tp + fp) / D["mins"], n / D["mins"]


def stack(Ds, mode):
    X = [D["Xa"][D["usable"]] for D in Ds]; y = [D["y"][D["usable"]] for D in Ds]
    if mode == "aligned+devicelike":
        X += [D["Xd"][D["usable"]] for D in Ds]; y += [D["y"][D["usable"]] for D in Ds]
    return np.vstack(X), np.concatenate(y)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sessions", nargs="+", required=True); ap.add_argument("--header", required=True)
    ap.add_argument("--class-weights", type=float, nargs="+", default=[1, 4, 20])
    a = ap.parse_args()
    Ds = [load(s) for s in a.sessions]
    ths = np.round(np.arange(-1.0, 2.01, 0.25), 2)
    best = None
    for cw in a.class_weights:
        for mode in ("aligned", "aligned+devicelike"):
            cur = []
            for i, te in enumerate(Ds):
                w, b = train(*stack(Ds[:i] + Ds[i + 1:], mode), cw)
                cur.append({float(t): score(te, te["Xd"] @ w + b, t) for t in ths})   # held-out session, device-like crop
            th = max(ths, key=lambda t: np.mean([c[float(t)][0] for c in cur]))
            f1 = np.mean([c[float(th)][0] for c in cur]); ratio = [round(c[float(th)][1] / c[float(th)][2], 1) for c in cur]
            print(f"class weight {cw:>4} {mode:20s}: best threshold {th:+.2f}, mean F1 {f1:.2f}, detected/true rate per session {ratio}", flush=True)
            if best is None or f1 > best[0]: best = (f1, cw, mode, float(th))
    f1, cw, mode, th = best
    w, b = train(*stack(Ds, mode), cw)
    T.write_header(a.header, w, b, th,
                   [f"Pooled {', '.join(os.path.basename(D['session']) for D in Ds)}: {mode} crops, class weight {cw:g}.",
                    f"Leave-one-session-out on device-like (fixed-ROI) crops: mean F1 {f1:.2f} at threshold {th:+.2f}."])
    print(f"final: class weight {cw:g}, {mode}, threshold {th:+.2f} -> {a.header}")


if __name__ == "__main__":
    main()
