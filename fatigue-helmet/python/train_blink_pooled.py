#!/usr/bin/env python3
"""Pool several hand-labelled sessions into one HOG blink classifier, 5-fold CV.

Fold k = the k-th contiguous fifth (by frame index) of EVERY session; train on the other
fifths (5-frame gap), score the held-out fifth of each session through session_replay.
Writes the pooled header to --header (NOT BlinkWeights.h -- copy it over yourself if it wins).

Usage: python train_blink_pooled.py --sessions sessions/session_101 sessions/session_119 --header out.h
"""
import argparse, csv, os
import numpy as np
from train_blink_classifier import (HOG_LEN, FOLDS, TOL, replay, train_svm, write_model,
                                    write_header, score_events)


def load(s, jitter_n, jitter_px):
    work = os.path.join(s, "blink_classifier"); os.makedirs(work, exist_ok=True)
    rows = list(csv.DictReader(open(os.path.join(s, "blink_labels.csv"))))
    idx_of = {int(r["timestamp_ms"]): int(r["frame_idx"]) for r in rows}
    state = np.full(max(idx_of.values()) + 1, "", dtype="<U10")
    for r in rows: state[int(r["frame_idx"])] = r["state"]
    dump = os.path.join(work, "hog_features.bin")
    replay(s, work, "--hog-dump=" + dump, *([f"--hog-jitter={jitter_n},{jitter_px}"] if jitter_n else []))
    rec = np.fromfile(dump, dtype=[("ts", "<u4"), ("mean", "<f4"), ("f", "<f4", (HOG_LEN,))])
    rec = rec[np.array([t in idx_of for t in rec["ts"]])]
    fidx = np.array([idx_of[t] for t in rec["ts"]]); st = state[fidx]
    closed = np.where(state == "closed")[0]; closed = closed[closed >= fidx[0]]
    blinks = np.split(closed, np.where(np.diff(closed) > 2)[0] + 1)
    near = np.array([np.abs(closed - i).min() <= TOL for i in fidx])
    usable = (st != "unsure") & ((st == "closed") | ~near)
    return dict(s=s, work=work, idx_of=idx_of, state=state, X=rec["f"], fidx=fidx, y=(st == "closed").astype(np.int32),
                usable=usable, blinks=blinks, edges=np.quantile(np.unique(fidx), np.linspace(0, 1, FOLDS + 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sessions", nargs="+", required=True)
    ap.add_argument("--header", required=True)
    ap.add_argument("--jitter-n", type=int, default=0); ap.add_argument("--jitter-px", type=int, default=16)
    a = ap.parse_args()
    S = [load(s, a.jitter_n, a.jitter_px) for s in a.sessions]
    for d in S: print(d["s"], len(d["blinks"]), "blinks,", int(d["usable"].sum()), "usable rows")
    ths = [0.0, 0.25, 0.5, 0.75, 1.0, 1.5]
    tally = {(d["s"], t): [0, 0, 0] for d in S for t in ths}
    for k in range(FOLDS):
        def held(d): return (d["fidx"] >= d["edges"][k]) & (d["fidx"] <= d["edges"][k + 1] if k == FOLDS - 1 else d["fidx"] < d["edges"][k + 1])
        def train_mask(d):
            lo, hi = d["edges"][k], d["edges"][k + 1]
            return d["usable"] & ((d["fidx"] < lo - 5) | (d["fidx"] > hi + 5))
        X = np.vstack([d["X"][train_mask(d)] for d in S]); y = np.concatenate([d["y"][train_mask(d)] for d in S])
        w, b = train_svm(X, y)
        for d in S:
            lo, hi = d["edges"][k], d["edges"][k + 1]
            fb = [blk for blk in d["blinks"] if lo <= blk[0] <= hi]
            for t in ths:
                m = os.path.join(d["work"], "fold_model.bin"); write_model(m, w, b, t)
                out = os.path.join(d["work"], "fold_hog.csv")
                replay(d["s"], d["work"], "--hog-model=" + m, "--hog-csv=" + out)
                ev = [d["idx_of"][int(r["timestamp_ms"])] for r in csv.DictReader(open(out))
                      if r["blink"] == "1" and int(r["timestamp_ms"]) in d["idx_of"]]
                ev = [e for e in ev if lo <= e <= hi]
                for i, v in enumerate(score_events(ev, d["state"], fb)): tally[(d["s"], t)][i] += v
        print("fold", k, "done", flush=True)
    best, bf = ths[0], -1
    for t in ths:
        TP = FP = N = 0
        for d in S:
            tp, fp, un = tally[(d["s"], t)]; n = len(d["blinks"]); TP += tp; FP += fp; N += n
            print(f"  th {t:+.2f} {os.path.basename(d['s'])}: caught {tp}/{n} ({tp/n:.0%}), FA {fp}, prec {tp/max(tp+fp,1):.0%}")
        r_, p_ = TP / N, TP / max(TP + FP, 1); f1 = 2 * r_ * p_ / max(r_ + p_, 1e-9)
        print(f"  th {t:+.2f} POOLED: recall {r_:.0%} precision {p_:.0%} F1 {f1:.2f}")
        if f1 > bf: best, bf = t, f1
    X = np.vstack([d["X"][d["usable"]] for d in S]); y = np.concatenate([d["y"][d["usable"]] for d in S])
    w, b = train_svm(X, y)
    write_header(a.header, w, b, best, ["Pooled " + ", ".join(os.path.basename(d["s"]) for d in S) +
                 f": {sum(len(d['blinks']) for d in S)} labelled blinks, {FOLDS}-fold CV best F1 {bf:.2f} at threshold {best:+.2f}."])
    print("pooled header ->", a.header)


if __name__ == "__main__":
    main()
