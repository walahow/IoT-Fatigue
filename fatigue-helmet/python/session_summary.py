#!/usr/bin/env python3
"""
session_summary.py — what a recorded session actually contains, channel by channel.
===================================================================================
Reads sensor_data.csv and reports each channel's distribution, its trend across
thirds of the session, and the checks that decide whether the session is usable
as training data.

The point is to answer "did this session give us anything" before spending time
on video processing. On session_101 it surfaced, in one run, that heart rate held
100% contact with a real -1.59 bpm/min decline, that head movement rose over the
same period (the opposite of what drowsiness predicts), and that the 44% CRITICAL
fatigue score was being driven by a blink channel stuck near zero -- i.e. the
score was measuring a sensor fault, not the rider.

That last check is the important one. FuzzyFatigue takes blink rate as an input,
so a dead blink channel reads as "almost never blinking", which is a strong
drowsiness cue. Any session whose blink channel is flat should have risk_pct and
alert_level discarded rather than merely discounted.

Usage:
    python session_summary.py sessions/session_101
    python session_summary.py sessions/session_101 --csv     # machine-readable

Columns are the 18 the firmware writes (see main.cpp's #HEADER line). The file
is headerless on disk, so the names are supplied here.
"""

import argparse
import csv
import io
import os
import statistics as st
import sys

COLS = ["ts", "hr", "pulse", "ax", "ay", "az", "gx", "gy", "gz", "mov", "sq",
        "blink", "pitch", "gvar", "nod", "risk", "alert", "imu"]


def load(session):
    path = os.path.join(session, "sensor_data.csv")
    if not os.path.exists(path):
        raise SystemExit("no sensor_data.csv in %s" % session)
    rows = []
    for r in csv.reader(io.open(path)):
        # tolerate a header line and any short/blank rows
        if len(r) < len(COLS) or not r[0].lstrip("-").isdigit():
            continue
        rows.append(dict(zip(COLS, r)))
    if not rows:
        raise SystemExit("no usable rows in %s" % path)
    return rows


def metadata(session):
    path = os.path.join(session, "metadata.txt")
    meta = {}
    if os.path.exists(path):
        for line in io.open(path):
            if "=" in line:
                k, v = line.strip().split("=", 1)
                meta[k] = v
    return meta


def describe(label, v, unit=""):
    print("  %-13s min %8.2f  med %8.2f  mean %8.2f  max %8.2f  sd %7.2f %s"
          % (label, min(v), st.median(v), st.mean(v), max(v), st.pstdev(v), unit))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--csv", action="store_true", help="one machine-readable summary line")
    args = ap.parse_args()

    rows = load(args.session)
    meta = metadata(args.session)
    n = len(rows)
    t0, t1 = int(rows[0]["ts"]), int(rows[-1]["ts"])
    dur_min = (t1 - t0) / 60000.0
    rate = n / max(1e-9, (t1 - t0) / 1000.0)

    num = lambda k: [float(r[k]) for r in rows]
    hr, pitch, mov = num("hr"), num("pitch"), num("mov")
    gvar, nod, risk, blink = num("gvar"), num("nod"), num("risk"), num("blink")
    alert = [int(r["alert"]) for r in rows]
    contact = 100.0 * sum(1 for r in rows if r["sq"] == "1") / n

    # per-minute heart-rate means and a least-squares slope over them
    buckets = {}
    for r in rows:
        buckets.setdefault(int((int(r["ts"]) - t0) // 60000), []).append(float(r["hr"]))
    mins = sorted(buckets)
    means = [st.mean(buckets[m]) for m in mins]
    slope = 0.0
    if len(mins) > 1:
        mx, my = st.mean(mins), st.mean(means)
        denom = sum((x - mx) ** 2 for x in mins)
        slope = sum((x - mx) * (y - my) for x, y in zip(mins, means)) / denom if denom else 0.0

    blink_flat = (max(blink) - min(blink)) <= 1.0
    crit = 100.0 * sum(1 for a in alert if a == 2) / n

    if args.csv:
        print("session,rows,dur_min,rate_hz,contact_pct,hr_mean,hr_slope,"
              "pitch_med,mov_mean,crit_pct,blink_flat,labelled")
        print("%s,%d,%.2f,%.2f,%.1f,%.1f,%.3f,%.1f,%.1f,%.1f,%d,%d"
              % (os.path.basename(args.session.rstrip("/\\")), n, dur_min, rate,
                 contact, st.mean(hr), slope, st.median(pitch), st.mean(mov),
                 crit, int(blink_flat), int(len(rows[0]) > len(COLS))))
        return

    print("%s: %d rows, %.1f min, %.2f Hz" % (args.session, n, dur_min, rate))
    if meta:
        armed = [k for k in ("armed_imu", "armed_hr", "armed_eye") if k in meta]
        if armed:
            print("  arming: %s | timed_out=%s | %s ms"
                  % (" ".join("%s=%s" % (k.replace("armed_", ""), meta[k]) for k in armed),
                     meta.get("arming_timed_out", "?"), meta.get("arming_duration_ms", "?")))

    print("\nCHANNELS")
    describe("heart rate", hr, "bpm")
    describe("pitch", pitch, "deg")
    describe("head move", mov)
    describe("gyro var", gvar)
    describe("nod score", nod)
    describe("blink rate", blink, "/min")
    describe("risk", risk, "%")

    print("\nTREND (thirds)")
    k = n // 3
    print("  %-13s %9s %9s %9s" % ("", "first", "middle", "last"))
    for label, v in (("heart rate", hr), ("pitch", pitch), ("head move", mov),
                     ("gyro var", gvar), ("nod score", nod), ("risk %", risk)):
        print("  %-13s %9.2f %9.2f %9.2f"
              % (label, st.mean(v[:k]), st.mean(v[k:2 * k]), st.mean(v[2 * k:])))

    print("\nHEART RATE")
    d = [abs(hr[i] - hr[i - 1]) for i in range(1, n) if hr[i] > 0 and hr[i - 1] > 0]
    print("  contact          : %.1f%% of rows" % contact)
    print("  linear trend     : %+.2f bpm/min (%+.1f bpm over the session)"
          % (slope, slope * (max(mins) - min(mins)) if len(mins) > 1 else 0.0))
    if d:
        print("  row-to-row |d|   : mean %.2f bpm, max %.0f" % (st.mean(d), max(d)))

    print("\nVERDICT")
    ok = []
    bad = []
    (ok if contact >= 95 else bad).append(
        "heart rate: %.1f%% contact" % contact)
    (bad if blink_flat else ok).append(
        "blink rate: %s (range %.1f-%.1f/min)"
        % ("FLAT -- channel failed" if blink_flat else "varying", min(blink), max(blink)))
    if blink_flat:
        bad.append("risk/alert: DISCARD -- %.1f%% CRITICAL is driven by the dead "
                   "blink input, not the rider" % crit)
    labelled = len(rows[0]) > len(COLS)
    (ok if labelled else bad).append(
        "label: present" if labelled else "label: MISSING -- cannot enter training")
    if rate < 5:
        bad.append("sample rate: %.2f Hz -- the documented format assumes 100 Hz, "
                   "and HRV needs beat-level timing" % rate)
    for s in ok:
        print("  [ok]   %s" % s)
    for s in bad:
        print("  [flag] %s" % s)


if __name__ == "__main__":
    main()
