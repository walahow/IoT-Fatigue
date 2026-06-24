#!/usr/bin/env python3
"""
fuzzy_model.py — Mamdani Fuzzy Inference System: Fatigue Detection
====================================================================
IoT Fatigue Helmet | ESP32-S3-CAM Prototype
Python reference implementation + simulation tool.
The C++ equivalent lives in firmware/src/FuzzyFatigue.h.

Source of truth: thresholds_walkthrough.md (repo root)
Numerical verification: scratch/fis_sim.py

════════════════════════════════════════════════════════════════════
SYSTEM OVERVIEW
════════════════════════════════════════════════════════════════════
  3 raw sensor inputs
       │
       ▼
  Mamdani Fuzzification (all spatial MFs evaluated inside FIS)
       │
       ▼
  7-Rule Inference (AND=min, aggregation=max)
       │
       ▼
  Centroid Defuzzification (21 points on universe [0, 100])
       │
       ▼
  Risk Score (%) → Alert Level (Safe / Warning / Critical)

════════════════════════════════════════════════════════════════════
INPUT MEMBERSHIP FUNCTIONS
════════════════════════════════════════════════════════════════════

INPUT 1 — HR_Diff (%)
  Formula : (current_BPM − baseline_BPM) / baseline_BPM × 100
  Baseline: rolling 60-second average of valid BPM readings on boot

  MF          Shape              Breakpoints (a, b, c, d)
  ──────────  ─────────────────  ────────────────────────────────────
  Dropped     Left trapezoid     (−∞, −∞, −15, −5)   full ≤ −15%
  Stable      Trapezoid          (−15, −9, +10, +15)  flat-top −9 to +10%
  Elevated    Right trapezoid    (+10, +15, +∞, +∞)  full ≥ +15%

  Source: thresholds_walkthrough.md §1 (MePhy, FatigueSet datasets)

INPUT 2 — Blink_Rate (blinks/min)
  Source: 60-second rolling blink count from camera feature stream
          (USB mode: received via serial command BLINK:<float>\n from PC)

  MF          Shape              Breakpoints
  ──────────  ─────────────────  ────────────────────────────────────
  Low         Left trapezoid     (0, 0, 4, 8)         full ≤ 4 bl/min
  Normal      Trapezoid          (6, 8, 18, 20)        flat-top 8–18 bl/min
  High        Right trapezoid    (20, 24, +∞, +∞)     full ≥ 24 bl/min

  Source: thresholds_walkthrough.md §3 (Divjak 2009, IICIP 2016)
    Alert baseline  : 8–18 bl/min (IICIP 2016: 8–10; Divjak 2009: 18 ± 3)
    Low / fatigued  : 4–6 bl/min  (IICIP 2016: Phase 2 collapse)
    High / fighting : > 24 bl/min (Divjak 2009: Phase 1 rapid bursts)

INPUT 3 — IMU (two raw sub-values fuzzified inside FIS)
  Sub-inputs:
    gyro_var  — rolling variance of head_movement magnitude over last 10s
    pitch_deg — angular deviation from calibration gravity vector (°)
    nodding_score — pre-computed [0..1] from 6-second 10 Hz pitch buffer
                    Formula: clamp(ZCR/4, 0, 1) × (slope < 0 ? 1 : 0)
                    Engineering assumption: nodding frequency 0.5–2 Hz

  Sub-MF      Shape              Breakpoints        Source
  ──────────  ─────────────────  ─────────────────  ───────────────────────
  gyro_Stable Left trapezoid     (0, 0, 400, 800)   FatigueSet low-intensity
  gyro_Fidgety Right trapezoid   (750, 1200, ∞, ∞)  FatigueSet earable >1200
  pitch_Limp  Right trapezoid    (20, 25, ∞, ∞)     Alparslan / Freitas 2024

  NOTE: gyro_Fidgety starts rising at 750 (not 800) to avoid a dead zone at
  gyro_var=800 where both gyro_Stable and gyro_Fidgety = 0 simultaneously.

  Derived IMU memberships (computed inside FIS, NOT pre-computed in firmware):
    limp_drowsy  = min(pitch_Limp, gyro_Stable)
                   ← Limp only valid if head is ALSO still (not just nodding forward)
    IMU_Stable   = gyro_Stable
    IMU_Fidgety  = gyro_Fidgety
    IMU_Drowsy   = max(limp_drowsy, nodding_score)

  Source: thresholds_walkthrough.md §5 (FatigueSet, Alparslan, Freitas 2024)

════════════════════════════════════════════════════════════════════
OUTPUT MEMBERSHIP FUNCTIONS — Risk_Score (%)
════════════════════════════════════════════════════════════════════

  MF          Shape              Breakpoints         Alert action
  ──────────  ─────────────────  ──────────────────  ─────────────────────────
  Safe        Left trapezoid     (0, 0, 15, 30)       No alarm
  Warning     Triangle           (30, 50, 70)         Single beep + slow LED
  Critical    Right trapezoid    (70, 85, 100, 100)   Continuous buzzer + rapid LED

  Zone boundaries:
    Safe     :  0 – 30%   (Green)
    Warning  : 31 – 70%   (Yellow)
    Critical : 71 – 100%  (Red)

  Source: thresholds_walkthrough.md §6

════════════════════════════════════════════════════════════════════
RULE TABLE (7 RULES)
════════════════════════════════════════════════════════════════════

  #   Antecedent (AND = min)                              Consequent       Intent
  ──  ──────────────────────────────────────────────────  ───────────────  ──────────────────────────────────
  R1  HR_Stable   AND Blink_Normal AND IMU_Stable         Risk_Safe        Normal alert riding
  R2  HR_Dropped  AND Blink_Low                           Risk_Critical    Drowsiness/collapse (confirmed)
  R3  IMU_Drowsy                                          Risk_Critical    Head drop or nodding event
  R4  Blink_Low  AND NOT(HR_Dropped)                        Risk_Warning     Early single-sensor drowsiness signal
  R5  HR_Elevated AND IMU_Fidgety                         Risk_Warning     Active/stressed — not drowsy
  R6  Blink_High  AND HR_Stable                           Risk_Warning     Phase 1 fighting fatigue (Divjak 2009)
  R7  Blink_Normal AND HR_Stable AND IMU_Fidgety          Risk_Safe        False-positive prevention (active riding)

  Rule notes:
    R2 vs R4 — sensor escalation, NOT a duplicate:
      R2 = min(HR_Dropped, Blink_Low)              -> Critical (two sensors confirmed)
      R4 = min(Blink_Low, 1 - HR_Dropped)          -> Warning  (one sensor only)
      When HR is also dropping: 1-HR_Dropped -> 0, R4 backs off, R2 dominates.
      Ph5 diagnosis: original R4 = blink_Low gave 67.7% (WARN); FuzzyNOT gives 81.0% (CRIT).
      Verified numerically in scratch/fis_r4_diagnosis.py.

    R7 — mathematically required:
      Without R7, the state {HR_Stable, Blink_Normal, gyro >= 1200} produces zero
      activation across all rules, yielding centroid = 0/0 (UNDEFINED). In C++, this
      defaults to 50% -> Warning zone (false alarm). Verified numerically in fis_sim.py.
      R7 is an engineering addition for system robustness; declared as such in thesis.

    R6 — covers Phase 1 gap:
      Without R6, high blink rate with stable HR/IMU produces no rule activation
      (R1 needs Blink_Normal; R5 needs HR_Elevated). This would silently output
      Safe during active Phase 1 fatigue fighting. R6 closes this gap.

════════════════════════════════════════════════════════════════════
DEFUZZIFICATION
════════════════════════════════════════════════════════════════════
  Method  : Centroid (Center of Gravity)
  Points  : 21 discrete points, z_i = i × 5,  i = 0..20
  Formula : Σ(z_i × μ_agg(z_i)) / Σ(μ_agg(z_i))
  Fallback: if Σ(μ_agg) < ε → risk = 0.0 (Safe) — no rule fired

════════════════════════════════════════════════════════════════════
MODES
════════════════════════════════════════════════════════════════════
  --simulate          7-phase synthetic scenario, saves fuzzy_fatigue_simulation.png
  --session <path>    Load real dataset_merged.csv (future, when sessions available)
  --no-plot           Skip matplotlib output (headless / CI use)

════════════════════════════════════════════════════════════════════
REFERENCES
════════════════════════════════════════════════════════════════════
  thresholds_walkthrough.md   — primary design source (repo root)
  scratch/fis_sim.py          — numerical R7 verification + window analysis
  Divjak 2009                 — blink rate baseline and Phase 1/2 thresholds
  IICIP 2016                  — driving alert blink rate 8–10 bl/min
  FatigueSet (Nokia/Muse S)   — gyro variance thresholds at ear-level
  Alparslan et al.            — head drop as drowsiness marker
  Freitas et al. 2024         — nodding + head-tilted ratio as fatigue features
  MePhy Dataset (N=60)        — HR baseline and cognitive load thresholds
"""

import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")   # non-interactive backend — safe on headless / CI
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


# ─────────────────────────────────────────────────────────────────────────────
# Universe of discourse
# ─────────────────────────────────────────────────────────────────────────────

UNIVERSE = np.linspace(0, 100, 21)   # 0, 5, 10, ..., 100


# ─────────────────────────────────────────────────────────────────────────────
# Membership function primitives
# ─────────────────────────────────────────────────────────────────────────────

def _trap(x: float, a: float, b: float, c: float, d: float) -> float:
    """Trapezoidal MF: rises a→b, flat b→c, falls c→d.
    Left-shoulder  : a == b (or a = -inf).
    Right-shoulder : c == d (or d = +inf).
    """
    if x <= a or x >= d:
        return 0.0
    if x < b:
        return (x - a) / (b - a) if b > a else 1.0
    if x <= c:
        return 1.0
    return (d - x) / (d - c) if d > c else 0.0


def _tri(x: float, a: float, b: float, c: float) -> float:
    """Triangular MF (special case of trapezoid with b == c)."""
    return _trap(x, a, b, b, c)


# ─────────────────────────────────────────────────────────────────────────────
# Input membership functions
# ─────────────────────────────────────────────────────────────────────────────

# HR_Diff (%)
def hr_Dropped(v):   return _trap(v, -1e6, -1e6, -15.0,  -5.0)
def hr_Stable(v):    return _trap(v, -15.0,  -9.0, 10.0,  15.0)
def hr_Elevated(v):  return _trap(v,  10.0,  15.0, 1e6,   1e6)

# Blink_Rate (blinks/min)
def blink_Low(v):    return _trap(v,   0.0,   0.0,  4.0,   8.0)
def blink_Normal(v): return _trap(v,   6.0,   8.0, 18.0,  20.0)
def blink_High(v):   return _trap(v,  20.0,  24.0, 1e6,   1e6)

# IMU sub-MFs (gyro_var and pitch_deg — fuzzified inside FIS)
def gyro_Stable(v):  return _trap(v,   0.0,   0.0, 400.0, 800.0)
def gyro_Fidgety(v): return _trap(v, 750.0, 1200.0, 1e6,   1e6)   # starts at 750 (overlap fix)
def pitch_Limp(v):   return _trap(v,  20.0,  25.0, 1e6,   1e6)


# ─────────────────────────────────────────────────────────────────────────────
# Output membership functions
# ─────────────────────────────────────────────────────────────────────────────

def out_Safe(z):     return _trap(z,   0.0,  0.0, 15.0, 30.0)
def out_Warning(z):  return _tri( z,  30.0, 50.0, 70.0)
def out_Critical(z): return _trap(z,  70.0, 85.0, 100.0, 100.0)


# ─────────────────────────────────────────────────────────────────────────────
# Fuzzy Inference System
# ─────────────────────────────────────────────────────────────────────────────

class FuzzyFatigue:
    """
    Mamdani FIS for fatigue detection.

    All spatial (instantaneous) MF evaluations occur inside this class.
    The only pre-processed input is nodding_score (temporal, 0..1),
    computed externally from a 6-second 10 Hz pitch rolling buffer.
    """

    ALERT_SAFE     = 0
    ALERT_WARNING  = 1
    ALERT_CRITICAL = 2

    def __init__(self):
        self._risk  = 0.0
        self._alert = self.ALERT_SAFE

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self,
               hr_diff_pct:    float,
               blink_rate_bpm: float,
               gyro_var:       float,
               pitch_deg:      float,
               nodding_score:  float) -> tuple[float, int]:
        """
        Run one FIS inference tick (call at 1 Hz).

        Parameters
        ----------
        hr_diff_pct    : (BPM - baseline) / baseline * 100
        blink_rate_bpm : 60-second rolling blink rate
        gyro_var       : rolling variance of head_movement (last 10 s)
        pitch_deg      : angular deviation from calibration gravity vector
        nodding_score  : pre-computed [0..1] oscillation score from 6 s buffer

        Returns
        -------
        (risk_score: float, alert_level: int)
        """
        # ── Step 1: Fuzzify HR_Diff ───────────────────────────────────────────
        hD = hr_Dropped(hr_diff_pct)
        hS = hr_Stable(hr_diff_pct)
        hE = hr_Elevated(hr_diff_pct)

        # ── Step 2: Fuzzify Blink_Rate ────────────────────────────────────────
        bL = blink_Low(blink_rate_bpm)
        bN = blink_Normal(blink_rate_bpm)
        bH = blink_High(blink_rate_bpm)

        # ── Step 3: Fuzzify IMU sub-inputs (ALL inside FIS) ───────────────────
        gS = gyro_Stable(gyro_var)
        gF = gyro_Fidgety(gyro_var)
        pL = pitch_Limp(pitch_deg)

        # Combine: Limp is drowsy ONLY when gyro is also Stable (head is still)
        limp_drowsy  = min(pL, gS)
        imu_Drowsy   = max(limp_drowsy, float(nodding_score))
        imu_Stable   = gS
        imu_Fidgety  = gF

        # ── Step 4: Fire rules (AND = min) ────────────────────────────────────
        r1 = min(hS, bN, imu_Stable)   # R1: Normal alert riding    → Safe
        r2 = min(hD, bL)               # R2: Drowsiness collapse     → Critical
        r3 = imu_Drowsy                # R3: Head drop / nodding     → Critical
        r4 = min(bL, 1.0 - hD)         # R4: Blink_Low AND NOT(HR_Dropped) → Warning  [FuzzyNOT fix]
                                       #     Backs off when R2 (hD+bL) is active.
                                       #     Without fix: Ph5 = 67.7% (WARN); with fix: 81.0% (CRIT).
        r5 = min(hE, imu_Fidgety)      # R5: Active / stressed       → Warning
        r6 = min(bH, hS)              # R6: Phase 1 fighting        → Warning
        r7 = min(bN, hS, imu_Fidgety) # R7: False-positive guard    → Safe

        # ── Step 5: Aggregate (max-clip per output zone) ──────────────────────
        agg = np.zeros(21)
        for i, z in enumerate(UNIVERSE):
            s = out_Safe(z)
            w = out_Warning(z)
            c = out_Critical(z)
            safe_agg = max(min(r1, s), min(r7, s))
            warn_agg = max(min(r4, w), min(r5, w), min(r6, w))
            crit_agg = max(min(r2, c), min(r3, c))
            agg[i]   = max(safe_agg, warn_agg, crit_agg)

        # ── Step 6: Centroid defuzzification ──────────────────────────────────
        den = float(np.sum(agg))
        if den < 1e-9:
            # No rule fired: safe fallback (no alarm is better than false alarm)
            self._risk = 0.0
        else:
            self._risk = float(np.dot(UNIVERSE, agg) / den)

        # ── Step 7: Determine alert zone ──────────────────────────────────────
        if self._risk <= 30.0:
            self._alert = self.ALERT_SAFE
        elif self._risk <= 70.0:
            self._alert = self.ALERT_WARNING
        else:
            self._alert = self.ALERT_CRITICAL

        return self._risk, self._alert

    def get_risk(self)  -> float: return self._risk
    def get_alert(self) -> int:   return self._alert


# ─────────────────────────────────────────────────────────────────────────────
# Simulation — 7-phase synthetic scenario
# ─────────────────────────────────────────────────────────────────────────────

_PHASES = [
    # (hr_diff, blink, gyro_var, pitch_deg, nod_score, label)
    ( +5.0, 13.0,  200,  5.0, 0.00, "Ph1: Normal Alert"),
    (+20.0, 14.0, 1500,  8.0, 0.00, "Ph2: Stress Spike"),
    ( +3.0, 12.0,  300,  4.0, 0.00, "Ph3: Recovery"),
    ( -8.0,  5.0,  250,  7.0, 0.00, "Ph4: Early Drowsiness"),
    (-14.0,  3.0,  180, 10.0, 0.00, "Ph5: Full Collapse"),
    (-12.0,  4.0,  150, 28.0, 0.85, "Ph6: Head Drop"),
    ( +2.0, 10.0,  220,  5.0, 0.00, "Ph7: Recovery"),
]
_TICKS_PER_PHASE = 10


def simulate() -> dict:
    """Generate synthetic fatigue scenario across all 7 phases."""
    fis = FuzzyFatigue()
    rng = np.random.default_rng(seed=42)

    results = {k: [] for k in ("t", "hr", "blink", "gyro", "pitch", "nod",
                                "risk", "alert", "phase_starts", "phase_labels")}

    t = 0
    for ph_hr, ph_bk, ph_gv, ph_pt, ph_nod, label in _PHASES:
        results["phase_starts"].append(t)
        results["phase_labels"].append(label)
        for _ in range(_TICKS_PER_PHASE):
            hr    = ph_hr  + rng.normal(0, 0.5)
            blink = max(0.0, ph_bk  + rng.normal(0, 0.3))
            gyro  = max(0.0, ph_gv  + rng.normal(0, 20.0))
            pitch = max(0.0, ph_pt  + rng.normal(0, 0.3))
            nod   = ph_nod

            risk, alert = fis.update(hr, blink, gyro, pitch, nod)

            results["t"].append(t)
            results["hr"].append(hr)
            results["blink"].append(blink)
            results["gyro"].append(gyro)
            results["pitch"].append(pitch)
            results["nod"].append(nod)
            results["risk"].append(risk)
            results["alert"].append(alert)
            t += 1

    return results


# ─────────────────────────────────────────────────────────────────────────────
# Visualization
# ─────────────────────────────────────────────────────────────────────────────

_ALERT_COLOR = {0: "#27ae60", 1: "#e67e22", 2: "#e74c3c"}
_ALERT_LABEL = {0: "SAFE", 1: "WARN", 2: "CRIT"}


def plot_results(res: dict, save_path: str) -> None:
    fig = plt.figure(figsize=(14, 11))
    fig.patch.set_facecolor("#1a1a2e")
    fig.suptitle("Fuzzy Mamdani Fatigue Detection — 7-Phase Simulation",
                 fontsize=13, fontweight="bold", color="white", y=0.98)

    gs = gridspec.GridSpec(4, 1, hspace=0.50, top=0.94, bottom=0.06)
    axes = [fig.add_subplot(gs[i]) for i in range(4)]

    for ax in axes:
        ax.set_facecolor("#12122a")
        ax.tick_params(colors="gray", labelsize=8)
        ax.spines[:].set_color("#333355")
        ax.grid(True, alpha=0.2, color="gray")
        ax.set_xlim(0, max(res["t"]))
        # Phase dividers
        for ps, pl in zip(res["phase_starts"], res["phase_labels"]):
            ax.axvline(ps, color="#444466", linewidth=0.9, linestyle="--")

    times = res["t"]

    # ── Panel 1: HR_Diff ──────────────────────────────────────────────────────
    ax = axes[0]
    ax.fill_between(times, -1e3, -10, alpha=0.08, color="#3498db")
    ax.fill_between(times,  -10,  10, alpha=0.08, color="#2ecc71")
    ax.fill_between(times,   10, 1e3, alpha=0.08, color="#e74c3c")
    ax.plot(times, res["hr"], color="#5dade2", linewidth=1.6)
    ax.axhline(-10, color="#3498db", linestyle=":", linewidth=0.8)
    ax.axhline( 10, color="#2ecc71", linestyle=":", linewidth=0.8)
    ax.axhline( 15, color="#e74c3c", linestyle=":", linewidth=0.8)
    ax.set_ylim(-28, 32)
    ax.set_ylabel("HR_Diff (%)", color="white", fontsize=9)
    ax.text(1, -20, "Dropped", color="#3498db", fontsize=7)
    ax.text(1,   2, "Stable",  color="#2ecc71", fontsize=7)
    ax.text(1,  22, "Elevated",color="#e74c3c", fontsize=7)

    # ── Panel 2: Blink_Rate ───────────────────────────────────────────────────
    ax = axes[1]
    ax.fill_between(times, 0,  8, alpha=0.08, color="#9b59b6")
    ax.fill_between(times, 8, 18, alpha=0.08, color="#2ecc71")
    ax.fill_between(times,24, 40, alpha=0.08, color="#e67e22")
    ax.plot(times, res["blink"], color="#a29bfe", linewidth=1.6)
    for y, c in [(4, "#9b59b6"), (8, "#2ecc71"), (18, "#2ecc71"), (24, "#e67e22")]:
        ax.axhline(y, color=c, linestyle=":", linewidth=0.8)
    ax.set_ylim(0, 35)
    ax.set_ylabel("Blink Rate (bl/min)", color="white", fontsize=9)
    ax.text(1,  2, "Low",    color="#9b59b6", fontsize=7)
    ax.text(1, 12, "Normal", color="#2ecc71", fontsize=7)
    ax.text(1, 26, "High",   color="#e67e22", fontsize=7)

    # ── Panel 3: Pitch + Gyro summary ────────────────────────────────────────
    ax = axes[2]
    gyro_norm = [min(g / 15.0, 100) for g in res["gyro"]]
    ax.plot(times, gyro_norm,  color="#fd79a8", linewidth=1.3, label="Gyro var (/15)")
    ax.plot(times, res["pitch"], color="#ffeaa7", linewidth=1.3, label="Pitch (°)")
    ax.axhline(400 / 15, color="#fd79a8", linestyle=":", linewidth=0.7)
    ax.axhline(25,       color="#ffeaa7", linestyle=":", linewidth=0.7)
    ax.set_ylim(0, 120)
    ax.set_ylabel("IMU sub-inputs", color="white", fontsize=9)
    ax.legend(fontsize=7, loc="upper right", facecolor="#1a1a2e", edgecolor="gray",
              labelcolor="white")

    # ── Panel 4: Risk Score ───────────────────────────────────────────────────
    ax = axes[3]
    ax.fill_between(times,  0, 30,  alpha=0.10, color="#2ecc71")
    ax.fill_between(times, 30, 70,  alpha=0.10, color="#e67e22")
    ax.fill_between(times, 70, 100, alpha=0.10, color="#e74c3c")

    for i in range(len(times) - 1):
        ax.plot(times[i:i+2], res["risk"][i:i+2],
                color=_ALERT_COLOR[res["alert"][i]], linewidth=2.2)

    ax.axhline(30, color="#2ecc71", linestyle="--", linewidth=0.8)
    ax.axhline(70, color="#e74c3c", linestyle="--", linewidth=0.8)
    ax.set_ylim(0, 100)
    ax.set_ylabel("Risk Score (%)", color="white", fontsize=9)
    ax.set_xlabel("Time (seconds)", color="white", fontsize=9)
    ax.text(1, 12, "SAFE",     color="#2ecc71", fontsize=8, fontweight="bold")
    ax.text(1, 48, "WARNING",  color="#e67e22", fontsize=8, fontweight="bold")
    ax.text(1, 80, "CRITICAL", color="#e74c3c", fontsize=8, fontweight="bold")

    # Phase labels on bottom panel
    for ps, pl in zip(res["phase_starts"], res["phase_labels"]):
        ax.text(ps + 0.3, 97, pl, fontsize=6, color="#aaaaaa", va="top")

    for ax in axes:
        ax.yaxis.label.set_color("white")

    plt.savefig(save_path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"[PLOT] Saved → {save_path}")


# ─────────────────────────────────────────────────────────────────────────────
# Console table
# ─────────────────────────────────────────────────────────────────────────────

def print_table(res: dict) -> None:
    times        = res["t"]
    phase_starts = res["phase_starts"]
    phase_labels = res["phase_labels"]

    ph_idx = 0
    header = f"{'t':>4}  {'HR_Diff':>8}  {'Blink':>6}  {'Gyro':>6}  {'Pitch':>6}  {'Nod':>5}  {'Risk%':>7}  Zone"
    print("\n" + header)
    print("─" * len(header))

    for i, t in enumerate(times):
        if ph_idx < len(phase_starts) - 1 and t >= phase_starts[ph_idx + 1]:
            ph_idx += 1
        if t == phase_starts[ph_idx]:
            print(f"\n  ┌── {phase_labels[ph_idx]} ──")
        zone = _ALERT_LABEL[res["alert"][i]]
        print(f"  {t:>3}  {res['hr'][i]:>+8.1f}  {res['blink'][i]:>6.1f}"
              f"  {res['gyro'][i]:>6.0f}  {res['pitch'][i]:>6.1f}"
              f"  {res['nod'][i]:>5.2f}  {res['risk'][i]:>7.1f}  {zone}")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Fuzzy Mamdani FIS — Fatigue Detection (simulation + validation)"
    )
    parser.add_argument("--simulate", action="store_true",
                        help="Run 7-phase synthetic simulation (default if no --session)")
    parser.add_argument("--session", metavar="PATH",
                        help="Load real dataset_merged.csv for offline evaluation")
    parser.add_argument("--no-plot", action="store_true",
                        help="Skip plot generation (headless / CI use)")
    args = parser.parse_args()

    if args.session:
        print("[FIS] Session mode not yet implemented. Use --simulate for now.")
        return

    # Default: simulation mode
    print("[FIS] Running 7-phase synthetic simulation...")
    res = simulate()
    print_table(res)

    if not args.no_plot:
        save_path = "d:/proj/IoT-Fatigue/fatigue-helmet/python/fuzzy_fatigue_simulation.png"
        plot_results(res, save_path)


if __name__ == "__main__":
    main()
