# IoT Fatigue Helmet: Fuzzy Logic Inference System — Design Walkthrough

This document describes the complete design, mathematical formulation, and implementation decisions for the **Mamdani Fuzzy Inference System (FIS)** used in the IoT Fatigue Helmet.

It is a companion document to [`thresholds_walkthrough.md`](thresholds_walkthrough.md), which contains the physiological source data and literature references behind each threshold value. This document focuses on *how those thresholds were translated into a fuzzy logic system* and documents every design decision made during that process.

**Implementation files:**
- Python reference model: [`fatigue-helmet/python/fuzzy_model.py`](fatigue-helmet/python/fuzzy_model.py)
- C++ firmware class: [`fatigue-helmet/firmware/src/FuzzyFatigue.h`](fatigue-helmet/firmware/src/FuzzyFatigue.h) *(in progress)*
- Numerical verification: [`fatigue-helmet/python/scratch/fis_sim.py`](fatigue-helmet/python/scratch/fis_sim.py), [`fis_r4_diagnosis.py`](fatigue-helmet/python/scratch/fis_r4_diagnosis.py)

---

## 1. System Overview

### Why Fuzzy Logic?

The three sensor streams (heart rate, blink rate, IMU) each produce continuous numerical signals. Hard threshold comparisons (`if BPM < 55 then alert`) suffer from two problems:

1. **Brittleness at boundaries:** A reading of 55.1 BPM produces no alert; 54.9 BPM produces a full alarm. There is no biological justification for this discontinuity.
2. **False alarms on single sensors:** Each sensor can produce noise spikes. Requiring exact threshold crossings on any single sensor causes both false positives and false negatives.

A Fuzzy Inference System solves both problems by:
- Replacing binary membership ("is/isn't") with *degree of membership* (0.0–1.0) — smooth transitions across boundaries.
- Combining multiple sensor signals through *rules* — a single-sensor anomaly produces only a partial, lower-confidence output.

### Architecture

```
+------------------------------------------------------------------+
|  Raw Inputs (sampled at 1 Hz)                                    |
|                                                                  |
|  hr_diff_pct    = (BPM - baseline) / baseline x 100             |
|  blink_rate_bpm = 60-second rolling blink count from camera      |
|  gyro_var       = rolling variance of head_movement (10 s)       |
|  pitch_deg      = angular deviation from calibration vector (deg)|
|  nodding_score  = pre-computed [0..1] from 6 s oscillation buffer|
+---------------------------+--------------------------------------+
                            |
                            v
+------------------------------------------------------------------+
|  FUZZIFICATION                                                   |
|  All raw-to-membership conversions happen inside the FIS class.  |
|  Exception: nodding_score (temporal computation, passed in)      |
+---------------------------+--------------------------------------+
                            |
                            v
+------------------------------------------------------------------+
|  RULE INFERENCE  -  7 Rules                                      |
|  AND operator : min(uA, uB)                                      |
|  Aggregation  : max across all fired rules per output zone       |
+---------------------------+--------------------------------------+
                            |
                            v
+------------------------------------------------------------------+
|  CENTROID DEFUZZIFICATION                                        |
|  21 discrete points on universe [0, 100]                         |
|  Risk_Score = SUM(z_i x u_agg(z_i)) / SUM(u_agg(z_i))          |
+---------------------------+--------------------------------------+
                            |
                            v
+------------------------------------------------------------------+
|  OUTPUT: Risk_Score (%) -> Alert Level                           |
|   0-30%  -> Safe     (no alarm)                                  |
|  31-70%  -> Warning  (single beep + slow LED)                    |
|  71-100% -> Critical (continuous buzzer + rapid LED)             |
+------------------------------------------------------------------+
```

### Scope Constraints (Current Version)

| Input              | Status                                        | Reason for exclusion                               |
|:-------------------|:----------------------------------------------|:---------------------------------------------------|
| `HR_Diff`          | Included                                      | Direct sensor output from pulse sensor             |
| `Blink_Rate`       | Included                                      | Computed from camera stream via Python             |
| `IMU_State`        | Included (gyro_var + pitch_deg + nodding)     | MPU-6050 on helmet                                 |
| PERCLOS / EAR      | Excluded for now                              | Requires reliable eye-detection session data       |
| HRV (RMSSD/SDNN)   | Excluded for now                              | Requires clean RR-interval signal quality          |

---

## 2. Input Membership Functions

### 2.1 Heart Rate Deviation — `HR_Diff` (%)

**Calculation:** `HR_Diff = (current_BPM - baseline_BPM) / baseline_BPM x 100`

The baseline is computed as a rolling 60-second average of valid BPM readings (signal_quality = 1, BPM > 0) during the first minute of operation.

**Membership function shapes:**

```
  u (membership)
  |
1 |####\                    /----------\                    /####
  |     \                  /            \                  /
  |      \                /              \                /
0 +-------\--------------/                \--------------/-------> HR_Diff (%)
           |    |        |        |       |      |       |
         -15   -9       -9        0      +10    +12     +15

         [= Dropped =]   [===== Stable (flat top -9 to +10) =====]   [= Elevated =]
```

| MF         | Shape           | Breakpoints (a, b, c, d) | Interpretation                     |
|:-----------|:----------------|:-------------------------|:-----------------------------------|
| `Dropped`  | Left trapezoid  | (-inf, -inf, -15, -5)    | PNS-dominant drowsy state          |
| `Stable`   | Trapezoid       | (-15, -9, +10, +15)      | Normal alert driving fluctuation   |
| `Elevated` | Right trapezoid | (+10, +15, +inf, +inf)   | SNS-dominant cognitive stress      |

> **Design note:** `Stable` has a **flat top from -9 to +10** — both are literature-defined boundary values from the MePhy dataset. Neither edge is more "typical" than the other, so neither should have lower membership.

**Source:** `thresholds_walkthrough.md §1` — MePhy dataset (N=60), FatigueSet.

---

### 2.2 Blink Rate — `Blink_Rate` (blinks/min)

Computed as a 60-second rolling count of blink events detected by the camera pipeline. In USB mode, Python sends this value to the ESP32 via serial command `BLINK:<float>\n` every second.

**Two-phase blink model** (from `thresholds_walkthrough.md §3`):
- **Phase 1 — Fight fatigue:** Driver actively resists drowsiness → rapid blink bursts → `High` (>24 bl/min)
- **Phase 2 — Collapse:** Eyelid motor control fails → blink frequency drops → `Low` (<6 bl/min)

**Membership function shapes:**

```
  u (membership)
  |
1 |##\                    /----------\                    /##
  |   \                  / (flat top) \                  /
  |    \                /              \                /
0 +-----\--------------/                \--------------/--------> bl/min
         |   |    |  |                  |   |       |
         0   4    6  8                 18  20      24

         [= Low =]   [========= Normal (flat top 8 to 18) =========]   [= High =]
```

| MF       | Shape           | Breakpoints (a, b, c, d)  | Source                                                  |
|:---------|:----------------|:--------------------------|:--------------------------------------------------------|
| `Low`    | Left trapezoid  | (0, 0, 4, 8)              | IICIP 2016: 4-6 bl/min = fatigued driving state         |
| `Normal` | Trapezoid       | (6, 8, 18, 20)            | IICIP 2016: 8-10 bl/min alert; Divjak 2009: 18±3 rest  |
| `High`   | Right trapezoid | (20, 24, +inf, +inf)      | Divjak 2009: >24 bl/min = Phase 1 fighting fatigue      |

> **Design note:** `Normal` is a **trapezoid with a flat top from 8 to 18**, not a triangle. Both 8 and 18 are literature-defined boundary values for the alert state; neither is more "normal" than the other, so neither should have lower membership.

---

### 2.3 IMU State (composite — three sub-detectors)

The IMU input is **not a single crisp pre-computed number**. Instead, three independent fuzzy sub-detectors each produce a membership degree, which are then combined inside the FIS before rule evaluation.

This design choice is intentional: pre-computing a single "IMU score" before fuzzification would make the fuzzy layer cosmetic — the actual decision would already be made by crisp if-else logic. All three sub-detectors use genuine MFs, and their combination is a fuzzy operation.

#### Sub-detector A: Gyro Variance — `gyro_var`

Rolling variance of `head_movement` magnitude over the last 10 seconds, computed at 1 Hz.

```
  u (membership)
  |
1 |######\                                          /######
  |       \                                        /
  |        \                                      /
0 +----------\------------------------------------/-----------> gyro_var
              |                    |            |
             400                  750          1200

             [======= gyro_Stable =======]
                                    [===== gyro_Fidgety =====]
                                    ^
                                    overlap starts at 750
```

| MF             | Shape           | Breakpoints               | Source                                             |
|:---------------|:----------------|:--------------------------|:---------------------------------------------------|
| `gyro_Stable`  | Left trapezoid  | (0, 0, 400, 800)          | FatigueSet: low-intensity baseline variance ~143-274 |
| `gyro_Fidgety` | Right trapezoid | **(750, 1200, +inf, +inf)** | FatigueSet: high-intensity earable variance 1200-5000+ |

> **Design note — overlap at 750:** `gyro_Fidgety` starts rising at **750**, not 800. Without this overlap, both MFs equal 0 simultaneously at gyro_var=800, creating a dead zone where no rule fires and the centroid is undefined. The overlap region (750-800) ensures at least one MF always has partial membership.

#### Sub-detector B: Pitch Tilt — `pitch_deg`

Angular deviation from the calibration gravity vector, computed using:
```
pitch_deg = acos( dot(g_calib, g_now) / (||g_calib|| x ||g_now||) ) x (180/pi)
```

This dot-product method is orientation-independent and handles any helmet mounting angle.

```
  u (membership)
  |
1 |######\                                    /######
  |       \                                  /
  |        \                                /
0 +----------\------------------------------/-----------> pitch_deg
              |    |                   |    |
              0   10                  20   25

              [= pitch_Normal =]   [= pitch_Limp =]
```

| MF             | Shape           | Breakpoints            | Source                                                       |
|:---------------|:----------------|:-----------------------|:-------------------------------------------------------------|
| `pitch_Normal` | Left trapezoid  | (0, 0, 10, 15)         | Normal active riding range                                   |
| `pitch_Limp`   | Right trapezoid | (20, 25, +inf, +inf)   | Alparslan: "heads start to fall down"; Freitas 2024: head-tilted ratio |

#### Sub-detector C: Nodding Oscillation — `nodding_score` [0..1]

This detector identifies the characteristic rhythmic forward-backward micro-adjustment pattern as a rider fights sleep. It is the only sub-detector computed **externally** (in `main.cpp`) rather than inside the FIS class, because it requires a temporal rolling window rather than an instantaneous MF evaluation.

**Algorithm (computed at 1 Hz from 6-second, 10 Hz pitch buffer):**
```
1. Collect pitch readings at 10 Hz into a 60-sample ring buffer
2. Detrend the buffer (subtract mean)
3. Count zero crossings per second -> ZCR
4. Compute linear regression slope of the raw (non-detrended) buffer
5. nodding_score = clamp(ZCR / 4.0, 0.0, 1.0) x (slope < 0 ? 1.0 : 0.0)
```

The **slope gate** (`slope < 0`) ensures the oscillation is accompanied by a net downward drift — distinguishing progressive head-drop from stable back-and-forth movements during active riding.

> **Engineering assumption:** The target nodding frequency range of **0.5–2 Hz** is an engineering assumption based on typical human head dynamics, not a specific literature claim. It will be declared as such in the thesis methodology section.

**Why 6 seconds?**

| Window              | 0.5 Hz cycles | 0.5 Hz ZCR | Reliable?           | Detection latency |
|:--------------------|:--------------|:-----------|:--------------------|:------------------|
| 3 sec (30 samples)  | 1.5           | ~3         | No — below 2 cycles | 3 s               |
| **6 sec (60 samples)** | **3.0**    | **~6**     | **Yes**             | **6 s**           |
| 10 sec (100 samples)| 5.0           | ~10        | Yes — overkill      | 10 s              |

Minimum 3 complete cycles of the lower-bound frequency (0.5 Hz) are required for reliable zero-crossing detection. 6 seconds at 10 Hz (60 samples) is the minimum window that satisfies this. 6-second detection latency is acceptable for a fatigue monitoring system.

**Why 10 Hz sampling?**

The main CSV output loop runs at 1 Hz. To detect 2 Hz oscillation (Nyquist: sampling must be > 2x signal frequency), the MPU-6050 must be read at >= 4 Hz for the oscillation buffer. The firmware reads the IMU at 10 Hz into a background ring buffer; the 1 Hz CSV tick reads from this buffer and resets it.

#### Final IMU memberships (combined inside FIS)

```python
limp_drowsy  = min(pitch_Limp, gyro_Stable)
               # Valid head-drop only when head is also STILL.
               # Prevents misclassifying an active forward-lean as drowsy.

IMU_Stable   = gyro_Stable
IMU_Fidgety  = gyro_Fidgety
IMU_Drowsy   = max(limp_drowsy, nodding_score)
               # Either static head-drop OR rhythmic nodding -> Drowsy.
```

---

### 2.4 Output — `Risk_Score` (%)

```
  u (membership)
  |
1 |####\             /\             /####
  |     \           /  \           /
  |      \         /    \         /
0 +-------\-------/      \-------/---------> Risk_Score (%)
            |    |    |    |    |
           15   30   50   70   85

           [= Safe =]   [= Warning =]   [= Critical =]
                           (peak 50)
```

| MF         | Shape           | Breakpoints          | Alert action                        |
|:-----------|:----------------|:---------------------|:------------------------------------|
| `Safe`     | Left trapezoid  | (0, 0, 15, 30)       | No alarm                            |
| `Warning`  | Triangle        | (30, 50, 70)         | Single beep + slow LED blink        |
| `Critical` | Right trapezoid | (70, 85, 100, 100)   | Continuous buzzer + rapid LED       |

**Zone boundaries:**
- Safe:     0–30%
- Warning: 31–70%
- Critical: 71–100%

---

## 3. The 7-Rule Set

### Rule table

| Rule   | Antecedent (AND = min)                          | Consequent      | Intent                                | Source                                    |
|:-------|:------------------------------------------------|:----------------|:--------------------------------------|:------------------------------------------|
| **R1** | `HR_Stable AND Blink_Normal AND IMU_Stable`     | `Risk_Safe`     | Normal alert riding                   | `thresholds_walkthrough.md §6 R1`         |
| **R2** | `HR_Dropped AND Blink_Low`                      | `Risk_Critical` | Drowsiness confirmed by two sensors   | `thresholds_walkthrough.md §6 R2`         |
| **R3** | `IMU_Drowsy`                                    | `Risk_Critical` | Head drop or nodding event            | `thresholds_walkthrough.md §6 R3`         |
| **R4** | `Blink_Low AND NOT(HR_Dropped)`                 | `Risk_Warning`  | Early single-sensor drowsiness signal | `thresholds_walkthrough.md §6 R4` (modified) |
| **R5** | `HR_Elevated AND IMU_Fidgety`                   | `Risk_Warning`  | Active/stressed state — not drowsy    | `thresholds_walkthrough.md §6 R5`         |
| **R6** | `Blink_High AND HR_Stable`                      | `Risk_Warning`  | Phase 1 fighting fatigue              | Divjak 2009 (Phase 1 rapid bursts)        |
| **R7** | `Blink_Normal AND HR_Stable AND IMU_Fidgety`    | `Risk_Safe`     | Active riding — false positive guard  | Engineering addition                      |

### Original 5-rule base (from `thresholds_walkthrough.md`)

The `thresholds_walkthrough.md §6` specifies 5 rules using `EAR_PERCLOS` and `IMU_Limp/Fidgety` as inputs. Since PERCLOS is excluded from the current implementation scope, the rules were adapted:
- `EAR_PERCLOS Closed` → `Blink_Low`
- `EAR_PERCLOS Sluggish` → `Blink_Low` (same mapping at this scope level)
- `IMU_Limp` → `IMU_Drowsy` (now covering both static head-drop and nodding)

Two rules were **added** beyond the original 5:
- **R6** was added to close a coverage gap: without it, the state `{Blink_High, HR_Stable, IMU_Stable}` (Phase 1 fighting fatigue per Divjak 2009) fires no rule and produces an undefined or Safe output — a false negative during an active fatigue signal.
- **R7** was added as an engineering robustness measure (see §4.2 below).

---

## 4. Design Decisions and Numerical Justifications

### 4.1 R4 — Why `NOT(HR_Dropped)`, not `Blink_Low` alone

**Original design:** `R4: Blink_Low -> Warning`

**Problem discovered during simulation:** R4 fires at full strength (1.0) simultaneously with R2 (0.9), pulling the centroid down from Critical into Warning territory.

**Numerical diagnosis (Ph5: Full Collapse scenario):**

```
  Input: HR_Diff = -14%, Blink = 3 bl/min, Gyro = 180, Pitch = 10 deg

  hr_Dropped(-14%)  = 0.90    <- HR firmly in Dropped zone
  blink_Low(3)      = 1.00    <- Blink fully in Low zone

  R2 = min(0.90, 1.00) = 0.90  ->  clips Critical MF at 0.9
  R4 = blink_Low       = 1.00  ->  clips Warning  MF at 1.0  (FULL strength)

  Centroid with R4 original  ->  67.7%  (WARN)  <- WRONG
  Centroid with R4 disabled  ->  86.8%  (CRIT)  <- correct
  Delta: 19.1 percentage points pulled down by R4
```

**Root cause:** `Blink_Low` alone is a perfect subset condition of R2. Whenever R2 fires (both HR_Dropped AND Blink_Low), R4 also fires at full strength — the Warning MF centered at 50 generates enough aggregated area to dominate the centroid.

**The fix:** `R4 = min(blink_Low, 1 - hr_Dropped)` — standard Mamdani fuzzy NOT.

**Semantic interpretation:** *"Blink rate drops, AND heart rate is NOT yet confirmed dropping."* This implements a **sensor escalation** pattern:
- Single sensor confirms drowsiness (blink only): R4 dominates → **Warning**
- Two sensors confirm drowsiness (blink + HR): R2 dominates, R4 automatically backs off → **Critical**

**R4 firing strength across HR operating points (blink_Low = 1.0):**

| HR_Diff | hr_Dropped | 1 - hr_Dropped | R4 fires at | Verdict               |
|:--------|:-----------|:---------------|:------------|:----------------------|
| -16%    | 1.000      | 0.000          | **0.000**   | R2 takes over         |
| -14%    | 0.900      | 0.100          | **0.100**   | R2 dominates, R4 silent |
| -10%    | 0.500      | 0.500          | **0.500**   | Moderate Warning      |
| -6%     | 0.100      | 0.900          | **0.900**   | Strong Warning        |
| 0%      | 0.000      | 1.000          | **1.000**   | Full Warning          |

**Effect across all 7 simulation phases:**

| Phase                 | Original      | FuzzyNOT fix      | Expected |
|:----------------------|:--------------|:------------------|:---------|
| Ph1: Normal Alert     | 13% (SAFE) ✓  | 13% (SAFE) ✓      | SAFE     |
| Ph2: Stress Spike     | 50% (WARN) ✓  | 50% (WARN) ✓      | WARN     |
| Ph3: Recovery         | 13% (SAFE) ✓  | 13% (SAFE) ✓      | SAFE     |
| Ph4: Early Drowsiness | 60% (WARN) ✓  | 60% (WARN) ✓      | WARN     |
| Ph5: Full Collapse    | 68% (WARN) ✗  | **81% (CRIT) ✓**  | CRIT     |
| Ph6: Head Drop        | 69% (WARN) ✗  | **75% (CRIT) ✓**  | CRIT     |
| Ph7: Recovery         | 13% (SAFE) ✓  | 13% (SAFE) ✓      | SAFE     |

Verification script: `scratch/fis_r4_diagnosis.py`

> **Thesis note:** The use of `1 - hr_Dropped` is a standard Mamdani fuzzy complement operation. It does not introduce rule weights or priority mechanisms — the antecedent of R4 is simply `min(u_Blink_Low, 1 - u_HR_Dropped)`, which is a valid fuzzy proposition equivalent to "Blink is Low AND HR is NOT Dropped."

---

### 4.2 R7 — Why it is mathematically required (not just defensive)

**Problem:** When `gyro_var >= 1000` with normal HR and blink rate (active riding scenario):
- `gyro_Stable = 0` (already faded out)
- `gyro_Fidgety > 0` (starting to activate)
- R1 dies: `min(HR_Stable, Blink_Normal, IMU_Stable=0) = 0`
- R5 dies: `min(HR_Elevated=0, IMU_Fidgety) = 0`
- All other rules: zero activation

Without R7, **zero rules fire**. Centroid = 0/0 → undefined. In C++, this defaults to either 0 (Safe, acceptable) or 50 (Warning — a false alarm on every pothole).

**Numerical verification:**

| gyro_var | IMU(Dr, St, Fd)    | Risk with R7   | Risk without R7  |
|:---------|:-------------------|:---------------|:-----------------|
| 1500     | (0.00, 0.00, 1.00) | 12.9% (SAFE) ✓ | UNDEFINED        |
| 1200     | (0.00, 0.00, 1.00) | 12.9% (SAFE) ✓ | UNDEFINED        |
| 1000     | (0.00, 0.00, 0.50) | 14.3% (SAFE) ✓ | UNDEFINED        |
| 600      | (0.00, 0.50, 0.00) | 14.3% (SAFE) ✓ | 14.3% (SAFE) ✓  |

Verification script: `scratch/fis_sim.py`

> **Thesis note:** R7 is declared as an engineering addition for system robustness, not derived from literature. Its role is to provide a defined output when the rider is physically active (high gyro variance) but shows no ocular or cardiac fatigue signals. The rule is conservative: when only the IMU is active, the system defaults to Safe rather than generating a false Warning.

---

### 4.3 IMU composite design — why not a single pre-computed value

An earlier design proposal mapped `(pitch_deg, gyro_var)` → single crisp 0–100 value using if-else logic, then fuzzified that single number. This was rejected for two reasons:

1. **The fuzzy decision is made before fuzzification:** The if-else logic (`if pitch >= 25 AND gyro < 400 -> 0.0`) already decides "drowsy or not" in crisp Boolean terms. The subsequent fuzzification adds no analytical value.
2. **Not defensible to thesis examiners:** If asked "where does fuzzy logic operate on the IMU input?", the answer would be "on a number already determined by crisp if-else." The three-sub-detector design gives a clear answer: "each raw sub-metric is fuzzified by its own membership function; their combination is a fuzzy AND/OR operation."

The `min(pitch_Limp, gyro_Stable)` combination is the key insight: it enforces that head-drop Drowsy membership is *jointly* determined by both tilt angle AND stillness. If the head is tilting forward but the gyro is active (rider looking at road), `limp_drowsy` is suppressed by the low `gyro_Stable` value.

---

### 4.4 gyro_Fidgety overlap fix (750 vs 800)

The original design set `gyro_Fidgety` to start rising at 800, which is exactly where `gyro_Stable` fades to 0. At gyro_var = 800 exactly:
- `gyro_Stable(800) = 0.0` (just faded out)
- `gyro_Fidgety(800) = 0.0` (just starting to rise)

Both are simultaneously zero → neither R1 (needs `IMU_Stable > 0`) nor R7 (needs `IMU_Fidgety > 0`) fire → undefined centroid. Shifting `gyro_Fidgety` to start at 750 creates an overlap region (750–800) where both MFs have partial membership, eliminating the dead zone.

---

## 5. Defuzzification

### Centroid method (Center of Gravity)

**Universe:** [0, 100%] discretized to 21 points: `z_i = 5i`, `i = 0..20`

**Formula:**
```
Risk_Score = SUM( z_i x u_aggregated(z_i) ) / SUM( u_aggregated(z_i) )
```

**Aggregation:** The aggregated MF at each point is the maximum across all clipped output contributions:
```
u_agg(z) = max(
    max( min(r1, Safe(z)),  min(r7, Safe(z)) ),      <- Safe rules
    max( min(r4, Warn(z)),  min(r5, Warn(z)),
         min(r6, Warn(z)) ),                          <- Warning rules
    max( min(r2, Crit(z)),  min(r3, Crit(z)) )        <- Critical rules
)
```

**Fallback:** If `SUM(u_agg) < eps` (no rules fired), `Risk_Score = 0.0` (Safe default). This should not occur with the 7-rule set covering all expected operating states, but is retained as a safety fallback.

**Why 21 points?**

21 points at step size 5 provides adequate centroid accuracy for this application. The output zone boundaries (30, 70) align exactly with discretization points, ensuring no zone-boundary ambiguity from quantization. Increasing to 101 points would improve accuracy by ~0.5% but adds negligible value given sensor noise levels.

---

## 6. Complete Updated Rule Set (Final)

```
R1: IF  HR_Stable
    AND Blink_Normal
    AND IMU_Stable
    THEN Risk_Safe

R2: IF  HR_Dropped
    AND Blink_Low
    THEN Risk_Critical

R3: IF  IMU_Drowsy
    THEN Risk_Critical

R4: IF  Blink_Low
    AND NOT(HR_Dropped)         <- min(blink_Low, 1 - hr_Dropped)
    THEN Risk_Warning

R5: IF  HR_Elevated
    AND IMU_Fidgety
    THEN Risk_Warning

R6: IF  Blink_High
    AND HR_Stable
    THEN Risk_Warning

R7: IF  Blink_Normal
    AND HR_Stable
    AND IMU_Fidgety
    THEN Risk_Safe
```

**AND operator:** `min(uA, uB)`
**NOT operator:** `1 - uA` (standard Mamdani complement)
**Aggregation:** `max` across rules per output zone

---

## 7. Alert Level Mapping

| Risk_Score | Alert Level  | Buzzer (GPIO 14) | LED                | Intended Response      |
|:-----------|:-------------|:-----------------|:-------------------|:-----------------------|
| 0–30%      | 0 — Safe     | Off              | Off                | No intervention        |
| 31–70%     | 1 — Warning  | Single beep      | Slow blink (1 Hz)  | Rider awareness        |
| 71–100%    | 2 — Critical | Continuous       | Rapid blink (5 Hz) | Immediate intervention |

**GPIO notes:**
- `BUZZER_PIN = 14`
- `STATUS_LED_PIN = -1` (disabled by default; GPIO 2 is used by MPU-6050 I2C SDA)
- External LED can be connected to any free GPIO and configured at compile time

---

## 8. Simulation Validation

The Python file `fuzzy_model.py` implements `--simulate` mode, which generates a 7-phase synthetic scenario and saves the output plot as `fuzzy_fatigue_simulation.png`.

### 7-phase scenario design

| Phase                 | HR_Diff | Blink | Gyro Var | Pitch | Nod  | Expected |
|:----------------------|:--------|:------|:---------|:------|:-----|:---------|
| Ph1: Normal Alert     | +5%     | 13    | 200      | 5 deg | 0.00 | Safe     |
| Ph2: Stress Spike     | +20%    | 14    | 1500     | 8 deg | 0.00 | Warning  |
| Ph3: Recovery         | +3%     | 12    | 300      | 4 deg | 0.00 | Safe     |
| Ph4: Early Drowsiness | -8%     | 5     | 250      | 7 deg | 0.00 | Warning  |
| Ph5: Full Collapse    | -14%    | 3     | 180      | 10 deg| 0.00 | Critical |
| Ph6: Head Drop        | -12%    | 4     | 150      | 28 deg| 0.85 | Critical |
| Ph7: Recovery         | +2%     | 10    | 220      | 5 deg | 0.00 | Safe     |

### Simulation results (with FuzzyNOT R4 fix applied)

All 7 phases match expected zone after R4 fix:

```
Ph1: Normal Alert     ->  12.9%   SAFE      (R1: all three sensors normal)
Ph2: Stress Spike     ->  50.0%   WARNING   (R5: HR_Elevated + IMU_Fidgety)
Ph3: Recovery         ->  12.9%   SAFE      (R1)
Ph4: Early Drowsiness ->  60.0%   WARNING   (R4: Blink_Low, HR not yet confirmed)
Ph5: Full Collapse    ->  81.0%   CRITICAL  (R2 dominates, R4 backs off to 0.1)
Ph6: Head Drop        ->  75.0%   CRITICAL  (R3: IMU_Drowsy from pitch + nod)
Ph7: Recovery         ->  12.9%   SAFE      (R1)
```

---

## 9. Next Implementation Steps

- [x] Python reference FIS (`fuzzy_model.py`) — complete
- [x] 7-phase simulation validation — complete
- [x] R4 fuzzy NOT fix — validated numerically, pending code update
- [ ] Apply R4 fix in `fuzzy_model.py`
- [ ] C++ header `FuzzyFatigue.h` — 5-input `update()`, centroid, alert level
- [ ] `main.cpp` changes: pitch calc, gyro rolling variance, 10 Hz IMU buffer, nodding detector, baseline HR, serial BLINK parser, buzzer GPIO 14
- [ ] Hardware test: head-drop, serial BLINK command, normal state
