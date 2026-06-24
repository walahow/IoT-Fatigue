/**
 * FuzzyFatigue.h  —  Mamdani Fuzzy Inference System: Fatigue Detection
 * ======================================================================
 * IoT Fatigue Helmet | ESP32-S3-CAM Firmware
 * Header-only, heap-free, STL-free implementation.
 * Direct C++ translation of the Python reference: fuzzy_model.py
 *
 * Source of truth : thresholds_walkthrough.md  (repo root)
 * Design rationale: fuzzy_walkthrough.md        (repo root)
 * Python reference: fatigue-helmet/python/fuzzy_model.py
 *
 * Usage (in main.cpp):
 *   #include "FuzzyFatigue.h"
 *   FuzzyFatigue fis;
 *   float risk; int alert;
 *   fis.update(hr_diff, blink_rate, gyro_var, pitch_deg, nod_score,
 *              risk, alert);
 *
 * ======================================================================
 * RULE TABLE (7 RULES)
 * ======================================================================
 *
 *   #   Antecedent (AND = min)                            Consequent      Intent
 *   --  -----------------------------------------------   ----------      --------------------------------
 *   R1  HR_Stable  AND Blink_Normal AND IMU_Stable        Risk_Safe       Normal alert riding
 *   R2  HR_Dropped AND Blink_Low                          Risk_Critical   Drowsiness confirmed (2 sensors)
 *   R3  IMU_Drowsy                                        Risk_Critical   Head drop or nodding event
 *   R4  Blink_Low  AND NOT(HR_Dropped)                    Risk_Warning    Early single-sensor drowsiness
 *   R5  HR_Elevated AND IMU_Fidgety                       Risk_Warning    Active / stressed — not drowsy
 *   R6  Blink_High  AND HR_Stable                         Risk_Warning    Phase 1 fighting fatigue
 *   R7  Blink_Normal AND HR_Stable AND IMU_Fidgety        Risk_Safe       False-positive guard (active riding)
 *
 *   Notes:
 *     R4 uses fuzzy NOT: antecedent = min(blink_Low, 1 - hr_Dropped)
 *       Semantic: "Blink drops, but HR not yet confirmed dropping."
 *       Without this fix, R4 = blink_Low fires at 1.0 while R2 fires at 0.9,
 *       pulling the centroid from ~87% (CRIT) down to ~68% (WARN) at Ph5.
 *       FuzzyNOT fix: Ph5 -> 81% (CRIT). Verified in scratch/fis_r4_diagnosis.py.
 *
 *     R7 is mathematically required (not just defensive):
 *       Without R7, gyro_var >= 1200 with normal HR/blink produces zero rule
 *       activation -> centroid = 0/0 -> fallback 0% (OK) or 50% (false WARN).
 *       R7 forces defined Safe output for active-riding state.
 *
 * ======================================================================
 * INPUT MEMBERSHIP FUNCTIONS
 * ======================================================================
 *
 *   HR_Diff (%) = (current_BPM - baseline_BPM) / baseline_BPM x 100
 *     Dropped   : left-trap  (-inf, -inf, -15, -5)   <- PNS drowsy
 *     Stable    : trapezoid  (-15, -9, +10, +15)      <- alert normal
 *     Elevated  : right-trap (+10, +15, +inf, +inf)   <- SNS stress
 *
 *   Blink_Rate (bl/min)
 *     Low       : left-trap  (0, 0, 4, 8)             <- Phase 2 collapse
 *     Normal    : trapezoid  (6, 8, 18, 20)            <- flat-top 8-18
 *     High      : right-trap (20, 24, +inf, +inf)     <- Phase 1 fighting
 *
 *   IMU sub-inputs (fuzzified inside FIS, NOT pre-computed)
 *     gyro_Stable   : left-trap  (0, 0, 400, 800)
 *     gyro_Fidgety  : right-trap (750, 1200, +inf, +inf)  <- 750 overlap fix
 *     pitch_Limp    : right-trap (20, 25, +inf, +inf)
 *
 *   Derived IMU memberships (fuzzy AND / OR, not crisp):
 *     limp_drowsy = min(pitch_Limp, gyro_Stable)  <- tilt + stillness both needed
 *     IMU_Drowsy  = max(limp_drowsy, nodding_score)
 *     IMU_Stable  = gyro_Stable
 *     IMU_Fidgety = gyro_Fidgety
 *
 *   nodding_score [0..1] is the ONLY pre-computed value, passed in by caller.
 *   It is computed from a 6-second 10 Hz pitch ring buffer in main.cpp:
 *     nodding_score = clamp(ZCR / 4.0, 0, 1) * (slope < 0 ? 1 : 0)
 *
 * ======================================================================
 * OUTPUT MEMBERSHIP FUNCTIONS — Risk_Score [0..100]
 * ======================================================================
 *
 *     Safe     : left-trap  (0, 0, 15, 30)
 *     Warning  : triangle   (30, 50, 70)
 *     Critical : right-trap (70, 85, 100, 100)
 *
 *   Alert zones:
 *     0 — Safe     :  0-30%
 *     1 — Warning  : 31-70%
 *     2 — Critical : 71-100%
 *
 * ======================================================================
 * DEFUZZIFICATION
 * ======================================================================
 *   Method  : Centroid (Center of Gravity)
 *   Universe: 21 discrete points  z[i] = i * 5,  i = 0..20
 *   Formula : SUM(z[i] * u_agg[i]) / SUM(u_agg[i])
 *   Fallback: SUM < 1e-6 -> risk = 0.0 (no rule fired, safe default)
 */

#pragma once

#include <Arduino.h>   // min/max, constrain, float
#include <math.h>      // fminf, fmaxf

// ─────────────────────────────────────────────────────────────────────────────
// Alert level constants (mirrors Python FuzzyFatigue class)
// ─────────────────────────────────────────────────────────────────────────────

#define ALERT_SAFE     0
#define ALERT_WARNING  1
#define ALERT_CRITICAL 2

// ─────────────────────────────────────────────────────────────────────────────
// Membership function primitives (static inline — no call overhead)
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Trapezoidal MF: rises a->b, flat b->c, falls c->d.
 *   Left-shoulder  : a == b (pass a = -1e9 for practical -inf)
 *   Right-shoulder : c == d (pass d = +1e9 for practical +inf)
 *
 * Matches Python _trap() exactly, including boundary conditions
 * (x <= a or x >= d returns 0.0).
 */
static inline float _ftrap(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) return 0.0f;
    if (x < b)  return (b > a) ? (x - a) / (b - a) : 1.0f;
    if (x <= c) return 1.0f;
    return (d > c) ? (d - x) / (d - c) : 0.0f;
}

/** Triangular MF (trapezoid with b == c). */
static inline float _ftri(float x, float a, float b, float c) {
    return _ftrap(x, a, b, b, c);
}

/** Convenience: two-argument fuzzy AND (min). */
static inline float _fand2(float a, float b)         { return fminf(a, b); }

/** Convenience: three-argument fuzzy AND (min). */
static inline float _fand3(float a, float b, float c) { return fminf(fminf(a, b), c); }

/** Convenience: fuzzy OR (max). */
static inline float _for2(float a, float b)           { return fmaxf(a, b); }
static inline float _for3(float a, float b, float c)  { return fmaxf(fmaxf(a, b), c); }

// ─────────────────────────────────────────────────────────────────────────────
// Input membership functions
// ─────────────────────────────────────────────────────────────────────────────

// ── HR_Diff (%) ──────────────────────────────────────────────────────────────
static inline float mf_hr_Dropped (float v) { return _ftrap(v, -1e9f, -1e9f, -15.0f, -5.0f); }
static inline float mf_hr_Stable  (float v) { return _ftrap(v, -15.0f, -9.0f, 10.0f, 15.0f); }
static inline float mf_hr_Elevated(float v) { return _ftrap(v,  10.0f, 15.0f, 1e9f,  1e9f); }

// ── Blink_Rate (bl/min) ──────────────────────────────────────────────────────
static inline float mf_blink_Low   (float v) { return _ftrap(v,  0.0f,  0.0f,  4.0f,  8.0f); }
static inline float mf_blink_Normal(float v) { return _ftrap(v,  6.0f,  8.0f, 18.0f, 20.0f); }
static inline float mf_blink_High  (float v) { return _ftrap(v, 20.0f, 24.0f, 1e9f,  1e9f); }

// ── IMU sub-MFs (gyro_var, pitch_deg — fuzzified inside FIS) ─────────────────
static inline float mf_gyro_Stable (float v) { return _ftrap(v,   0.0f,    0.0f, 400.0f, 800.0f); }
static inline float mf_gyro_Fidgety(float v) { return _ftrap(v, 750.0f, 1200.0f,   1e9f,   1e9f); }
//  ^ gyro_Fidgety starts rising at 750 (not 800): overlap fix avoids dead zone
//    at gyro_var=800 where both MFs would equal 0 simultaneously.
static inline float mf_pitch_Limp  (float v) { return _ftrap(v,  20.0f,   25.0f,   1e9f,   1e9f); }

// ── Output membership functions ───────────────────────────────────────────────
static inline float mf_out_Safe    (float z) { return _ftrap(z,   0.0f,   0.0f,  15.0f, 30.0f); }
static inline float mf_out_Warning (float z) { return _ftri( z,  30.0f,  50.0f,  70.0f); }
static inline float mf_out_Critical(float z) { return _ftrap(z,  70.0f,  85.0f, 100.0f, 100.0f); }

// ─────────────────────────────────────────────────────────────────────────────
// Universe of discourse  (21 points, z = 0, 5, 10, ..., 100)
// ─────────────────────────────────────────────────────────────────────────────

static const uint8_t FIS_UNIVERSE_N = 21;
static const float   FIS_UNIVERSE_STEP = 5.0f;   // z[i] = i * 5

// ─────────────────────────────────────────────────────────────────────────────
// FuzzyFatigue class
// ─────────────────────────────────────────────────────────────────────────────

class FuzzyFatigue {
public:
    FuzzyFatigue() : _risk(0.0f), _alert(ALERT_SAFE) {}

    // ── Public API ─────────────────────────────────────────────────────────────

    /**
     * Run one inference tick (call at 1 Hz from main loop).
     *
     * @param hr_diff_pct    (current_BPM - baseline_BPM) / baseline_BPM * 100
     * @param blink_rate_bpm 60-second rolling blink rate (bl/min)
     * @param gyro_var       Rolling variance of head_movement magnitude (10 s)
     * @param pitch_deg      Angular deviation from calibration gravity vector (deg)
     * @param nodding_score  Pre-computed [0..1] from 6-s 10 Hz pitch buffer
     * @param risk_out       [out] Risk score 0..100 (%)
     * @param alert_out      [out] Alert level: ALERT_SAFE / ALERT_WARNING / ALERT_CRITICAL
     */
    void update(float hr_diff_pct,
                float blink_rate_bpm,
                float gyro_var,
                float pitch_deg,
                float nodding_score,
                float &risk_out,
                int   &alert_out)
    {
        // ── Step 1: Fuzzify HR_Diff ───────────────────────────────────────────
        const float hD = mf_hr_Dropped (hr_diff_pct);
        const float hS = mf_hr_Stable  (hr_diff_pct);
        const float hE = mf_hr_Elevated(hr_diff_pct);

        // ── Step 2: Fuzzify Blink_Rate ────────────────────────────────────────
        const float bL = mf_blink_Low   (blink_rate_bpm);
        const float bN = mf_blink_Normal(blink_rate_bpm);
        const float bH = mf_blink_High  (blink_rate_bpm);

        // ── Step 3: Fuzzify IMU sub-inputs (ALL inside FIS, no crisp pre-compute)
        const float gS = mf_gyro_Stable (gyro_var);
        const float gF = mf_gyro_Fidgety(gyro_var);
        const float pL = mf_pitch_Limp  (pitch_deg);

        // Composite IMU memberships (fuzzy AND / OR, not crisp logic)
        const float nod         = constrain(nodding_score, 0.0f, 1.0f);
        const float limp_drowsy = _fand2(pL, gS);   // head-drop valid only when still
        const float imu_Drowsy  = _for2(limp_drowsy, nod);
        const float imu_Stable  = gS;
        const float imu_Fidgety = gF;

        // ── Step 4: Fire 7 rules (AND = min) ─────────────────────────────────
        //
        // R4 uses fuzzy NOT: min(blink_Low, 1 - hr_Dropped)
        //   -> R4 backs off automatically when R2 is active (both sensors confirm)
        //   -> Without this, R4 fires at 1.0 while R2 fires at 0.9, pulling the
        //      centroid from ~87% (CRIT) down to ~68% (WARN) at full-collapse state.

        const float r1 = _fand3(hS, bN, imu_Stable);           // Normal alert    -> Safe
        const float r2 = _fand2(hD, bL);                        // Confirmed drown -> Critical
        const float r3 = imu_Drowsy;                            // Head drop / nod -> Critical
        const float r4 = _fand2(bL, 1.0f - hD);               // Early blink-only -> Warning  [FuzzyNOT fix]
        const float r5 = _fand2(hE, imu_Fidgety);              // Active/stressed  -> Warning
        const float r6 = _fand2(bH, hS);                        // Phase 1 fight   -> Warning
        const float r7 = _fand3(bN, hS, imu_Fidgety);          // Active riding   -> Safe

        // ── Step 5: Aggregate output (max-clip per zone, iterate universe) ────
        float num = 0.0f;   // SUM(z * u_agg)
        float den = 0.0f;   // SUM(u_agg)

        for (uint8_t i = 0; i < FIS_UNIVERSE_N; i++) {
            const float z = i * FIS_UNIVERSE_STEP;

            // Evaluate output MFs at this z
            const float s = mf_out_Safe    (z);
            const float w = mf_out_Warning (z);
            const float c = mf_out_Critical(z);

            // Aggregate per zone (max-clip)
            const float safe_agg = _for2(_fand2(r1, s), _fand2(r7, s));
            const float warn_agg = _for3(_fand2(r4, w), _fand2(r5, w), _fand2(r6, w));
            const float crit_agg = _for2(_fand2(r2, c), _fand2(r3, c));

            // Global max across all zones
            const float u = _for3(safe_agg, warn_agg, crit_agg);

            num += z * u;
            den += u;
        }

        // ── Step 6: Centroid defuzzification ──────────────────────────────────
        if (den < 1e-6f) {
            // No rule fired — safe fallback (no alarm is better than false alarm)
            _risk = 0.0f;
        } else {
            _risk = num / den;
        }

        // ── Step 7: Classify alert level ──────────────────────────────────────
        if (_risk <= 30.0f) {
            _alert = ALERT_SAFE;
        } else if (_risk <= 70.0f) {
            _alert = ALERT_WARNING;
        } else {
            _alert = ALERT_CRITICAL;
        }

        risk_out  = _risk;
        alert_out = _alert;
    }

    /** Last computed risk score [0..100]. */
    float getRisk()  const { return _risk; }

    /** Last computed alert level (ALERT_SAFE / ALERT_WARNING / ALERT_CRITICAL). */
    int   getAlert() const { return _alert; }

private:
    float _risk;
    int   _alert;
};
