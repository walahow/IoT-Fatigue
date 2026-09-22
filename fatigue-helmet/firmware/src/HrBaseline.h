// HrBaseline.h -- resting heart-rate baseline, frozen once formed.
// Header-only and Arduino-free so test/test_fis_hr can run it on the PC.
//
// The baseline used to be the mean of the FIRST 20 valid readings. On the hardware
// test of 2026-09-21 those were 119, 116, 109, 111, ... falling to 78 within 15 s
// (the pulse channel settling after the sensor came on), so the baseline froze at 96.6
// BPM and ordinary later readings of 75-95 BPM scored -10..-22% ("HR dropped"),
// which is what R2 needs. Now the first SKIP valid readings are discarded and the
// baseline is the MEDIAN of the next N -- a settling transient or a few wild readings
// cannot drag it.
//
// A "wait until the readings agree" criterion was tried on paper and rejected: the
// recorded sessions 101-104 (sensor worn) have a median 20 s spread of 21-28 BPM,
// the same as the sensor-not-worn test (median 27), so such a gate would never open.
// That says more about the pulse channel than about the baseline; see the notes in
// the memory file. Frozen, not rolling, so progressive drowsiness is not normalised out.
#pragma once

#include <stdint.h>

struct HrBaseline {
    static const uint8_t SKIP = 10;   // valid readings discarded while the pulse channel settles
    static const uint8_t N    = 20;   // readings the median is taken over
    static const uint8_t NEEDED = SKIP + N;

    float   buf[N];
    uint8_t seen   = 0;               // valid readings pushed, saturating at SKIP
    uint8_t count  = 0;               // readings stored in buf
    bool    formed = false;
    float   bpm    = 0.0f;

    void reset() { seen = 0; count = 0; formed = false; bpm = 0.0f; }

    // Readings taken so far, out of NEEDED -- for the arming progress line.
    uint8_t progress() const { return (uint8_t)(seen + count); }

    // Push one valid reading (signal_quality == 1 and bpm > 0). Returns true on the
    // reading that forms the baseline; a formed baseline never changes.
    bool push(float v) {
        if (formed) return false;
        if (seen < SKIP) { seen++; return false; }
        buf[count++] = v;
        if (count < N) return false;

        float s[N];
        for (uint8_t i = 0; i < N; i++) s[i] = buf[i];
        for (uint8_t i = 1; i < N; i++) {                 // insertion sort, N = 20, runs once
            float x = s[i]; int j = i - 1;
            while (j >= 0 && s[j] > x) { s[j + 1] = s[j]; j--; }
            s[j + 1] = x;
        }
        bpm = 0.5f * (s[N / 2 - 1] + s[N / 2]);
        formed = true;
        return true;
    }
};
