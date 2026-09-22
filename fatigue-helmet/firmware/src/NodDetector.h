// NodDetector.h -- nodding score from the 10 Hz pitch ring buffer.
// Header-only and Arduino-free so test/test_nod_alert can run it on the PC.
//
// History. The first version scored clamp(zero-crossings/s / 4) gated by "slope > 0",
// behind a 2 deg peak-to-peak floor. On sessions 101-104 (riding) it was non-zero in
// ~50% of seconds, and the rate did not depend on how much the head moved: 47-60% at
// 2-4 deg of swing, 41-71% at 15-20 deg, 44-68% above 20 deg. Three causes:
//   - pitch comes from the accelerometer's gravity vector, so bumps, braking and engine
//     vibration show up as several degrees of pitch "swing" -- far above the 2 deg floor,
//     which was measured with the rider sitting still;
//   - zero crossings of the detrended signal count every wobble around the mean, however
//     small;
//   - a regression slope over a window that is mostly oscillation has a random sign, so
//     the slope gate just halved the firing rate.
// And because the fuzzy model's Critical output is a clipped shoulder whose centroid
// barely moves with the clip height, ANY non-zero nod score meant Critical: the score is
// effectively a boolean, so it has to be 0 unless the motion really looks like nodding.
//
// Now: the window must swing at least MIN_SWING_DEG, and only alternations between
// +band and -band around the window mean count (a Schmitt trigger, band = 1/4 of the
// swing), so jitter riding on a big slow movement is ignored. At least MIN_CROSSINGS
// alternations are needed: a single glance down and back is two, so it takes more than
// one glance's worth of movement -- a nod-nod, or two glances inside the 6 s window. The
// slope gate is gone: real nodding has no preferred slope sign.
//
// UNVALIDATED against real nods: every recorded session is 1 Hz and none contains a
// labelled nod. The constants are set from the rider-still noise floor and the typical
// size of a drowsy head nod (>= ~10 deg); tune them on a recording of deliberate nods.
#pragma once

#include <stdint.h>
#include <string.h>

struct NodDetector {
    static const uint8_t N = 60;   // 6 s x 10 Hz
    float   buf[N];
    uint8_t head;
    uint8_t count;   // saturates at N

    NodDetector() : head(0), count(0) { memset(buf, 0, sizeof(buf)); }

    void push(float pitch_deg) {
        buf[head] = pitch_deg;
        head      = (head + 1) % N;
        if (count < N) count++;
    }

    void reset() { head = 0; count = 0; }

    static constexpr float MIN_SWING_DEG   = 10.0f;  // peak-to-peak within the window
    static constexpr float BAND_FRACTION   = 0.25f;  // hysteresis band = this x swing
    static const int       MIN_CROSSINGS   = 3;      // +band/-band alternations; one glance = 2
    static const int       FULL_CROSSINGS  = 6;      // score reaches 1.0 here

    // Returns nodding_score [0..1]: 0 unless the window holds >= MIN_CROSSINGS deep
    // alternations. Chronological index: count<N -> oldest at buf[0]; count==N -> buf[head].
    float score() const {
        if (count < 6) return 0.0f;

        float mean = 0.0f;
        float lo = buf[(count < N) ? 0 : head];
        float hi = lo;
        for (uint8_t i = 0; i < count; i++) {
            const float v = buf[(count < N) ? i : (uint8_t)((head + i) % N)];
            mean += v;
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        mean /= (float)count;

        const float swing = hi - lo;
        if (swing < MIN_SWING_DEG) return 0.0f;
        const float band = swing * BAND_FRACTION;

        int state = 0, crossings = 0;   // state: +1 above +band, -1 below -band
        for (uint8_t i = 0; i < count; i++) {
            const float v = buf[(count < N) ? i : (uint8_t)((head + i) % N)] - mean;
            if (v > band)       { if (state == -1) crossings++; state = 1;  }
            else if (v < -band) { if (state ==  1) crossings++; state = -1; }
        }
        if (crossings < MIN_CROSSINGS) return 0.0f;

        const float s = (float)crossings / (float)FULL_CROSSINGS;
        return s > 1.0f ? 1.0f : s;
    }
};
