// AlertGate.h -- turns the fuzzy model's per-second alert into the one the rider hears.
// Header-only and Arduino-free so test/test_nod_alert and tools/session_replay can run it.
//
// Why it exists: the raw alert fires the instant a condition holds, so a glance at a
// mirror or the dash (a second or two of head motion) beeped exactly like a real head
// drop. On sessions 101-104 the raw Critical episodes were 78-91% <= 5 s long, and there
// were 324-414 Critical onsets per hour.
//
// A level is gated ON once the raw alert has been at or above it for its dwell time
// (Warning 5 s, Critical 8 s) and gated OFF once the raw alert has been below it for
// RELEASE_HOLD_MS. Dips shorter than the hold neither restart the dwell nor cancel the
// alert. Raw Critical counts toward Warning too, so a Critical stretch reaches gated
// Warning after 5 s and gated Critical after 8 s. The raw alert keeps being logged as
// alert_level; this one is alert_gated, and is what drives the buzzer.
#pragma once

#include <stdint.h>

struct AlertGate {
    static const uint32_t WARN_DWELL_MS    = 5000;
    static const uint32_t CRIT_DWELL_MS    = 8000;
    static const uint32_t RELEASE_HOLD_MS  = 3000;

    bool     active[3]   = {false, false, false};   // index = level (0 unused)
    uint32_t start[3]    = {0, 0, 0};                // when the current stretch began
    uint32_t lastSeen[3] = {0, 0, 0};                // last tick with raw >= level

    void reset() {
        for (int i = 0; i < 3; i++) { active[i] = false; start[i] = lastSeen[i] = 0; }
    }

    // raw: 0 safe / 1 warning / 2 critical. Returns the gated level. Timestamps are
    // compared as unsigned differences, so millis() wrap and SD-stall gaps are fine.
    int update(int raw, uint32_t nowMs) {
        int gated = 0;
        for (int level = 1; level <= 2; level++) {
            if (raw >= level) {
                if (!active[level]) { active[level] = true; start[level] = nowMs; }
                lastSeen[level] = nowMs;
            } else if (active[level] && (uint32_t)(nowMs - lastSeen[level]) >= RELEASE_HOLD_MS) {
                active[level] = false;
            }
            const uint32_t dwell = (level == 1) ? WARN_DWELL_MS : CRIT_DWELL_MS;
            if (active[level] && (uint32_t)(nowMs - start[level]) >= dwell) gated = level;
        }
        return gated;
    }
};
