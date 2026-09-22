// Host-side checks for NodDetector.h and AlertGate.h.
// Run: pio test -e native -f test_nod_alert -v
#include <unity.h>
#include <cmath>
#include <cstdlib>
#include "NodDetector.h"
#include "AlertGate.h"

void setUp(void) {}
void tearDown(void) {}

// Deterministic jitter in [-1, 1]. rand() would make these tests depend on the libc.
static float jitter(uint32_t &s) {
    s = s * 1664525u + 1013904223u;
    return ((s >> 8) & 0xFFFF) / 32767.5f - 1.0f;
}

static float scoreOf(float (*pitchAt)(int, uint32_t &), uint32_t seed = 1) {
    NodDetector d;
    for (int i = 0; i < 60; i++) d.push(pitchAt(i, seed));
    return d.score();
}

// ── NodDetector ──────────────────────────────────────────────────────────────

static float restNoise(int, uint32_t &s)   { return 8.0f + 0.3f * jitter(s); }
static float ridingVibration(int, uint32_t &s) { return 10.0f + 3.0f * jitter(s); }
// Look down slowly over the window while vibrating: big swing, jitter under the band.
static float leanWithVibration(int i, uint32_t &s) { return 10.0f + 14.0f * i / 60.0f + 3.0f * jitter(s); }
// One glance at the dash: down at 2 s, back up at 4 s. (Two alternations: below the minimum.)
static float singleGlance(int i, uint32_t &) { return (i >= 20 && i < 40) ? 25.0f : 10.0f; }
// 1 Hz nodding, +-8 deg around 15.
static float nodding1Hz(int i, uint32_t &) { return 15.0f + 8.0f * sinf(2.0f * 3.14159265f * i / 10.0f); }
// Slow 0.33 Hz nodding, two full cycles in the window.
static float noddingSlow(int i, uint32_t &) { return 15.0f + 8.0f * sinf(2.0f * 3.14159265f * i / 30.0f); }
// Real nodding with the same riding vibration on top.
static float noddingWithVibration(int i, uint32_t &s) { return nodding1Hz(i, s) + 3.0f * jitter(s); }

void test_still_rider_scores_zero(void)        { TEST_ASSERT_EQUAL_FLOAT(0.0f, scoreOf(restNoise)); }
void test_riding_vibration_scores_zero(void)   { TEST_ASSERT_EQUAL_FLOAT(0.0f, scoreOf(ridingVibration)); }
void test_lean_with_vibration_scores_zero(void){ TEST_ASSERT_EQUAL_FLOAT(0.0f, scoreOf(leanWithVibration)); }
void test_single_glance_scores_zero(void)      { TEST_ASSERT_EQUAL_FLOAT(0.0f, scoreOf(singleGlance)); }

// Two glances inside the window is enough to count -- it is what a nod-nod looks like too.
static float twoGlances(int i, uint32_t &) { return ((i >= 8 && i < 18) || (i >= 34 && i < 44)) ? 25.0f : 10.0f; }
void test_two_glances_in_one_window_count(void) { TEST_ASSERT_TRUE(scoreOf(twoGlances) > 0.0f); }

void test_nodding_scores_high(void) {
    TEST_ASSERT_TRUE(scoreOf(nodding1Hz) >= 0.99f);
}
void test_slow_nodding_still_detected(void) {
    float s = scoreOf(noddingSlow);
    TEST_ASSERT_TRUE(s >= 0.4f && s <= 1.0f);
}
void test_nodding_survives_vibration(void) {
    TEST_ASSERT_TRUE(scoreOf(noddingWithVibration) >= 0.6f);
}

void test_nod_score_needs_a_filled_buffer(void) {
    NodDetector d;
    for (int i = 0; i < 5; i++) d.push(i % 2 ? 30.0f : 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.score());
}

void test_nod_score_does_not_depend_on_where_the_ring_wraps(void) {
    NodDetector a, b;
    uint32_t seed = 0;
    for (int i = 0; i < 60; i++) a.push(nodding1Hz(i, seed));
    for (int i = 0; i < 90; i++) b.push(nodding1Hz(i + 30, seed));   // wraps once
    TEST_ASSERT_TRUE(fabsf(a.score() - b.score()) < 0.01f);
}

// ── AlertGate ────────────────────────────────────────────────────────────────

// Feed `raw` once a second for `secs` seconds starting at t0; returns the gated level of the last tick.
static int feed(AlertGate &g, int raw, uint32_t t0, int secs, int *gatedAtEachSecond = nullptr) {
    int last = 0;
    for (int s = 0; s <= secs; s++) {
        last = g.update(raw, t0 + s * 1000u);
        if (gatedAtEachSecond) gatedAtEachSecond[s] = last;
    }
    return last;
}

void test_warning_needs_five_seconds(void) {
    AlertGate g; int at[10];
    feed(g, 1, 0, 9, at);
    TEST_ASSERT_EQUAL_INT(0, at[4]);
    TEST_ASSERT_EQUAL_INT(1, at[5]);
}

void test_critical_needs_eight_seconds_and_passes_through_warning(void) {
    AlertGate g; int at[12];
    feed(g, 2, 0, 11, at);
    TEST_ASSERT_EQUAL_INT(0, at[4]);
    TEST_ASSERT_EQUAL_INT(1, at[5]);
    TEST_ASSERT_EQUAL_INT(1, at[7]);
    TEST_ASSERT_EQUAL_INT(2, at[8]);
}

void test_short_critical_never_fires(void) {
    AlertGate g;
    feed(g, 2, 0, 3);                       // a 3 s glance
    int gated = feed(g, 0, 4000, 10);
    TEST_ASSERT_EQUAL_INT(0, gated);
}

void test_gate_holds_for_release_time_then_clears(void) {
    AlertGate g;
    feed(g, 2, 0, 10);                      // gated Critical
    TEST_ASSERT_EQUAL_INT(2, g.update(0, 11000));
    TEST_ASSERT_EQUAL_INT(2, g.update(0, 12000));
    TEST_ASSERT_EQUAL_INT(0, g.update(0, 14000));   // 3 s below -> released
}

void test_short_dip_does_not_restart_the_dwell(void) {
    AlertGate g;
    feed(g, 2, 0, 5);                       // 5 s critical
    g.update(0, 6000);                      // 1 s dip
    g.update(0, 7000);                      // 2 s dip, still inside the hold
    TEST_ASSERT_EQUAL_INT(2, g.update(2, 8000));   // stretch began at 0 -> 8 s reached
}

void test_critical_falls_back_to_warning(void) {
    AlertGate g;
    feed(g, 2, 0, 10);
    feed(g, 1, 11000, 4);                   // raw settles at Warning
    TEST_ASSERT_EQUAL_INT(1, g.update(1, 15000));
}

void test_reset_clears_state(void) {
    AlertGate g;
    feed(g, 2, 0, 10);
    g.reset();
    TEST_ASSERT_EQUAL_INT(0, g.update(0, 11000));
}

void test_gate_survives_millis_wrap(void) {
    AlertGate g; uint32_t t = 0xFFFFFFFFu - 2000u;
    int last = 0;
    for (int s = 0; s <= 9; s++) last = g.update(1, t + s * 1000u);
    TEST_ASSERT_EQUAL_INT(1, last);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_still_rider_scores_zero);
    RUN_TEST(test_riding_vibration_scores_zero);
    RUN_TEST(test_lean_with_vibration_scores_zero);
    RUN_TEST(test_single_glance_scores_zero);
    RUN_TEST(test_two_glances_in_one_window_count);
    RUN_TEST(test_nodding_scores_high);
    RUN_TEST(test_slow_nodding_still_detected);
    RUN_TEST(test_nodding_survives_vibration);
    RUN_TEST(test_nod_score_needs_a_filled_buffer);
    RUN_TEST(test_nod_score_does_not_depend_on_where_the_ring_wraps);
    RUN_TEST(test_warning_needs_five_seconds);
    RUN_TEST(test_critical_needs_eight_seconds_and_passes_through_warning);
    RUN_TEST(test_short_critical_never_fires);
    RUN_TEST(test_gate_holds_for_release_time_then_clears);
    RUN_TEST(test_short_dip_does_not_restart_the_dwell);
    RUN_TEST(test_critical_falls_back_to_warning);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_gate_survives_millis_wrap);
    return UNITY_END();
}
