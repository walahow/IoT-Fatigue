// Host-side checks for FuzzyFatigue.h (blink hole, blink_valid, alpha-cut) and HrBaseline.h.
// Run: pio test -e native -f test_fis_hr -v
#include <unity.h>
#include <cmath>
#include "FuzzyFatigue.h"
#include "HrBaseline.h"

void setUp(void) {}
void tearDown(void) {}

struct Out { float risk; int alert; };
static Out fis(float hr, float blink, float gyro, float pitch, float nod, bool blinkValid = true) {
    FuzzyFatigue f; Out o;
    f.update(hr, blink, gyro, pitch, nod, o.risk, o.alert, blinkValid);
    return o;
}

// ── FuzzyFatigue ─────────────────────────────────────────────────────────────

void test_zero_blinks_is_low_not_a_rule_hole(void) {
    Out o = fis(0, 0.0f, 150, 8, 0);            // was: no rule fired -> risk 0 / Safe
    TEST_ASSERT_EQUAL_INT(ALERT_WARNING, o.alert);
    TEST_ASSERT_EQUAL_FLOAT(fis(0, 1.0f, 150, 8, 0).risk, o.risk);
}

void test_blink_offline_ignores_blink_rules(void) {
    Out o = fis(0, 0.0f, 150, 8, 0, false);      // blink unknown, HR/IMU quiet -> nothing fires
    TEST_ASSERT_EQUAL_INT(ALERT_SAFE, o.alert);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.risk);
    // ...but a real head drop still alarms with blink offline
    TEST_ASSERT_EQUAL_INT(ALERT_CRITICAL, fis(0, 0.0f, 150, 28, 0.85f, false).alert);
}

void test_weak_critical_evidence_is_not_critical(void) {
    // HR dropped 22%, blink rate 6.96 -> Blink_Low = 0.01, so R2 = 0.01. Used to give risk 85.
    Out o = fis(-22.0f, 6.96f, 150, 8, 0);
    TEST_ASSERT_TRUE(o.alert != ALERT_CRITICAL);
}

void test_strong_critical_evidence_still_critical(void) {
    TEST_ASSERT_EQUAL_INT(ALERT_CRITICAL, fis(-20.0f, 3.0f, 150, 8, 0).alert);   // R2 at 1.0
    // limp head held still, blink channel offline so R1 (Safe) cannot argue back
    TEST_ASSERT_EQUAL_INT(ALERT_CRITICAL, fis(0, 13.0f, 150, 28, 0.0f, false).alert);
}

void test_rule_at_exactly_the_cut_still_fires(void) {
    // Blink_Low(6.0) = (7-6)/(7-3) = 0.25 exactly: R4 fires (Warning), it is not cut.
    TEST_ASSERT_EQUAL_INT(ALERT_WARNING, fis(0, 6.0f, 150, 8, 0).alert);
}

void test_alert_riding_stays_safe(void) {
    Out o = fis(2.0f, 13.0f, 200, 5, 0);
    TEST_ASSERT_EQUAL_INT(ALERT_SAFE, o.alert);
}

// The documented 7-phase scenario (fuzzy_model.py _PHASES), noise-free.
void test_seven_phase_scenario_levels(void) {
    struct P { float hr, blink, gyro, pitch, nod; int expect; } ph[] = {
        {  5.0f, 13.0f,  200,  5.0f, 0.00f, ALERT_SAFE},      // Ph1 normal alert
        { 20.0f, 14.0f, 1500,  8.0f, 0.00f, ALERT_WARNING},   // Ph2 stress spike
        {  3.0f, 12.0f,  300,  4.0f, 0.00f, ALERT_SAFE},      // Ph3 recovery
        { -8.0f,  5.0f,  250,  7.0f, 0.00f, ALERT_WARNING},   // Ph4 early drowsiness
        {-14.0f,  3.0f,  180, 10.0f, 0.00f, ALERT_CRITICAL},  // Ph5 full collapse
        {-12.0f,  4.0f,  150, 28.0f, 0.85f, ALERT_CRITICAL},  // Ph6 head drop
        {  2.0f, 10.0f,  220,  5.0f, 0.00f, ALERT_SAFE},      // Ph7 recovery
    };
    for (unsigned i = 0; i < sizeof(ph) / sizeof(ph[0]); i++) {
        Out o = fis(ph[i].hr, ph[i].blink, ph[i].gyro, ph[i].pitch, ph[i].nod);
        TEST_ASSERT_EQUAL_INT_MESSAGE(ph[i].expect, o.alert, "phase level changed");
    }
}

// ── HrBaseline ───────────────────────────────────────────────────────────────

void test_baseline_discards_the_settling_readings(void) {
    // The 2026-09-21 hardware log: 10 settling readings, then the pulse holds ~85.
    float settling[10] = {119, 116, 109, 111, 109, 104, 106, 108, 108, 98};
    HrBaseline b;
    for (int i = 0; i < 10; i++) TEST_ASSERT_FALSE(b.push(settling[i]));
    bool formedNow = false;
    for (int i = 0; i < 20; i++) formedNow = b.push(85.0f + (i % 3));
    TEST_ASSERT_TRUE(formedNow);
    TEST_ASSERT_TRUE(b.formed);
    TEST_ASSERT_TRUE(fabsf(b.bpm - 86.0f) < 1.0f);          // not the old 96.6
}

void test_baseline_median_ignores_wild_readings(void) {
    HrBaseline b;
    for (int i = 0; i < 10; i++) b.push(120.0f);
    for (int i = 0; i < 20; i++) b.push((i == 3 || i == 11 || i == 17) ? 200.0f : 80.0f);
    TEST_ASSERT_EQUAL_FLOAT(80.0f, b.bpm);
}

void test_baseline_needs_thirty_readings_and_freezes(void) {
    HrBaseline b;
    for (int i = 0; i < 29; i++) TEST_ASSERT_FALSE(b.push(80.0f));
    TEST_ASSERT_EQUAL_UINT8(29, b.progress());
    TEST_ASSERT_TRUE(b.push(80.0f));
    TEST_ASSERT_FALSE(b.push(150.0f));                       // frozen once formed
    TEST_ASSERT_EQUAL_FLOAT(80.0f, b.bpm);
}

void test_baseline_reset_starts_over(void) {
    HrBaseline b;
    for (int i = 0; i < 30; i++) b.push(80.0f);
    b.reset();
    TEST_ASSERT_FALSE(b.formed);
    TEST_ASSERT_EQUAL_UINT8(0, b.progress());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_blinks_is_low_not_a_rule_hole);
    RUN_TEST(test_blink_offline_ignores_blink_rules);
    RUN_TEST(test_weak_critical_evidence_is_not_critical);
    RUN_TEST(test_strong_critical_evidence_still_critical);
    RUN_TEST(test_rule_at_exactly_the_cut_still_fires);
    RUN_TEST(test_alert_riding_stays_safe);
    RUN_TEST(test_seven_phase_scenario_levels);
    RUN_TEST(test_baseline_discards_the_settling_readings);
    RUN_TEST(test_baseline_median_ignores_wild_readings);
    RUN_TEST(test_baseline_needs_thirty_readings_and_freezes);
    RUN_TEST(test_baseline_reset_starts_over);
    return UNITY_END();
}
