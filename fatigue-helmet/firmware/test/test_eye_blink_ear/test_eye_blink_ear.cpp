#include <unity.h>
#include <cstring>
#include <cmath>
#include "EyeBlinkEAR.h"

void setUp(void) {}
void tearDown(void) {}

void test_otsu_separates_two_clusters(void) {
    uint8_t img[100];  // 10x10, two flat-ish clusters with a small spread
    for (int i = 0; i < 50; i++) img[i] = 40 + (i % 10);
    for (int i = 50; i < 100; i++) img[i] = 190 + ((i - 50) % 10);

    uint8_t threshold = EyeBlinkEAR::otsuThreshold(img, 10, 10);

    for (int i = 0; i < 50; i++) TEST_ASSERT_TRUE(img[i] <= threshold);
    for (int i = 50; i < 100; i++) TEST_ASSERT_TRUE(img[i] > threshold);
}

static void fillEllipseMask(uint8_t *mask, int w, int h, int cx, int cy, int rx, int ry) {
    memset(mask, 0, (size_t)w * h);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double nx = (x - cx) / (double)rx;
            double ny = (y - cy) / (double)ry;
            if (nx * nx + ny * ny <= 1.0) mask[y * w + x] = 1;
        }
    }
}

void test_ear_round_blob_is_near_one(void) {
    uint8_t mask[40 * 40];
    fillEllipseMask(mask, 40, 40, 20, 20, 15, 15);

    EyeBlinkEAR::EarResult r = EyeBlinkEAR::computeEAR(mask, 40, 40);

    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_TRUE(r.ear > 0.9f);
}

void test_ear_squashed_blob_is_near_zero(void) {
    uint8_t mask[40 * 40];
    fillEllipseMask(mask, 40, 40, 20, 20, 15, 4);

    EyeBlinkEAR::EarResult r = EyeBlinkEAR::computeEAR(mask, 40, 40);

    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_TRUE(r.ear < 0.35f);
}

void test_ear_empty_mask_is_invalid(void) {
    uint8_t mask[40 * 40];
    memset(mask, 0, sizeof(mask));

    EyeBlinkEAR::EarResult r = EyeBlinkEAR::computeEAR(mask, 40, 40);

    TEST_ASSERT_FALSE(r.valid);
}

void test_centroid_of_offset_blob(void) {
    uint8_t mask[20 * 20];
    fillEllipseMask(mask, 20, 20, 15, 5, 3, 3);

    float cx = -1, cy = -1;
    bool found = EyeBlinkEAR::centroid(mask, 20, 20, cx, cy);

    TEST_ASSERT_TRUE(found);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 15.0f, cx);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 5.0f, cy);
}

void test_centroid_of_empty_mask_fails(void) {
    uint8_t mask[20 * 20];
    memset(mask, 0, sizeof(mask));

    float cx, cy;
    bool found = EyeBlinkEAR::centroid(mask, 20, 20, cx, cy);

    TEST_ASSERT_FALSE(found);
}

void test_lock_roi_ignores_outlier_via_median(void) {
    float xs[5] = {10, 12, 11, 50, 11};  // index 3 is a bad single-frame misread
    float ys[5] = {10, 9, 11, 50, 10};

    EyeBlinkEAR::RoiLock roi;
    bool ok = EyeBlinkEAR::lockRoiFromSamples(xs, ys, 5, 20, 100, 100, roi);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(roi.locked);
    // median x = 11, median y = 10 -> roi (size 20) centered there -> x=1, y=0
    TEST_ASSERT_EQUAL_INT(1, roi.x);
    TEST_ASSERT_EQUAL_INT(0, roi.y);
    TEST_ASSERT_EQUAL_INT(20, roi.size);
}

void test_lock_roi_clamps_to_frame_bounds(void) {
    float xs[3] = {2, 2, 2};
    float ys[3] = {2, 2, 2};

    EyeBlinkEAR::RoiLock roi;
    bool ok = EyeBlinkEAR::lockRoiFromSamples(xs, ys, 3, 20, 100, 100, roi);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(roi.x >= 0);
    TEST_ASSERT_TRUE(roi.y >= 0);
}

void test_drift_blend_accepts_small_shift(void) {
    float outCx, outCy;
    bool moved = EyeBlinkEAR::driftBlend(100.0f, 100.0f, 105.0f, 100.0f,
                                          20.0f, 0.3f, outCx, outCy);

    TEST_ASSERT_TRUE(moved);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 101.5f, outCx);  // 100*0.7 + 105*0.3
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, outCy);
}

void test_drift_blend_rejects_large_shift(void) {
    float outCx, outCy;
    bool moved = EyeBlinkEAR::driftBlend(100.0f, 100.0f, 200.0f, 100.0f,
                                          20.0f, 0.3f, outCx, outCy);

    TEST_ASSERT_FALSE(moved);
}

void test_baseline_is_75th_percentile(void) {
    EyeBlinkEAR::BlinkDetector det;
    float values[5] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f};
    for (int i = 0; i < 5; i++) det.update(values[i], (uint32_t)(i * 100));

    // sorted == same order here; idx = int(5*0.75) = 3 -> 0.4
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.4f, det.baseline());
}

void test_blink_detected_on_dip_and_recovery(void) {
    EyeBlinkEAR::BlinkDetector det;
    uint32_t t = 0;

    // Warm up with stable "eye open" EAR so the baseline settles near 0.9.
    for (int i = 0; i <= EyeBlinkEAR::BlinkDetector::WARMUP_SAMPLES; i++) {
        bool evt = det.update(0.9f, t);
        TEST_ASSERT_FALSE(evt);
        t += 50;
    }

    bool duringDip = det.update(0.2f, t);  // below 0.9*0.7 = 0.63
    t += 50;
    TEST_ASSERT_FALSE(duringDip);  // blink run started, not yet completed

    bool afterRecovery = det.update(0.9f, t);  // back above threshold
    TEST_ASSERT_TRUE(afterRecovery);  // blink completes on recovery
}

void test_cooldown_prevents_double_count(void) {
    EyeBlinkEAR::BlinkDetector det;
    uint32_t t = 0;

    for (int i = 0; i <= EyeBlinkEAR::BlinkDetector::WARMUP_SAMPLES; i++) {
        det.update(0.9f, t);
        t += 50;
    }
    det.update(0.2f, t); t += 50;
    bool firstBlink = det.update(0.9f, t); t += 50;
    TEST_ASSERT_TRUE(firstBlink);

    // Dip again immediately, well within the cooldown window.
    det.update(0.2f, t); t += 50;
    bool secondAttempt = det.update(0.9f, t); t += 50;
    TEST_ASSERT_FALSE(secondAttempt);  // suppressed by cooldown
}

void test_rolling_rate_evicts_old_blinks(void) {
    EyeBlinkEAR::BlinkDetector det;
    det.pushBlinkTimestamp(1000);
    det.pushBlinkTimestamp(2000);

    float rateNow = det.rollingRateBpm(30000);  // both within last 60s
    TEST_ASSERT_EQUAL_FLOAT(2.0f, rateNow);

    float rateLater = det.rollingRateBpm(70000);  // both now older than 60s
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rateLater);
}

void test_percentile_threshold_isolates_darkest_fraction(void) {
    uint8_t img[100];
    for (int i = 0; i < 10; i++) img[i] = 5;     // darkest 10 pixels
    for (int i = 10; i < 100; i++) img[i] = 200; // remaining 90, bright

    uint8_t threshold = EyeBlinkEAR::percentileThreshold(img, 100, 1, 10.0f);

    for (int i = 0; i < 10; i++) TEST_ASSERT_TRUE(img[i] <= threshold);
    for (int i = 10; i < 100; i++) TEST_ASSERT_TRUE(img[i] > threshold);
}

void test_darkest_window_prefers_compact_blob_over_diffuse_region(void) {
    const int W = 40, H = 40;
    uint8_t mask[W * H];
    memset(mask, 0, sizeof(mask));

    // Diffuse region: sparse grid over a big area -- best 8x8 window here
    // contains at most 4x4=16 "on" pixels (spacing 2 in both axes).
    for (int y = 0; y < 32; y += 2) {
        for (int x = 0; x < 32; x += 2) {
            mask[y * W + x] = 1;
        }
    }
    // Compact blob: solid 6x6 block -- an 8x8 window can fully contain it,
    // scoring 36 "on" pixels, beating the diffuse region's best of 16.
    for (int y = 33; y < 39; y++) {
        for (int x = 33; x < 39; x++) {
            mask[y * W + x] = 1;
        }
    }

    uint16_t rowSumScratch[W * H];
    float cx, cy;
    bool found = EyeBlinkEAR::findDarkestWindow(mask, W, H, 8, 1, 1, rowSumScratch, cx, cy);

    TEST_ASSERT_TRUE(found);
    // Centroid should land inside/near the compact blob (33..39), not the
    // diffuse region (0..32).
    TEST_ASSERT_TRUE(cx > 30.0f);
    TEST_ASSERT_TRUE(cy > 30.0f);
}

void test_darkest_window_aggregates_fragmented_speckles(void) {
    const int W = 30, H = 30;
    uint8_t mask[W * H];
    memset(mask, 0, sizeof(mask));

    // Three small disconnected fragments close together (simulating JPEG
    // noise breaking up a real pupil into pieces) -- no other dark pixels
    // anywhere else in the buffer.
    mask[10 * W + 10] = 1;
    mask[11 * W + 13] = 1;
    mask[14 * W + 11] = 1;
    mask[14 * W + 12] = 1;

    uint16_t rowSumScratch[W * H];
    float cx, cy;
    bool found = EyeBlinkEAR::findDarkestWindow(mask, W, H, 10, 1, 1, rowSumScratch, cx, cy);

    TEST_ASSERT_TRUE(found);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 11.5f, cx);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 12.0f, cy);
}

void test_darkest_window_empty_mask_fails(void) {
    const int W = 20, H = 20;
    uint8_t mask[W * H];
    memset(mask, 0, sizeof(mask));

    uint16_t rowSumScratch[W * H];
    float cx, cy;
    bool found = EyeBlinkEAR::findDarkestWindow(mask, W, H, 6, 1, 1, rowSumScratch, cx, cy);

    TEST_ASSERT_FALSE(found);
}

void test_isolate_largest_component_keeps_only_the_bigger_blob(void) {
    const int W = 20, H = 20;
    uint8_t mask[W * H];
    memset(mask, 0, sizeof(mask));

    // Small 2x2 speck (4 px) -- an eyelash fragment / shadow speck.
    mask[2 * W + 2] = 1; mask[2 * W + 3] = 1;
    mask[3 * W + 2] = 1; mask[3 * W + 3] = 1;

    // Larger 4x4 blob (16 px) elsewhere, not touching the speck -- the pupil.
    for (int y = 10; y < 14; y++)
        for (int x = 10; x < 14; x++)
            mask[y * W + x] = 1;

    int16_t labelScratch[W * H];
    uint16_t queueScratch[W * H];
    uint8_t outMask[W * H];
    bool found = EyeBlinkEAR::isolateLargestComponent(mask, W, H, labelScratch, queueScratch, outMask);
    TEST_ASSERT_TRUE(found);

    // The speck must be gone entirely.
    TEST_ASSERT_EQUAL_UINT8(0, outMask[2 * W + 2]);
    TEST_ASSERT_EQUAL_UINT8(0, outMask[3 * W + 3]);

    // The larger blob must survive intact.
    int keptCount = 0;
    for (int y = 10; y < 14; y++)
        for (int x = 10; x < 14; x++)
            if (outMask[y * W + x]) keptCount++;
    TEST_ASSERT_EQUAL_INT(16, keptCount);

    // Nothing else got turned on.
    int totalOn = 0;
    for (int i = 0; i < W * H; i++) if (outMask[i]) totalOn++;
    TEST_ASSERT_EQUAL_INT(16, totalOn);
}

// ── Glint blink detector: the precision-first guarantees ─────────────────
void test_glint_count_counts_specular_pixels(void) {
    const int W = 10, H = 10;
    uint8_t gray[W * H];
    memset(gray, 50, sizeof(gray));      // dim background
    gray[0] = 255; gray[1] = 220; gray[2] = 199;  // 2 at/above 200, 1 just below
    TEST_ASSERT_EQUAL_INT(2, EyeBlinkEAR::countGlintPixels(gray, W, H, 200));
}

void test_glint_blink_counted_on_recovery(void) {
    EyeBlinkEAR::GlintBlinkDetector d;
    TEST_ASSERT_FALSE(d.update(40, 50, 1000, 1000));   // open
    TEST_ASSERT_FALSE(d.update(0, 90, 100, 1100));   // closed 1 (brighter)
    TEST_ASSERT_FALSE(d.update(0, 90, 100, 1200));   // closed 2 -> run is now valid
    TEST_ASSERT_TRUE (d.update(40, 50, 1000, 1300));   // reopened -> blink counted here
}

void test_glint_single_frame_dropout_is_ignored(void) {
    // The exact ambiguous case the >=2 rule exists to reject.
    EyeBlinkEAR::GlintBlinkDetector d;
    d.update(40, 50, 1000, 1000);
    TEST_ASSERT_FALSE(d.update(0, 90, 100, 1100));   // one absent frame only
    TEST_ASSERT_FALSE(d.update(40, 50, 1000, 1200));   // back -> must NOT count
}

void test_glint_gone_without_brightening_is_not_a_blink(void) {
    // The false-positive case the second cue exists to kill: the reflection
    // leaves the crop (or is occluded) while the eye stays OPEN, so the dark
    // iris still fills the region and it never brightens.
    EyeBlinkEAR::GlintBlinkDetector d;
    for (int i = 0; i < 10; i++) d.update(40, 50, 1000, i * 100);  // open, learn baseline 50
    TEST_ASSERT_FALSE(d.update(0, 50, 1000, 2000));   // glint gone, brightness UNCHANGED
    TEST_ASSERT_FALSE(d.update(0, 51, 1000, 2100));   // still no brightening
    TEST_ASSERT_FALSE(d.update(40, 50, 1000, 2200));  // returns -> must NOT count as a blink
}

void test_glint_gaze_shift_is_not_a_blink(void) {
    // The session_067 false positive: the eye ROLLS sideways. The glint
    // leaves the ROI and bright sclera raises brightness, so cues 1 and 2
    // both pass -- but the dark pupil is still there, just moved. Cue 3
    // must veto it.
    EyeBlinkEAR::GlintBlinkDetector d;
    for (int i = 0; i < 20; i++) d.update(40, 50, 1000, i * 100);  // open: baselines settle
    TEST_ASSERT_FALSE(d.update(0, 80, 900, 3000));   // glint gone, brighter, pupil STILL present
    TEST_ASSERT_FALSE(d.update(0, 80, 880, 3100));
    TEST_ASSERT_FALSE(d.update(40, 50, 1000, 3200)); // returns -> must NOT count
}

void test_glint_long_absence_is_not_a_blink(void) {
    // Covered lens / darkness / eye out of frame must read as "no data".
    EyeBlinkEAR::GlintBlinkDetector d;
    d.update(40, 50, 1000, 0);
    for (int i = 0; i < 40; i++) d.update(0, 90, 100, 100 + i * 100);
    TEST_ASSERT_FALSE(d.update(40, 50, 1000, 9000));   // recovery after a long gap: not a blink
}

void test_glint_rolling_rate_evicts_old_blinks(void) {
    EyeBlinkEAR::GlintBlinkDetector d;
    d.update(40, 50, 1000, 1000);
    d.update(0, 90, 100, 1100); d.update(0, 90, 100, 1200);
    TEST_ASSERT_TRUE(d.update(40, 50, 1000, 1300));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, d.rollingRateBpm(2000));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d.rollingRateBpm(90000));  // aged out of the 60s window
}

void test_isolate_largest_component_empty_mask_fails(void) {
    const int W = 12, H = 12;
    uint8_t mask[W * H];
    memset(mask, 0, sizeof(mask));

    int16_t labelScratch[W * H];
    uint16_t queueScratch[W * H];
    uint8_t outMask[W * H];
    bool found = EyeBlinkEAR::isolateLargestComponent(mask, W, H, labelScratch, queueScratch, outMask);

    TEST_ASSERT_FALSE(found);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_otsu_separates_two_clusters);
    RUN_TEST(test_ear_round_blob_is_near_one);
    RUN_TEST(test_ear_squashed_blob_is_near_zero);
    RUN_TEST(test_ear_empty_mask_is_invalid);
    RUN_TEST(test_centroid_of_offset_blob);
    RUN_TEST(test_centroid_of_empty_mask_fails);
    RUN_TEST(test_lock_roi_ignores_outlier_via_median);
    RUN_TEST(test_lock_roi_clamps_to_frame_bounds);
    RUN_TEST(test_drift_blend_accepts_small_shift);
    RUN_TEST(test_drift_blend_rejects_large_shift);
    RUN_TEST(test_baseline_is_75th_percentile);
    RUN_TEST(test_blink_detected_on_dip_and_recovery);
    RUN_TEST(test_cooldown_prevents_double_count);
    RUN_TEST(test_rolling_rate_evicts_old_blinks);
    RUN_TEST(test_percentile_threshold_isolates_darkest_fraction);
    RUN_TEST(test_darkest_window_prefers_compact_blob_over_diffuse_region);
    RUN_TEST(test_darkest_window_aggregates_fragmented_speckles);
    RUN_TEST(test_darkest_window_empty_mask_fails);
    RUN_TEST(test_isolate_largest_component_keeps_only_the_bigger_blob);
    RUN_TEST(test_isolate_largest_component_empty_mask_fails);
    RUN_TEST(test_glint_count_counts_specular_pixels);
    RUN_TEST(test_glint_blink_counted_on_recovery);
    RUN_TEST(test_glint_single_frame_dropout_is_ignored);
    RUN_TEST(test_glint_gone_without_brightening_is_not_a_blink);
    RUN_TEST(test_glint_gaze_shift_is_not_a_blink);
    RUN_TEST(test_glint_long_absence_is_not_a_blink);
    RUN_TEST(test_glint_rolling_rate_evicts_old_blinks);
    return UNITY_END();
}
