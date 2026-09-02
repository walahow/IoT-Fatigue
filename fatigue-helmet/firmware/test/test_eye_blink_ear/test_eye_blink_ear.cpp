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
    return UNITY_END();
}
