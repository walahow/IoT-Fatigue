# On-Device Blink Detection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the ESP32-S3 compute a real, live blink rate from its own camera feed (no PC required) and feed it into the existing on-device `FuzzyFatigue` risk engine, replacing the hardcoded `13.0` stub.

**Architecture:** A new header-only, hardware-independent module (`EyeBlinkEAR.h`) implements Otsu thresholding, image-moment-based EAR (eigenvalue ratio of the thresholded eye blob, standing in for a full ellipse fit), a blink/cooldown state machine with a 60-second rolling rate, and ROI centroid/lock/drift helpers — all pure functions over plain pixel buffers, unit-tested on the host via PlatformIO's `native` platform. A thin integration layer in `main.cpp` decodes the JPEG frame already captured for SD/dataset use (via `esp32-camera`'s `fmt2rgb888()`), crops/converts small regions to grayscale, and calls into the pure module. This is additive to `esp32s3cam_sd`/`esp32s3cam_test_sd` — `esp32s3cam` (USB debug mode) and every existing SD file format are untouched.

**Tech Stack:** PlatformIO (`espressif32` + `native` platforms), Arduino-ESP32 core, `esp32-camera` (`img_converters.h`), Unity test framework (bundled with PlatformIO's native test runner), C++14.

**Spec:** `docs/superpowers/specs/2026-09-01-on-device-fatigue-detection-design.md` §5 (On-device EAR pipeline).

---

## File structure

- **Create:** `fatigue-helmet/firmware/src/EyeBlinkEAR.h` — pure math module (Otsu threshold, moments/EAR, blink detector, ROI centroid/lock/drift). No Arduino or esp32-camera types. Built incrementally across Tasks 1-6.
- **Create:** `fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp` — host-side Unity tests for every function in `EyeBlinkEAR.h`. Built incrementally alongside it.
- **Modify:** `fatigue-helmet/firmware/platformio.ini` — add `[env:native]` for host testing (Task 0).
- **Modify:** `fatigue-helmet/firmware/src/main.cpp` — includes, EAR global state + `processEarFrame()`, the `cameraTask()` call site, and the `g_blinkRate` fallback (Task 7, all on-hardware/build-verified only — this glue code cannot be unit tested on the host since it depends on real `esp32-camera` types).

---

### Task 0: Native test environment

**Files:**
- Modify: `fatigue-helmet/firmware/platformio.ini`

- [ ] **Step 1: Add a native platform environment for host-side unit testing**

Append to `fatigue-helmet/firmware/platformio.ini`:

```ini
[env:native]
; ── Host-side unit tests for hardware-independent modules (EyeBlinkEAR.h) ──
; Run with: pio test -e native -f test_eye_blink_ear -v
platform = native
build_flags = -std=gnu++14
```

- [ ] **Step 2: Verify PlatformIO recognizes the new environment**

Run: `cd fatigue-helmet/firmware && pio project config`
Expected: prints config for all environments including `env:native` (platform, build_flags) with no errors. (`pio project config` doesn't take an `-e` filter; `pio run -e native` is an equally valid way to confirm PlatformIO accepts the env.)

- [ ] **Step 3: Commit**

```bash
git add fatigue-helmet/firmware/platformio.ini
git commit -m "$(cat <<'EOF'
Add native PlatformIO env for host-side firmware unit tests

Lets the new EyeBlinkEAR module be unit tested on the host instead of
needing real ESP32-S3 hardware for its pure math.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 1: Otsu threshold

**Files:**
- Create: `fatigue-helmet/firmware/src/EyeBlinkEAR.h`
- Create: `fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp`

- [ ] **Step 1: Write the failing test**

Create `fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp`:

```cpp
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

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_otsu_separates_two_clusters);
    return UNITY_END();
}
```

- [ ] **Step 2: Create the (empty) header and run the test to verify it fails**

Create `fatigue-helmet/firmware/src/EyeBlinkEAR.h`:

```cpp
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

// Pure, hardware-independent eye-blink detection math: Otsu thresholding,
// image-moment-based EAR, a blink/cooldown state machine with a 60s
// rolling rate, and ROI centroid/lock/drift helpers. No Arduino or
// esp32-camera types are used here, so this is unit tested on the host --
// see fatigue-helmet/firmware/test/test_eye_blink_ear/.
namespace EyeBlinkEAR {

}  // namespace EyeBlinkEAR
```

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: FAIL to compile with `'otsuThreshold' is not a member of 'EyeBlinkEAR'`

- [ ] **Step 3: Implement `otsuThreshold`**

Replace the empty namespace body in `EyeBlinkEAR.h` with:

```cpp
namespace EyeBlinkEAR {

// Standard between-class-variance-maximizing Otsu threshold over an 8-bit
// grayscale buffer of w*h pixels (row-major, one byte per pixel).
inline uint8_t otsuThreshold(const uint8_t *gray, int w, int h) {
    uint32_t hist[256] = {0};
    int total = w * h;
    for (int i = 0; i < total; i++) hist[gray[i]]++;

    float sum = 0.0f;
    for (int t = 0; t < 256; t++) sum += (float)t * (float)hist[t];

    float sumB = 0.0f;
    uint32_t wB = 0;
    float maxVar = 0.0f;
    uint8_t threshold = 0;

    for (int t = 0; t < 256; t++) {
        wB += hist[t];
        if (wB == 0) continue;
        uint32_t wF = (uint32_t)total - wB;
        if (wF == 0) break;

        sumB += (float)t * (float)hist[t];
        float mB = sumB / (float)wB;
        float mF = (sum - sumB) / (float)wF;
        float varBetween = (float)wB * (float)wF * (mB - mF) * (mB - mF);
        if (varBetween > maxVar) {
            maxVar = varBetween;
            threshold = (uint8_t)t;
        }
    }
    return threshold;
}

}  // namespace EyeBlinkEAR
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: `test_otsu_separates_two_clusters:PASSED`, and `1 Tests 0 Failures 0 Ignored`

- [ ] **Step 5: Commit**

```bash
git add fatigue-helmet/firmware/src/EyeBlinkEAR.h fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp
git commit -m "$(cat <<'EOF'
Add Otsu threshold to EyeBlinkEAR with a host-side unit test

First piece of the on-device EAR pipeline (spec section 5.1): pure,
hardware-independent thresholding, verified without needing real ESP32
hardware.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: EAR via image moments (eigenvalue ratio)

**Files:**
- Modify: `fatigue-helmet/firmware/src/EyeBlinkEAR.h`
- Modify: `fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp`

- [ ] **Step 1: Write the failing tests**

Add to `test_eye_blink_ear.cpp`, above `int main(...)`:

```cpp
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
```

Add the three `RUN_TEST(...)` calls into `main()`, before `return UNITY_END();`:

```cpp
    RUN_TEST(test_ear_round_blob_is_near_one);
    RUN_TEST(test_ear_squashed_blob_is_near_zero);
    RUN_TEST(test_ear_empty_mask_is_invalid);
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: FAIL to compile with `'EarResult' is not a member of 'EyeBlinkEAR'`

- [ ] **Step 3: Implement `EarResult` and `computeEAR`**

Add to `EyeBlinkEAR.h`, inside the `EyeBlinkEAR` namespace, after `otsuThreshold`:

```cpp
struct EarResult {
    bool valid;
    float ear;    // sqrt(lambda_minor) / sqrt(lambda_major), 0..1
    float major;  // sqrt(lambda_major), for diagnostics
    float minor;  // sqrt(lambda_minor)
};

// EAR from second-order image moments of a binary mask (w*h, nonzero =
// "on"/blob pixel) -- the minor/major axis ratio of the ellipse with the
// same second moments as the blob. Equivalent to fitting an ellipse to
// the blob's contour, without needing contour tracing at all.
inline EarResult computeEAR(const uint8_t *mask, int w, int h) {
    EarResult r = {false, 0.0f, 0.0f, 0.0f};

    double m00 = 0, m10 = 0, m01 = 0;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (mask[y * w + x]) {
                m00 += 1.0;
                m10 += x;
                m01 += y;
            }
        }
    }
    if (m00 < 1.0) return r;  // empty mask

    double cx = m10 / m00;
    double cy = m01 / m00;

    double mu20 = 0, mu02 = 0, mu11 = 0;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (mask[y * w + x]) {
                double dx = x - cx;
                double dy = y - cy;
                mu20 += dx * dx;
                mu02 += dy * dy;
                mu11 += dx * dy;
            }
        }
    }
    mu20 /= m00;
    mu02 /= m00;
    mu11 /= m00;

    double trace = mu20 + mu02;
    double diff = sqrt((mu20 - mu02) * (mu20 - mu02) + 4.0 * mu11 * mu11);
    double lambdaMajor = (trace + diff) / 2.0;
    double lambdaMinor = (trace - diff) / 2.0;
    if (lambdaMinor < 0.0) lambdaMinor = 0.0;

    r.major = (float)sqrt(lambdaMajor);
    r.minor = (float)sqrt(lambdaMinor);
    r.valid = true;
    r.ear = (r.major > 1e-6f) ? (r.minor / r.major) : 0.0f;
    return r;
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: all 4 tests `PASSED`, `4 Tests 0 Failures 0 Ignored`

- [ ] **Step 5: Commit**

```bash
git add fatigue-helmet/firmware/src/EyeBlinkEAR.h fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp
git commit -m "$(cat <<'EOF'
Add moment-based EAR computation to EyeBlinkEAR

Ports eye_ear.py's ellipse-fit EAR to plain C via image moments (spec
5.1) instead of contour tracing -- one pixel pass, a closed-form 2x2
eigenvalue solve, no contour/ellipse-fit library needed. Verified a
round blob reads near EAR=1 and a squashed one reads near EAR=0.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 3: Centroid helper

**Files:**
- Modify: `fatigue-helmet/firmware/src/EyeBlinkEAR.h`
- Modify: `fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp`

- [ ] **Step 1: Write the failing tests**

Add to `test_eye_blink_ear.cpp`, above `int main(...)`:

```cpp
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
```

Add to `main()`:

```cpp
    RUN_TEST(test_centroid_of_offset_blob);
    RUN_TEST(test_centroid_of_empty_mask_fails);
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: FAIL to compile with `'centroid' is not a member of 'EyeBlinkEAR'`

- [ ] **Step 3: Implement `centroid`**

Add to `EyeBlinkEAR.h`, after `computeEAR`:

```cpp
// Centroid (center of mass) of the "on" pixels in a mask. Returns false
// if the mask is empty. Coordinates are local to the given buffer (0..w,
// 0..h) -- the caller maps them back to full-frame coordinates if the
// buffer represents a sub-region.
inline bool centroid(const uint8_t *mask, int w, int h, float &outCx, float &outCy) {
    double m00 = 0, m10 = 0, m01 = 0;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (mask[y * w + x]) {
                m00 += 1.0;
                m10 += x;
                m01 += y;
            }
        }
    }
    if (m00 < 1.0) return false;
    outCx = (float)(m10 / m00);
    outCy = (float)(m01 / m00);
    return true;
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: all 6 tests `PASSED`, `6 Tests 0 Failures 0 Ignored`

- [ ] **Step 5: Commit**

```bash
git add fatigue-helmet/firmware/src/EyeBlinkEAR.h fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp
git commit -m "$(cat <<'EOF'
Add centroid helper to EyeBlinkEAR

Shared building block for both the boot-time ROI lock and the runtime
drift check (spec 5.4) -- center of mass of a thresholded mask.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 4: ROI boot-lock (median of samples)

**Files:**
- Modify: `fatigue-helmet/firmware/src/EyeBlinkEAR.h`
- Modify: `fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp`

- [ ] **Step 1: Write the failing tests**

Add to `test_eye_blink_ear.cpp`, above `int main(...)`:

```cpp
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
```

Add to `main()`:

```cpp
    RUN_TEST(test_lock_roi_ignores_outlier_via_median);
    RUN_TEST(test_lock_roi_clamps_to_frame_bounds);
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: FAIL to compile with `'RoiLock' is not a member of 'EyeBlinkEAR'`

- [ ] **Step 3: Implement `RoiLock` and `lockRoiFromSamples`**

Add to `EyeBlinkEAR.h`, after `centroid`:

```cpp
struct RoiLock {
    int x = 0;
    int y = 0;
    int size = 64;
    bool locked = false;
};

// Median-based ROI lock: xs/ys are full-frame-coordinate centroid samples
// gathered across several frames (some may be bad single-frame misreads --
// the median absorbs those, unlike a mean). Mutates xs/ys in place (sorts
// them) -- pass copies, not shared state. Clamps the resulting ROI to stay
// within [0, frameW) x [0, frameH).
inline bool lockRoiFromSamples(float *xs, float *ys, int n, int roiSize,
                                int frameW, int frameH, RoiLock &out) {
    if (n <= 0) return false;

    for (int i = 1; i < n; i++) {
        float kx = xs[i];
        int j = i - 1;
        while (j >= 0 && xs[j] > kx) { xs[j + 1] = xs[j]; j--; }
        xs[j + 1] = kx;
    }
    for (int i = 1; i < n; i++) {
        float ky = ys[i];
        int j = i - 1;
        while (j >= 0 && ys[j] > ky) { ys[j + 1] = ys[j]; j--; }
        ys[j + 1] = ky;
    }
    float medCx = xs[n / 2];
    float medCy = ys[n / 2];

    int rx = (int)(medCx - roiSize / 2.0f);
    int ry = (int)(medCy - roiSize / 2.0f);
    if (rx + roiSize > frameW) rx = frameW - roiSize;
    if (ry + roiSize > frameH) ry = frameH - roiSize;
    if (rx < 0) rx = 0;
    if (ry < 0) ry = 0;

    out.x = rx;
    out.y = ry;
    out.size = roiSize;
    out.locked = true;
    return true;
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: all 8 tests `PASSED`, `8 Tests 0 Failures 0 Ignored`

- [ ] **Step 5: Commit**

```bash
git add fatigue-helmet/firmware/src/EyeBlinkEAR.h fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp
git commit -m "$(cat <<'EOF'
Add median-based ROI boot lock to EyeBlinkEAR

Spec 5.4's boot-lock mechanism: median of N frame centroids, robust to
an occasional bad single-frame detection -- mirrors eye_ear.py's
median-of-30-Haar-detections lock, without needing Haar/ML on-device.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: Runtime drift correction

**Files:**
- Modify: `fatigue-helmet/firmware/src/EyeBlinkEAR.h`
- Modify: `fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp`

- [ ] **Step 1: Write the failing tests**

Add to `test_eye_blink_ear.cpp`, above `int main(...)`:

```cpp
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
```

Add to `main()`:

```cpp
    RUN_TEST(test_drift_blend_accepts_small_shift);
    RUN_TEST(test_drift_blend_rejects_large_shift);
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: FAIL to compile with `'driftBlend' is not a member of 'EyeBlinkEAR'`

- [ ] **Step 3: Implement `driftBlend`**

Add to `EyeBlinkEAR.h`, after `lockRoiFromSamples`:

```cpp
// Given the ROI's current full-frame center and a freshly-measured
// full-frame centroid, decides whether to nudge the ROI. Returns false
// (caller should leave the ROI alone) if the shift is large enough to
// look like noise (a blink, a shadow) rather than real drift -- otherwise
// outCx/outCy are the EMA-blended new center for the caller to clamp and
// apply.
inline bool driftBlend(float curCx, float curCy, float newCx, float newCy,
                        float maxDriftPx, float alpha,
                        float &outCx, float &outCy) {
    float dx = newCx - curCx;
    float dy = newCy - curCy;
    float shift = sqrtf(dx * dx + dy * dy);
    if (shift >= maxDriftPx) return false;

    outCx = curCx * (1.0f - alpha) + newCx * alpha;
    outCy = curCy * (1.0f - alpha) + newCy * alpha;
    return true;
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: all 10 tests `PASSED`, `10 Tests 0 Failures 0 Ignored`

- [ ] **Step 5: Commit**

```bash
git add fatigue-helmet/firmware/src/EyeBlinkEAR.h fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp
git commit -m "$(cat <<'EOF'
Add runtime drift correction to EyeBlinkEAR

Spec 5.4's drift-check mechanism: small shifts get EMA-blended in,
large jumps are ignored as noise -- mirrors eye_ear.py's
DRIFT_MAX_PX-gated Haar recheck, without needing Haar/ML on-device.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 6: Blink detector (baseline, cooldown, rolling rate)

**Files:**
- Modify: `fatigue-helmet/firmware/src/EyeBlinkEAR.h`
- Modify: `fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp`

- [ ] **Step 1: Write the failing tests**

Add to `test_eye_blink_ear.cpp`, above `int main(...)`:

```cpp
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
```

Add to `main()`:

```cpp
    RUN_TEST(test_baseline_is_75th_percentile);
    RUN_TEST(test_blink_detected_on_dip_and_recovery);
    RUN_TEST(test_cooldown_prevents_double_count);
    RUN_TEST(test_rolling_rate_evicts_old_blinks);
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: FAIL to compile with `'BlinkDetector' is not a member of 'EyeBlinkEAR'`

- [ ] **Step 3: Implement `BlinkDetector`**

Add to `EyeBlinkEAR.h`, after `driftBlend`:

```cpp
// Adaptive-baseline blink detector + 60s rolling rate. Mirrors
// eye_ear.py's blink logic: baseline = 75th percentile of recent EAR,
// blink fires when EAR drops below baseline * EAR_RATIO_THRESH, gated by
// a warm-up period and a cooldown so one blink isn't counted twice.
struct BlinkDetector {
    static const int BASELINE_WINDOW = 30;
    static const int COOLDOWN_SAMPLES = 6;
    static const int WARMUP_SAMPLES = 24;
    static constexpr float EAR_RATIO_THRESH = 0.70f;
    static const int MAX_BLINKS_TRACKED = 60;

    float history[BASELINE_WINDOW] = {0};
    int historyCount = 0;
    int historyHead = 0;
    int sampleIndex = 0;
    int cooldownRemaining = 0;
    bool inBlinkRun = false;

    uint32_t blinkTimestamps[MAX_BLINKS_TRACKED] = {0};
    int blinkCount = 0;

    float baseline() const {
        if (historyCount == 0) return 1.0f;
        float sorted[BASELINE_WINDOW];
        int n = historyCount;
        memcpy(sorted, history, n * sizeof(float));
        for (int i = 1; i < n; i++) {
            float key = sorted[i];
            int j = i - 1;
            while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
            sorted[j + 1] = key;
        }
        int idx = (int)(n * 0.75f);
        if (idx >= n) idx = n - 1;
        return sorted[idx];
    }

    void pushBlinkTimestamp(uint32_t nowMs) {
        if (blinkCount < MAX_BLINKS_TRACKED) {
            blinkTimestamps[blinkCount++] = nowMs;
        } else {
            memmove(blinkTimestamps, blinkTimestamps + 1,
                    (MAX_BLINKS_TRACKED - 1) * sizeof(uint32_t));
            blinkTimestamps[MAX_BLINKS_TRACKED - 1] = nowMs;
        }
    }

    // Returns true if this sample completed a blink event.
    bool update(float ear, uint32_t nowMs) {
        sampleIndex++;
        float base = baseline();

        history[historyHead] = ear;
        historyHead = (historyHead + 1) % BASELINE_WINDOW;
        if (historyCount < BASELINE_WINDOW) historyCount++;

        bool blinkEvent = false;
        if (sampleIndex > WARMUP_SAMPLES) {
            if (cooldownRemaining > 0) {
                cooldownRemaining--;
            } else {
                bool trigger = ear < base * EAR_RATIO_THRESH;
                if (trigger && !inBlinkRun) {
                    inBlinkRun = true;
                } else if (!trigger && inBlinkRun) {
                    inBlinkRun = false;
                    blinkEvent = true;
                    cooldownRemaining = COOLDOWN_SAMPLES;
                    pushBlinkTimestamp(nowMs);
                }
            }
        }
        return blinkEvent;
    }

    // 60-second rolling rate in blinks/minute (one blink event per minute
    // of trailing window == that many blinks/min by definition).
    float rollingRateBpm(uint32_t nowMs) {
        int evict = 0;
        while (evict < blinkCount && (nowMs - blinkTimestamps[evict]) > 60000) evict++;
        if (evict > 0) {
            memmove(blinkTimestamps, blinkTimestamps + evict,
                    (blinkCount - evict) * sizeof(uint32_t));
            blinkCount -= evict;
        }
        return (float)blinkCount;
    }
};
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd fatigue-helmet/firmware && pio test -e native -f test_eye_blink_ear -v`
Expected: all 14 tests `PASSED`, `14 Tests 0 Failures 0 Ignored`

- [ ] **Step 5: Commit**

```bash
git add fatigue-helmet/firmware/src/EyeBlinkEAR.h fatigue-helmet/firmware/test/test_eye_blink_ear/test_eye_blink_ear.cpp
git commit -m "$(cat <<'EOF'
Add blink detector with baseline, cooldown, and rolling rate

Completes EyeBlinkEAR.h (spec 5.5): adaptive-baseline blink detection
and a 60s rolling blinks/minute rate, ported from eye_ear.py's logic.
The module is now fully unit tested on the host with no ESP32 hardware
needed -- 14 tests covering thresholding, EAR, ROI lock/drift, and
blink detection.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 7: Wire it into `main.cpp`

This task cannot be unit tested on the host (it uses real `esp32-camera` types).
Verification is by build success and is completed on real hardware in Task 8.

**Files:**
- Modify: `fatigue-helmet/firmware/src/main.cpp`

- [ ] **Step 1: Add includes**

In `fatigue-helmet/firmware/src/main.cpp`, find (around line 44-45):

```cpp
#include <math.h>
#include "FuzzyFatigue.h"  // Mamdani FIS (heap-free, STL-free, header-only)
```

Replace with:

```cpp
#include <math.h>
#include "FuzzyFatigue.h"  // Mamdani FIS (heap-free, STL-free, header-only)

#if defined(STORAGE_MODE_SD)
#include "EyeBlinkEAR.h"        // on-device blink detection (spec section 5)
#include "img_converters.h"     // fmt2rgb888() -- esp32-camera
#include "esp_heap_caps.h"      // heap_caps_malloc() / MALLOC_CAP_SPIRAM
#endif
```

- [ ] **Step 2: Add EAR global state and `processEarFrame()`**

Find the closing `#endif` of the SD card globals block (around line 183):

```cpp
#if defined(STORAGE_MODE_SD)
SemaphoreHandle_t g_sdMutex = nullptr;
File g_csvFile;            // sensor_data.csv — kept open entire session
File g_mjpegFile;          // video.mjpeg     — kept open entire session
File g_idxFile;            // video.idx       — kept open entire session
uint32_t g_byteOffset = 0; // running byte offset into video.mjpeg
uint32_t g_frameIndex = 0; // monotonic frame counter
bool g_sdReady = false;
#endif
```

Immediately after that `#endif`, insert:

```cpp
// ── On-device EAR blink detection state (SD mode) ───────────────────────
// See docs/superpowers/specs/2026-09-01-on-device-fatigue-detection-design.md §5.
#if defined(STORAGE_MODE_SD)
static const int   EAR_THROTTLE_DIV   = 3;     // process every 3rd captured frame
static const int   EAR_ROI_SIZE       = 64;    // fixed square crop, pixels
static const int   EAR_LOCK_SAMPLES   = 30;    // processed frames used for boot lock
static const int   EAR_DRIFT_PERIOD   = 40;    // processed frames between drift checks
static const int   EAR_DRIFT_MARGIN   = 30;    // px added around the ROI when searching for drift
static const float EAR_DRIFT_MAX_PX   = 20.0f;
static const float EAR_DRIFT_ALPHA    = 0.3f;

static uint8_t *g_earRgbBuf  = nullptr;  // PSRAM, fullW*fullH*3 (decoded JPEG)
static uint8_t *g_earGrayBuf = nullptr;  // PSRAM, fullW*fullH, reused at partial size
static uint8_t *g_earMaskBuf = nullptr;  // PSRAM, fullW*fullH, reused at partial size
static int      g_earFullW   = 0;
static int      g_earFullH   = 0;

static uint32_t g_earFrameCounter = 0;
static uint32_t g_earDriftCounter = 0;

static float g_earLockSamplesX[EAR_LOCK_SAMPLES];
static float g_earLockSamplesY[EAR_LOCK_SAMPLES];
static int   g_earLockSampleCount = 0;
static bool  g_earLockDone = false;

EyeBlinkEAR::RoiLock      g_earRoi;
EyeBlinkEAR::BlinkDetector g_earBlink;
float g_onDeviceBlinkRate = 13.0f;  // read by the g_blinkRate fallback below

// Converts a rectangular region of an RGB888 buffer to grayscale (simple
// average of R,G,B) into a caller-provided buffer sized regionW*regionH.
// Out-of-frame coordinates clamp to the nearest edge.
static void earExtractGray(const uint8_t *rgb, int fullW, int fullH,
                            int regionX, int regionY, int regionW, int regionH,
                            uint8_t *outGray) {
  for (int ry = 0; ry < regionH; ry++) {
    int sy = regionY + ry;
    if (sy < 0) sy = 0;
    if (sy >= fullH) sy = fullH - 1;
    for (int rx = 0; rx < regionW; rx++) {
      int sx = regionX + rx;
      if (sx < 0) sx = 0;
      if (sx >= fullW) sx = fullW - 1;
      const uint8_t *px = rgb + ((size_t)sy * fullW + sx) * 3;
      outGray[ry * regionW + rx] = (uint8_t)(((int)px[0] + px[1] + px[2]) / 3);
    }
  }
}

// otsuThreshold() only cares about total pixel count, not 2D shape, so a
// flat n-pixel buffer can be passed as (n, 1) safely.
static void earThresholdToMask(const uint8_t *gray, uint8_t *mask, int n) {
  uint8_t t = EyeBlinkEAR::otsuThreshold(gray, n, 1);
  for (int i = 0; i < n; i++) mask[i] = (gray[i] <= t) ? 1 : 0;
}

void processEarFrame(camera_fb_t *fb, uint32_t timestampMs) {
  int w = (int)fb->width;
  int h = (int)fb->height;

  if (!g_earRgbBuf) {
    g_earRgbBuf  = (uint8_t *)heap_caps_malloc((size_t)w * h * 3, MALLOC_CAP_SPIRAM);
    g_earGrayBuf = (uint8_t *)heap_caps_malloc((size_t)w * h, MALLOC_CAP_SPIRAM);
    g_earMaskBuf = (uint8_t *)heap_caps_malloc((size_t)w * h, MALLOC_CAP_SPIRAM);
    g_earFullW = w;
    g_earFullH = h;
    if (!g_earRgbBuf || !g_earGrayBuf || !g_earMaskBuf) {
      Serial.println(F("#ERROR: EAR buffer alloc failed -- on-device blink detection disabled"));
      return;
    }
  }
  if (w != g_earFullW || h != g_earFullH) return;  // frame size changed mid-session, skip

  if (!fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, g_earRgbBuf)) {
    return;  // decode failed this frame, try again next throttled frame
  }

#if defined(EAR_ROI_MANUAL_X)
  if (!g_earRoi.locked) {
    g_earRoi.x = EAR_ROI_MANUAL_X;
    g_earRoi.y = EAR_ROI_MANUAL_Y;
    g_earRoi.size = EAR_ROI_SIZE;
    g_earRoi.locked = true;
    g_earLockDone = true;
    Serial.printf("#STATUS: EAR ROI manually pinned at x=%d y=%d size=%d\n",
                  g_earRoi.x, g_earRoi.y, g_earRoi.size);
  }
#endif

  if (!g_earLockDone) {
    // Search the middle 60% of the frame (not the whole frame, to reduce
    // the chance of the darkest-blob heuristic latching onto helmet
    // interior/hair shadow near the edges instead of the eye).
    int searchX = w / 5;
    int searchY = h / 5;
    int searchW = (3 * w) / 5;
    int searchH = (3 * h) / 5;

    earExtractGray(g_earRgbBuf, w, h, searchX, searchY, searchW, searchH, g_earGrayBuf);
    earThresholdToMask(g_earGrayBuf, g_earMaskBuf, searchW * searchH);

    float localCx, localCy;
    if (EyeBlinkEAR::centroid(g_earMaskBuf, searchW, searchH, localCx, localCy) &&
        g_earLockSampleCount < EAR_LOCK_SAMPLES) {
      g_earLockSamplesX[g_earLockSampleCount] = searchX + localCx;
      g_earLockSamplesY[g_earLockSampleCount] = searchY + localCy;
      g_earLockSampleCount++;
    }

    if (g_earLockSampleCount >= EAR_LOCK_SAMPLES) {
      EyeBlinkEAR::lockRoiFromSamples(g_earLockSamplesX, g_earLockSamplesY,
                                       g_earLockSampleCount, EAR_ROI_SIZE,
                                       w, h, g_earRoi);
      g_earLockDone = true;
      Serial.printf("#STATUS: EAR ROI locked at x=%d y=%d size=%d\n",
                    g_earRoi.x, g_earRoi.y, g_earRoi.size);
    }
    return;
  }

  if (++g_earDriftCounter % EAR_DRIFT_PERIOD == 0) {
    int dx = g_earRoi.x - EAR_DRIFT_MARGIN;
    int dy = g_earRoi.y - EAR_DRIFT_MARGIN;
    int dw = g_earRoi.size + 2 * EAR_DRIFT_MARGIN;
    int dh = g_earRoi.size + 2 * EAR_DRIFT_MARGIN;

    earExtractGray(g_earRgbBuf, w, h, dx, dy, dw, dh, g_earGrayBuf);
    earThresholdToMask(g_earGrayBuf, g_earMaskBuf, dw * dh);

    float localCx, localCy;
    if (EyeBlinkEAR::centroid(g_earMaskBuf, dw, dh, localCx, localCy)) {
      float newFullCx = dx + localCx;
      float newFullCy = dy + localCy;
      float curCx = g_earRoi.x + g_earRoi.size / 2.0f;
      float curCy = g_earRoi.y + g_earRoi.size / 2.0f;

      float blendedCx, blendedCy;
      if (EyeBlinkEAR::driftBlend(curCx, curCy, newFullCx, newFullCy,
                                   EAR_DRIFT_MAX_PX, EAR_DRIFT_ALPHA,
                                   blendedCx, blendedCy)) {
        int rx = (int)(blendedCx - g_earRoi.size / 2.0f);
        int ry = (int)(blendedCy - g_earRoi.size / 2.0f);
        if (rx + g_earRoi.size > w) rx = w - g_earRoi.size;
        if (ry + g_earRoi.size > h) ry = h - g_earRoi.size;
        if (rx < 0) rx = 0;
        if (ry < 0) ry = 0;
        g_earRoi.x = rx;
        g_earRoi.y = ry;
      }
    }
  }

  earExtractGray(g_earRgbBuf, w, h, g_earRoi.x, g_earRoi.y,
                 g_earRoi.size, g_earRoi.size, g_earGrayBuf);
  earThresholdToMask(g_earGrayBuf, g_earMaskBuf, g_earRoi.size * g_earRoi.size);

  EyeBlinkEAR::EarResult ear = EyeBlinkEAR::computeEAR(g_earMaskBuf, g_earRoi.size, g_earRoi.size);
  if (ear.valid) {
    g_earBlink.update(ear.ear, timestampMs);
    g_onDeviceBlinkRate = g_earBlink.rollingRateBpm(timestampMs);
  }
}
#endif  // STORAGE_MODE_SD
```

- [ ] **Step 3: Call `processEarFrame()` from `cameraTask()`**

Find (around line 599-609):

```cpp
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      if (fb->format == PIXFORMAT_JPEG) {
        uint32_t ts = (uint32_t)millis();

#if defined(STORAGE_MODE_SD)
        saveJpegToSD(fb->buf, fb->len, ts);
#elif defined(STORAGE_MODE_USB)
        sendJpegFrame(fb->buf, fb->len, ts);
#endif
        framesSent++;
      }
      esp_camera_fb_return(fb);
```

Replace with:

```cpp
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      if (fb->format == PIXFORMAT_JPEG) {
        uint32_t ts = (uint32_t)millis();

#if defined(STORAGE_MODE_SD)
        saveJpegToSD(fb->buf, fb->len, ts);
        if (++g_earFrameCounter % EAR_THROTTLE_DIV == 0) {
          processEarFrame(fb, ts);
        }
#elif defined(STORAGE_MODE_USB)
        sendJpegFrame(fb->buf, fb->len, ts);
#endif
        framesSent++;
      }
      esp_camera_fb_return(fb);
```

- [ ] **Step 4: Wire the FIS fallback to the on-device rate**

Find (around line 1132-1137):

```cpp
  // Fallback: sticky last-valid after timeout; 13.0 if pipeline never connected
  if (g_blinkEverRx && (now - g_lastBlinkRxTime > BLINK_TIMEOUT_MS)) {
    g_blinkRate = g_lastValidBlink;
  } else if (!g_blinkEverRx) {
    g_blinkRate = 13.0f;
  }
```

Replace with:

```cpp
  // Fallback: sticky last-valid after timeout; on-device EAR rate in SD
  // mode (see processEarFrame() above), or the 13.0 stub in USB mode
  // (unchanged -- USB debug mode still relies on the offline eye_ear.py
  // pipeline, see spec non-goals).
  if (g_blinkEverRx && (now - g_lastBlinkRxTime > BLINK_TIMEOUT_MS)) {
    g_blinkRate = g_lastValidBlink;
  } else if (!g_blinkEverRx) {
#if defined(STORAGE_MODE_SD)
    g_blinkRate = g_onDeviceBlinkRate;
#else
    g_blinkRate = 13.0f;
#endif
  }
```

- [ ] **Step 5: Build-verify**

Run: `cd fatigue-helmet/firmware && pio run -e esp32s3cam_sd`
Expected: `[SUCCESS]` with no compile errors.

Also verify the untouched USB debug env still builds (this task must not affect it):

Run: `cd fatigue-helmet/firmware && pio run -e esp32s3cam`
Expected: `[SUCCESS]` with no compile errors.

- [ ] **Step 6: Commit**

```bash
git add fatigue-helmet/firmware/src/main.cpp
git commit -m "$(cat <<'EOF'
Wire EyeBlinkEAR into main.cpp for SD-mode on-device blink detection

Decodes each throttled captured JPEG (fmt2rgb888), runs it through
EyeBlinkEAR's boot-lock/drift/EAR/blink pipeline, and feeds the
resulting rolling blink rate into the g_blinkRate fallback that the
fuzzy risk engine already consumes -- replacing the hardcoded 13.0
stub in SD mode. USB debug mode (esp32s3cam) is untouched.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 8: On-hardware bring-up and validation

Not unit-testable -- this is the spec's §8 "open validation items," checked
manually against real hardware. Requires the ESP32-S3 connected over USB
(`COM3` in prior sessions; substitute the actual port).

**Files:** none (verification only)

- [ ] **Step 1: Flash `esp32s3cam_sd` and reset**

```bash
cd fatigue-helmet/firmware
pio run -e esp32s3cam_sd --target upload --upload-port COM3
```

Expected: `[SUCCESS]`, and the board hard-resets automatically at the end of upload.

- [ ] **Step 2: Confirm the boot-time ROI lock fires**

Open a serial monitor at 115200 baud (`pio device monitor -p COM3 -b 115200`, or read the
first ~10 seconds of output). Insert or point the SD-mode camera at an eye/face if possible.

Expected: within the first few seconds, a line like:

```
#STATUS: EAR ROI locked at x=<N> y=<N> size=64
```

If this line never appears, `g_earLockSampleCount` never reached 30 -- check that the camera
is actually capturing frames (`#WARNING: Camera frame drops=...` would indicate a problem)
and that `EAR_THROTTLE_DIV`/search-window assumptions in Task 7 Step 2 match the physical setup.

- [ ] **Step 3: Confirm `blink_rate` is no longer the 13.0 stub**

After the lock message appears, watch the 1Hz status block (or the CSV row) continue for
~15-20 seconds without pressing the session button (the block prints unconditionally).

Expected: the `BLINK:` value in the status block (and the corresponding CSV column) is no
longer pinned at exactly `13.0` -- it should settle near whatever the actual blink rate is
(0 if no blinking is happening, since the rolling rate starts at 0 once real detection is
live, unlike the old stub default).

- [ ] **Step 4: Confirm a deliberate blink registers**

With the camera aimed at an open eye, blink deliberately several times over ~10-15 seconds,
then watch the `BLINK:` value over the next status ticks.

Expected: the rolling blink rate increases from 0 toward a nonzero blinks/minute value
shortly after blinking, and gradually decays back toward 0 over the following 60 seconds
if blinking stops (matches the 60s rolling-window eviction in `rollingRateBpm()`).

- [ ] **Step 5: Confirm SD recording still works, unaffected**

Press the GPIO 21 session button to start a recording, let it run ~10 seconds, press again
to stop, then check the card (or `#STATUS: Session closed` message) for the session folder.

Expected: `sessions/session_XXX/sensor_data.csv`, `video.mjpeg`, and `video.idx` are created
and populated exactly as before this plan's changes -- on-device EAR is additive, not a
replacement for the SD recording path.

- [ ] **Step 6: Note actual throttle/timing behavior for future tuning**

Watch for `#WARNING: Camera frame drops=...` messages during steps 2-5. If frame drops climb
noticeably compared to pre-change behavior, `EAR_THROTTLE_DIV` (currently 3, ~6-7Hz at 20fps
capture) may need to increase in `main.cpp` (Task 7 Step 2) to reduce EAR processing load.
This is expected tuning work per spec §8 item 1, not a sign Task 7 was done incorrectly --
record what you observe so it can inform that follow-up.

- [ ] **Step 7: Commit any tuning adjustments made during bring-up**

If Step 6 required changing `EAR_THROTTLE_DIV` or other constants in `main.cpp`:

```bash
git add fatigue-helmet/firmware/src/main.cpp
git commit -m "$(cat <<'EOF'
Tune on-device EAR throttle rate from hardware bring-up

<describe the specific observation that motivated the change, e.g.
"frame drops climbed under sustained EAR processing at div=3;
div=5 (~4Hz) eliminates them on this board.">

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

If no changes were needed, skip this step.

---

## Self-review notes

- **Spec coverage:** §5.1 (moments not contour) → Task 2. §5.2 (JPEG decode) → Task 7 Step 2
  (`fmt2rgb888`, verified against the actual installed header, not assumed). §5.3 (throttled
  cadence) → Task 7 Step 2 (`EAR_THROTTLE_DIV`) + Task 8 Step 6 (on-hardware tuning). §5.4
  (boot lock, drift correction, manual override) → Tasks 4, 5, and the `EAR_ROI_MANUAL_X`
  branch in Task 7 Step 2. §5.5 (blink + rolling rate) → Task 6. §5.6 (FIS integration,
  keeping the serial `BLINK:` override intact) → Task 7 Step 4. §5.7 (file placement,
  `esp32s3cam` untouched) → File structure section + Task 7 Step 5's dual build-verify.
  §7 ("what stays as-is") → verified explicitly by building both `esp32s3cam_sd` and
  `esp32s3cam` in Task 7 Step 5. §8 (open validation items) → Task 8 in full.
- **Not in this plan, by design:** spec §6 (PC live view) is a separate subsystem per the
  user's request and gets its own plan later -- it depends on this plan's `g_onDeviceBlinkRate`
  existing but adds no code to it.
- **Type/name consistency checked:** `EyeBlinkEAR::RoiLock`, `EyeBlinkEAR::BlinkDetector`,
  `EyeBlinkEAR::EarResult`, `g_onDeviceBlinkRate`, `g_earRoi`, `g_earBlink` are each defined
  once (Tasks 2-6, Task 7 Step 2) and referenced identically everywhere they're used later.
