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

// Threshold that isolates the darkest `percentile` percent of pixels (e.g.
// 10.0 for the darkest 10%), rather than Otsu's variance-maximizing ~50/50
// split. Used for pupil localization (findDarkestWindow's input mask), where
// we specifically want only the most extreme dark pixels -- the pupil is
// usually much darker than surrounding shadow/skin -- not a generic
// foreground/background split. Otsu's wider net was found (via retrospective
// validation against a real session) to let a large diffuse shadow region
// dominate a small genuinely-dark pupil.
inline uint8_t percentileThreshold(const uint8_t *gray, int w, int h, float percentile) {
    uint32_t hist[256] = {0};
    int total = w * h;
    for (int i = 0; i < total; i++) hist[gray[i]]++;

    if (percentile < 0.0f) percentile = 0.0f;
    if (percentile > 100.0f) percentile = 100.0f;
    long targetCount = (long)((percentile / 100.0f) * (float)total);

    long cumulative = 0;
    for (int t = 0; t < 256; t++) {
        cumulative += hist[t];
        if (cumulative >= targetCount) return (uint8_t)t;
    }
    return 255;
}

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

// Finds the winSize x winSize window with the highest count of "on" pixels
// (the densest concentration of below-threshold pixels) within a mask, using
// a separable two-pass box filter (row sums, then column sums) instead of a
// full summed-area table -- every intermediate sum stays bounded by
// winSize*winSize, so a 16-bit scratch buffer is enough even for a
// full-frame-sized search region, with no wide (32-bit) integral image
// needed.
//
// This is more fragmentation-tolerant than a bare centroid() call: a real
// pupil that breaks into a few small disconnected speckles (JPEG noise)
// still scores highly as long as the speckles fall within one window,
// without needing connected-component labeling (which would need
// unbounded-size queues/label buffers to implement safely on a
// microcontroller). It also naturally prefers a small compact dark region
// (a pupil) over a larger, more diffuse dark region (a shadow) of similar
// or even lower average darkness, since the diffuse region rarely packs as
// many "on" pixels into one small window as a solid compact blob does.
//
// rowSumScratch must be caller-provided, at least w*h uint16_t entries.
// strideX/strideY (>=1) skip candidate window positions to trade thoroughness
// for speed -- 1 checks every position, larger values check fewer.
// Returns false if no window has any "on" pixels at all, or if winSize
// doesn't fit within w/h.
inline bool findDarkestWindow(const uint8_t *mask, int w, int h, int winSize,
                               int strideX, int strideY,
                               uint16_t *rowSumScratch,
                               float &outCx, float &outCy) {
    if (winSize <= 0 || winSize > w || winSize > h) return false;
    if (strideX < 1) strideX = 1;
    if (strideY < 1) strideY = 1;

    // Pass 1: rowSumScratch[y*w + x] = sum of mask[y][x .. x+winSize)
    for (int y = 0; y < h; y++) {
        uint16_t sum = 0;
        for (int x = 0; x < winSize; x++) sum = (uint16_t)(sum + mask[y * w + x]);
        rowSumScratch[y * w + 0] = sum;
        for (int x = 1; x <= w - winSize; x++) {
            sum = (uint16_t)(sum - mask[y * w + (x - 1)] + mask[y * w + (x - 1 + winSize)]);
            rowSumScratch[y * w + x] = sum;
        }
    }

    // Pass 2: slide winSize-tall column sums over the row sums, tracking the
    // best (x, y) top-left window position seen.
    int lastX = w - winSize;
    int lastY = h - winSize;
    int bestX = -1, bestY = -1;
    uint32_t bestSum = 0;

    for (int x = 0; x <= lastX; x += strideX) {
        uint32_t colSum = 0;
        for (int y = 0; y < winSize; y++) colSum += rowSumScratch[y * w + x];
        if (colSum > bestSum) { bestSum = colSum; bestX = x; bestY = 0; }
        for (int y = 1; y <= lastY; y++) {
            colSum = colSum - rowSumScratch[(y - 1) * w + x] + rowSumScratch[(y - 1 + winSize) * w + x];
            if ((y % strideY) == 0 && colSum > bestSum) { bestSum = colSum; bestX = x; bestY = y; }
        }
    }

    if (bestX < 0 || bestSum == 0) return false;

    // Refine: centroid of the "on" pixels within the winning window only
    // (full-buffer coordinates), not the whole window's geometric center.
    double sx = 0, sy = 0;
    int cnt = 0;
    for (int y = 0; y < winSize; y++) {
        for (int x = 0; x < winSize; x++) {
            if (mask[(bestY + y) * w + (bestX + x)]) {
                sx += bestX + x;
                sy += bestY + y;
                cnt++;
            }
        }
    }
    if (cnt == 0) return false;

    outCx = (float)(sx / cnt);
    outCy = (float)(sy / cnt);
    return true;
}

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

}  // namespace EyeBlinkEAR
