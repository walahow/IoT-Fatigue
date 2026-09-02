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

}  // namespace EyeBlinkEAR
