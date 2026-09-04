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

// Isolates the single largest 4-connected component of "on" pixels in
// `mask` (w*h) into `outMask` (same size, caller-allocated, zeroed by this
// function). Everything outside that one component -- a stray eyelash
// fragment, a shadow speck, JPEG noise -- is dropped before EAR is computed
// on it.
//
// This is the fix for computeEAR() averaging in every thresholded pixel in
// the crop regardless of whether it's actually part of the same blob:
// retrospective validation (real footage, 2026-09-04) found isolating the
// blob first and THEN computing moments nearly doubles the achieved EAR
// drop on a real blink (min 0.291 -> 0.198 over a real session) versus
// computeEAR() run directly on the raw mask, for the same input pixels.
//
// Deliberately NOT full contour tracing + parametric ellipse fit (what the
// offline eye_ear.py pipeline does via cv2.fitEllipse): that goes further
// (min 0.062 on the same data) but needs a hand-written boundary tracer and
// a least-squares ellipse solver in C, and was shown on the same data to be
// occasionally unstable on small/partial contours (one frame mid-blink
// spiked back to 0.97). This is the cheaper, more stable middle step: same
// moment math already validated by computeEAR()'s own tests, just fed a
// cleaned-up mask first.
//
// Bounded by construction, unlike a full-frame connected-component search
// (which EyeBlinkEAR deliberately avoids elsewhere -- see findDarkestWindow's
// doc comment): this runs over the small, FIXED-size locked EAR crop, not a
// search region, so a plain BFS with a queue capacity of exactly w*h entries
// can never overflow -- each pixel is enqueued at most once.
//
// labelScratch and queueScratch must be caller-provided, >= w*h entries
// each (int16_t / uint16_t respectively). Returns false if the mask is
// empty (no "on" pixels at all).
inline bool isolateLargestComponent(const uint8_t *mask, int w, int h,
                                     int16_t *labelScratch, uint16_t *queueScratch,
                                     uint8_t *outMask) {
    int n = w * h;
    memset(labelScratch, 0, (size_t)n * sizeof(int16_t));
    memset(outMask, 0, (size_t)n);

    int16_t nextLabel = 0;
    int16_t bestLabel = -1;
    int bestSize = 0;
    static const int dx[4] = {-1, 1, 0, 0};
    static const int dy[4] = {0, 0, -1, 1};

    for (int start = 0; start < n; start++) {
        if (!mask[start] || labelScratch[start] != 0) continue;

        nextLabel++;
        int qHead = 0, qTail = 0;
        queueScratch[qTail++] = (uint16_t)start;
        labelScratch[start] = nextLabel;
        int compSize = 0;

        while (qHead < qTail) {
            uint16_t idx = queueScratch[qHead++];
            compSize++;
            int x = idx % w;
            int y = idx / w;
            for (int k = 0; k < 4; k++) {
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                int nidx = ny * w + nx;
                if (mask[nidx] && labelScratch[nidx] == 0) {
                    labelScratch[nidx] = nextLabel;
                    queueScratch[qTail++] = (uint16_t)nidx;
                }
            }
        }

        if (compSize > bestSize) {
            bestSize = compSize;
            bestLabel = nextLabel;
        }
    }

    if (bestLabel < 0) return false;
    for (int i = 0; i < n; i++) {
        if (labelScratch[i] == bestLabel) outMask[i] = 1;
    }
    return true;
}

// ── Corneal-glint blink detection ─────────────────────────────────────────
// Counts "specular" pixels (>= level) in a grayscale region. The corneal
// glint -- the small mirror reflection off the wet surface of the eye -- is
// present in essentially every open-eye frame and DISAPPEARS the moment the
// eyelid covers the cornea, because skin is diffuse and doesn't produce a
// specular highlight.
//
// This is the cheapest operation in this whole header: one compare and one
// increment per pixel. No Otsu, no moments, no contour tracing, no ellipse
// fit, no adaptive percentile baseline.
inline int countGlintPixels(const uint8_t *gray, int w, int h, uint8_t level) {
    int n = w * h;
    int count = 0;
    for (int i = 0; i < n; i++) {
        if (gray[i] >= level) count++;
    }
    return count;
}

// Counts pixels darker than (mean - offsetBelowMean) in a region. Used on a
// WIDER neighbourhood than the glint ROI to answer: "is the dark pupil/iris
// still anywhere nearby?"
//
// This is the cue that separates a real blink from a GAZE SHIFT. When the
// eyelid closes, the dark pupil vanishes completely. When the eye merely
// rotates to look sideways, the pupil is still there -- it has just moved,
// leaving bright sclera behind in the glint ROI. Glint-absence alone cannot
// tell those apart (validated on session_067: an eye roll at t=21.6s emptied
// the glint ROI and raised its brightness, satisfying both earlier cues, and
// was reported as a blink that never happened).
//
// The threshold is relative to the region's own mean, so it does not care
// about absolute lighting level.
inline int countDarkPixels(const uint8_t *gray, int w, int h, int offsetBelowMean) {
    int n = w * h;
    if (n <= 0) return 0;
    uint32_t sum = 0;
    for (int i = 0; i < n; i++) sum += gray[i];
    int thr = (int)(sum / (uint32_t)n) - offsetBelowMean;
    if (thr < 0) return 0;
    int count = 0;
    for (int i = 0; i < n; i++) if (gray[i] < thr) count++;
    return count;
}

// Glint count AND mean brightness of the same region in ONE pass -- the two
// cues GlintBlinkDetector needs. Costs one compare plus one add per pixel.
inline void glintAndBrightness(const uint8_t *gray, int w, int h, uint8_t level,
                                int &outGlintPx, int &outMeanBrightness) {
    int n = w * h;
    int count = 0;
    uint32_t sum = 0;
    for (int i = 0; i < n; i++) {
        uint8_t v = gray[i];
        sum += v;
        if (v >= level) count++;
    }
    outGlintPx = count;
    outMeanBrightness = (n > 0) ? (int)(sum / (uint32_t)n) : 0;
}

// Blink detector built on glint presence/absence rather than eye SHAPE.
//
// Why shape was abandoned: retrospective validation on real footage
// (session_040, 2026-09-04) showed every shape-based metric tried --
// whole-mask image moments, largest-connected-component moments, and true
// contour+fitEllipse -- was dominated by lighting gradients across the
// curved iris and by which crop edge the dark region happened to touch,
// not by eyelid state. All of them produced false "blinks" on a wide-open
// eye. Glint presence is a binary, physically-grounded cue that none of
// those failure modes affect.
//
// Tuned deliberately for PRECISION over recall (a missed blink is
// acceptable; a false alarm on a rider's helmet is not):
//   - requires the glint to be gone for MIN_ABSENT consecutive frames, so a
//     single noisy frame or a momentarily dimmed reflection can't fire it
//   - requires it to come BACK within MAX_ABSENT frames, so a covered lens,
//     darkness, or the eye leaving frame reads as "no data", not a blink
//   - counts the blink only on recovery (absent -> present), so a partial
//     closure that never fully reopens is never counted
//
// Validated on session_040: 12 detections over 61 s (~11.8 blinks/min,
// squarely in the normal 8-18/min range), and all 12 were confirmed real
// closures by frame-by-frame inspection -- no false positives. The
// MIN_ABSENT >= 2 rule is what rejects the ambiguous single-frame dips.
//
// IMPORTANT: MIN_ABSENT counts PROCESSED frames, so the caller must process
// every captured frame for these numbers to hold. At the ~8 fps this
// hardware actually achieves, a typical 250-300 ms blink spans only ~2
// frames -- throttling would drop it into the sampling gap entirely.
struct GlintBlinkDetector {
    static const uint8_t GLINT_LEVEL   = 200;  // intensity counted as specular
    static const int MIN_GLINT_PX      = 1;    // QVGA: glint is ~4x smaller than at VGA
    static const int MIN_ABSENT_FRAMES = 2;    // shortest run counted as a blink
    static const int MAX_ABSENT_FRAMES = 25;   // longer => occlusion/darkness, not a blink
    static const int MAX_BLINKS_TRACKED = 60;

    // ── Second, independent cue ──────────────────────────────────────────
    // Glint absence alone can't distinguish "the lid covered the cornea"
    // from "the reflection drifted out of the crop while the eye stayed
    // open" -- the latter would be a false positive, the exact thing we
    // most need to avoid. But the two cases differ physically: a real
    // closure replaces the dark iris with skin, so the region gets
    // BRIGHTER, while a glint that merely drifted away leaves the dark
    // iris still filling the crop.
    //
    // Measured on session_040: all 9 post-lock blinks brightened the ROI
    // by +4.1 to +33.6 grey levels against the open-eye median. 3 is set
    // well under the weakest of those so it corroborates without
    // rejecting genuine blinks.
    static const int MIN_BRIGHTEN = 3;
    static constexpr float OPEN_BRIGHTNESS_ALPHA = 0.05f;  // slow EMA, tracks lighting drift

    // ── Third cue: the dark pupil must actually be GONE ──────────────────
    // Rejects gaze shifts, which empty the glint ROI without the eye ever
    // closing. Measured on session_067: real blinks drop the wide-region
    // dark-pixel count to 5-50% of its open-eye level, while an eye roll
    // only reached 84% and a plainly open eye sits at 95-108%. 0.65 sits in
    // the gap with margin on both sides.
    static constexpr float MAX_DARK_FRACTION = 0.65f;
    static const int DARK_OFFSET_BELOW_MEAN = 28;   // for countDarkPixels()

    int  absentRun = 0;
    bool sawValidRun = false;      // current absent run is within [MIN, MAX]
    float openBrightness = -1.0f;  // EMA of ROI brightness while the eye is open
    int  brightestInRun = 0;       // peak ROI brightness during the current absent run
    float openDarkPx = -1.0f;      // EMA of wide-region dark pixels while eye is open
    int  minDarkInRun = 0x7FFFFFFF;// fewest dark pixels seen during the current run
    uint32_t blinkTimestamps[MAX_BLINKS_TRACKED] = {0};
    int blinkCount = 0;

    void pushBlink(uint32_t nowMs) {
        if (blinkCount < MAX_BLINKS_TRACKED) {
            blinkTimestamps[blinkCount++] = nowMs;
        } else {
            memmove(blinkTimestamps, blinkTimestamps + 1,
                    (MAX_BLINKS_TRACKED - 1) * sizeof(uint32_t));
            blinkTimestamps[MAX_BLINKS_TRACKED - 1] = nowMs;
        }
    }

    // Feed one processed frame's glint pixel count and the ROI's mean
    // brightness. Returns true on the frame where a blink is confirmed
    // (the recovery frame), which requires BOTH cues to agree: the glint
    // went away for a plausible blink duration, AND the region brightened
    // while it was gone.
    bool update(int glintPx, int roiBrightness, int wideDarkPx, uint32_t nowMs) {
        bool present = (glintPx >= MIN_GLINT_PX);
        bool blinkEvent = false;

        if (!present) {
            absentRun++;
            if (roiBrightness > brightestInRun) brightestInRun = roiBrightness;
            if (wideDarkPx < minDarkInRun) minDarkInRun = wideDarkPx;
            if (absentRun >= MIN_ABSENT_FRAMES && absentRun <= MAX_ABSENT_FRAMES) {
                sawValidRun = true;
            } else if (absentRun > MAX_ABSENT_FRAMES) {
                sawValidRun = false;   // too long -- not a blink, don't count on recovery
            }
        } else {
            // Only learn the open-eye references while the eye is actually open.
            if (openBrightness < 0.0f) openBrightness = (float)roiBrightness;
            else openBrightness += OPEN_BRIGHTNESS_ALPHA * ((float)roiBrightness - openBrightness);
            if (openDarkPx < 0.0f) openDarkPx = (float)wideDarkPx;
            else openDarkPx += OPEN_BRIGHTNESS_ALPHA * ((float)wideDarkPx - openDarkPx);

            if (sawValidRun) {
                bool brightened = (openBrightness < 0.0f) ||
                                  ((float)brightestInRun >= openBrightness + (float)MIN_BRIGHTEN);
                // Cue 3: the dark pupil must have actually disappeared, not
                // just moved out of the glint ROI (a gaze shift).
                bool pupilGone = (openDarkPx <= 0.0f) ||
                                 ((float)minDarkInRun <= openDarkPx * MAX_DARK_FRACTION);
                if (brightened && pupilGone) {
                    blinkEvent = true;
                    pushBlink(nowMs);
                }
            }
            absentRun = 0;
            sawValidRun = false;
            brightestInRun = 0;
            minDarkInRun = 0x7FFFFFFF;
        }
        return blinkEvent;
    }

    // 60-second rolling rate in blinks/minute.
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

// ── Temporal-motion eye localizer ──────────────────────────────────────────
// Locates the eye using the one property that reliably distinguishes it in a
// helmet-fixed view: it is the only region in frame that *changes*. The
// cheek, brow, hair and background are all rigidly attached to the head and
// move with the camera; only the eyelid and iris move independently, and they
// do so exactly when we care (a blink).
//
// This replaces "find the darkest blob", which has no eye-specific prior and
// was validated on sessions/session_023 to lock onto cheek shadow and brow
// instead of the eye. Retrospective validation of THIS method on the same
// session put the lock inside the eye in 7 of 8 independently-sampled
// windows spread across the whole recording.
//
// Cost: one decimated absolute-difference pass per frame over a GRID_W x
// GRID_H grid (4800 cells), i.e. a few thousand byte-ops -- negligible next
// to the JPEG decode that already happens. Memory is a fixed ~24 KB of
// plain arrays: no heap, no STL, same constraints as the rest of this header.
//
// Validated parameters (session_023, decimated-resolution sweep):
//   - 80x60 grid is the sweet spot: 40x30 loses peak confidence (1.45 vs
//     1.93), 160x120 gains little (2.16) for 4x the work.
//   - ~60 frames (3 s at 20 fps) is ENOUGH, and scores better than longer
//     windows (conf 3.18 at 60 frames vs 1.93 at 600) -- a long accumulation
//     smears the peak as the helmet drifts. Short, repeated locks beat one
//     long one.
struct MotionLocator {
    static const int GRID_W = 80;
    static const int GRID_H = 60;
    static const int GRID_N = GRID_W * GRID_H;
    static const int BLUR_RADIUS = 3;   // 7x7 box blur on the grid

    uint8_t  cur[GRID_N];    // member, not a local: 4800 B is too much to put
                             // on a FreeRTOS task stack (cameraTask's is 4-8 KB)
    uint8_t  prev[GRID_N];
    uint16_t energy[GRID_N];
    uint16_t smooth[GRID_N];
    bool     havePrev;
    int      framesAccumulated;

    MotionLocator() { reset(); }

    void reset() {
        memset(prev, 0, sizeof(prev));
        memset(energy, 0, sizeof(energy));
        memset(smooth, 0, sizeof(smooth));
        havePrev = false;
        framesAccumulated = 0;
    }

    // Box-average `gray` (w*h, 8-bit) down onto the GRID_W x GRID_H grid,
    // then accumulate |current - previous| per cell. Frames smaller than the
    // grid in either axis are rejected (returns without accumulating).
    void addFrame(const uint8_t *gray, int w, int h) {
        if (w < GRID_W || h < GRID_H) return;

        for (int gy = 0; gy < GRID_H; gy++) {
            int sy0 = (int)((int64_t)gy * h / GRID_H);
            int sy1 = (int)((int64_t)(gy + 1) * h / GRID_H);
            if (sy1 <= sy0) sy1 = sy0 + 1;
            for (int gx = 0; gx < GRID_W; gx++) {
                int sx0 = (int)((int64_t)gx * w / GRID_W);
                int sx1 = (int)((int64_t)(gx + 1) * w / GRID_W);
                if (sx1 <= sx0) sx1 = sx0 + 1;
                uint32_t sum = 0;
                uint32_t cnt = 0;
                for (int sy = sy0; sy < sy1; sy++) {
                    const uint8_t *row = gray + (size_t)sy * w;
                    for (int sx = sx0; sx < sx1; sx++) { sum += row[sx]; cnt++; }
                }
                cur[gy * GRID_W + gx] = (uint8_t)(sum / (cnt ? cnt : 1));
            }
        }

        if (havePrev) {
            for (int i = 0; i < GRID_N; i++) {
                int d = (int)cur[i] - (int)prev[i];
                if (d < 0) d = -d;
                uint32_t acc = (uint32_t)energy[i] + (uint32_t)d;
                energy[i] = (acc > 0xFFFFu) ? 0xFFFFu : (uint16_t)acc;  // saturate
            }
            framesAccumulated++;
        }
        memcpy(prev, cur, sizeof(prev));
        havePrev = true;
    }

    // Separable box blur of `energy` into `smooth`, then argmax.
    // Returns the peak location in FULL-FRAME coordinates, plus a confidence
    // = peak / mean. Confidence is the "do I actually see an eye?" gate that
    // the darkest-blob localizer never had: a flat energy map (nothing
    // moving, or motion smeared everywhere by the whole head moving) scores
    // near 1.0, while a clean single moving region scores well above it.
    // Returns false if too few frames have been accumulated.
    bool peak(int fullW, int fullH, int minFrames,
              float &outCx, float &outCy, float &outConfidence) {
        if (framesAccumulated < minFrames) return false;

        // Horizontal pass: energy -> smooth
        for (int y = 0; y < GRID_H; y++) {
            const uint16_t *src = energy + y * GRID_W;
            uint16_t *dst = smooth + y * GRID_W;
            for (int x = 0; x < GRID_W; x++) {
                uint32_t sum = 0;
                int cnt = 0;
                for (int k = -BLUR_RADIUS; k <= BLUR_RADIUS; k++) {
                    int xx = x + k;
                    if (xx < 0 || xx >= GRID_W) continue;
                    sum += src[xx]; cnt++;
                }
                dst[x] = (uint16_t)(sum / (cnt ? cnt : 1));
            }
        }
        // Vertical pass: smooth -> smooth, one column at a time via a stack
        // copy so the running reads aren't clobbered by the writes.
        for (int x = 0; x < GRID_W; x++) {
            uint16_t col[GRID_H];
            for (int y = 0; y < GRID_H; y++) col[y] = smooth[y * GRID_W + x];
            for (int y = 0; y < GRID_H; y++) {
                uint32_t sum = 0;
                int cnt = 0;
                for (int k = -BLUR_RADIUS; k <= BLUR_RADIUS; k++) {
                    int yy = y + k;
                    if (yy < 0 || yy >= GRID_H) continue;
                    sum += col[yy]; cnt++;
                }
                smooth[y * GRID_W + x] = (uint16_t)(sum / (cnt ? cnt : 1));
            }
        }

        uint32_t total = 0;
        int bestIdx = 0;
        uint16_t bestVal = 0;
        for (int i = 0; i < GRID_N; i++) {
            total += smooth[i];
            if (smooth[i] > bestVal) { bestVal = smooth[i]; bestIdx = i; }
        }
        if (bestVal == 0) return false;

        float mean = (float)total / (float)GRID_N;
        outConfidence = (mean > 1e-6f) ? ((float)bestVal / mean) : 0.0f;

        int gx = bestIdx % GRID_W;
        int gy = bestIdx / GRID_W;
        outCx = ((float)gx + 0.5f) * (float)fullW / (float)GRID_W;
        outCy = ((float)gy + 0.5f) * (float)fullH / (float)GRID_H;
        return true;
    }
};

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
    static const int BASELINE_WINDOW = 45;  // TEST: scaled for throttle=2
    static const int COOLDOWN_SAMPLES = 9;  // TEST: scaled for throttle=2
    static const int WARMUP_SAMPLES = 36;  // TEST: scaled for throttle=2
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
