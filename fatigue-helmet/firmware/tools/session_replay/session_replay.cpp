// session_replay.cpp
// ============================================================================
// Host-side validation harness: runs the EXACT, unmodified EyeBlinkEAR.h
// algorithm (same source file the ESP32 firmware compiles) against a folder
// of already-recorded JPEG frames from a real session, replicating
// processEarFrame()'s logic from main.cpp (same constants, same throttle,
// same boot-lock/drift/EAR/blink flow) -- just fed from decoded JPEG files
// on the host instead of camera_fb_t from a live ESP32-S3 camera.
//
// This does NOT run on the ESP32. It runs the same algorithm, compiled for
// the host, so you can see how it performs on real recorded footage without
// needing to build a live frame-injection mode on the actual chip.
//
// Usage:
//   session_replay <frames_dir> <output_csv>
//
// frames_dir must contain files named {timestamp_ms}.jpg (the naming
// convention used by unpack_session.py / debug_recorder.py).
// ============================================================================

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#if defined(_WIN32)
#include <windows.h>
#endif

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#include "stb_image.h"

#include "../../src/EyeBlinkEAR.h"

// ── Same constants as main.cpp's processEarFrame() integration ────────────
static const int   EAR_THROTTLE_DIV = 1;   // every frame, matches main.cpp
#ifndef EAR_ROI_SIZE
// 48 matches main.cpp and is correct for QVGA. A VGA recording covers the
// same eye with roughly twice the pixels, so replaying one of those wants
// -DEAR_ROI_SIZE=96. Must stay compile-time: it sizes the scratch arrays.
#define EAR_ROI_SIZE 48
#endif
static const int   EAR_ROI_SIZE_V   = EAR_ROI_SIZE;  // QVGA, matches main.cpp
static const int   EAR_DRIFT_PERIOD = 40;
static const int   EAR_DRIFT_MARGIN = 30;
static const float EAR_DRIFT_MAX_PX = 20.0f;
static const float EAR_DRIFT_ALPHA  = 0.3f;

// ── Pupil-localization constants (boot-lock + drift-check) ─────────────────
// Validated in Python against sessions/session_023's real frames before
// being ported here -- see the EyeBlinkEAR.h commits that added
// percentileThreshold() and findDarkestWindow().
static const int      EAR_LOCK_SAMPLES     = 60;   // more thorough one-time pass at boot
static const float    EAR_LOCK_PERCENTILE  = 10.0f;
static const int      EAR_LOCK_WIN_SIZE    = 22;   // ~pupil diameter at this resolution
static const int      EAR_LOCK_STRIDE      = 2;
static const uint32_t EAR_LOCK_WARMUP_MS   = 2000; // let the subject settle before sampling
static const int      EAR_DRIFT_BURST      = 8;    // samples per drift-check (median, not single-frame)
// Drift-correction is validated as noise-resistant on its own (median-of-8,
// two-burst confirmation), but retrospective validation against session_023
// found it can confidently track hair occluding the camera later in this
// specific session, since hair is a genuinely darker/more stable feature
// than the eye once it's in frame -- a different problem (occlusion) than
// what drift-correction was built to solve (real small physical drift).
// Disabled for now: ship the validated boot-lock, frozen for the session,
// and revisit occlusion-robust drift as a separate piece of work.
static const bool     EAR_DRIFT_ENABLED    = false;

// ── Motion-based localization (alternative to the darkest-blob boot lock) ──
// Validated on session_023 at decimated resolution: 60 frames is enough and
// scores better than longer windows; confidence (peak/mean) separates a real
// eye lock from a smeared/flat map.
#ifndef EAR_MOTION_MIN_FRAMES
#if EAR_LOCALIZER_VERSION >= 3
#define EAR_MOTION_MIN_FRAMES 120   // frame floor under v3's time window
#elif EAR_LOCALIZER_VERSION == 2
#define EAR_MOTION_MIN_FRAMES 240   // ~20 s; see main.cpp for why
#else
#define EAR_MOTION_MIN_FRAMES 60
#endif
#endif
static const int   EAR_MOTION_MIN_FRAMES_V = EAR_MOTION_MIN_FRAMES;  // ~3 s at 20 fps
static const float EAR_MOTION_MIN_CONF   = 2.0f;  // reject flat/smeared maps
static const int   EAR_MOTION_MAX_TRIES  = 12;    // give up after this many windows

// ── Same helpers as main.cpp (byte-identical logic) ────────────────────────
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

static void earThresholdToMask(const uint8_t *gray, uint8_t *mask, int n) {
  uint8_t t = EyeBlinkEAR::otsuThreshold(gray, n, 1);
  for (int i = 0; i < n; i++) mask[i] = (gray[i] <= t) ? 1 : 0;
}

// Percentile-threshold variant, used for pupil localization (boot-lock and
// drift-check) instead of Otsu -- see percentileThreshold()'s doc comment.
static void earPercentileMask(const uint8_t *gray, uint8_t *mask, int n) {
  uint8_t t = EyeBlinkEAR::percentileThreshold(gray, n, 1, EAR_LOCK_PERCENTILE);
  for (int i = 0; i < n; i++) mask[i] = (gray[i] <= t) ? 1 : 0;
}

// ── Frame listing ───────────────────────────────────────────────────────────
struct FrameFile {
  uint32_t timestampMs;
  std::string path;
};

static bool listFrames(const std::string &dir, std::vector<FrameFile> &out) {
#if defined(_WIN32)
  std::string pattern = dir + "\\*.jpg";
  WIN32_FIND_DATAA fd;
  HANDLE h = FindFirstFileA(pattern.c_str(), &fd);
  if (h == INVALID_HANDLE_VALUE) return false;
  do {
    std::string name = fd.cFileName;
    size_t dot = name.find(".jpg");
    if (dot == std::string::npos) continue;
    uint32_t ts = (uint32_t)strtoul(name.substr(0, dot).c_str(), nullptr, 10);
    out.push_back({ts, dir + "\\" + name});
  } while (FindNextFileA(h, &fd));
  FindClose(h);
#else
  return false; // not needed on this project's target host (Windows)
#endif
  std::sort(out.begin(), out.end(),
            [](const FrameFile &a, const FrameFile &b) { return a.timestampMs < b.timestampMs; });
  return true;
}

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "Usage: %s <frames_dir> <output_csv> [--lock=motion|dark] [--roi=X,Y]\n"
                    "       [--hog-dump=feat.bin] [--hog-model=model.bin --hog-csv=hog.csv]\n", argv[0]);
    return 1;
  }
  std::string framesDir = argv[1];
  std::string outCsvPath = argv[2];
  // Lock mode: "dark" = original darkest-blob boot lock, "motion" = temporal
  // motion-energy localizer. Default motion (the validated one).
  bool useMotionLock = true;
  int  maxFrames = 0;    // 0 = all
  int  skipFrames = 0;   // drop this many leading frames
  bool manualRoi = false;
  int  manualCx = 0, manualCy = 0;
  const char *hogDumpPath = nullptr, *hogModelPath = nullptr, *hogCsvPath = nullptr;
  for (int i = 3; i < argc; i++) {
    if (strcmp(argv[i], "--lock=dark") == 0) useMotionLock = false;
    else if (strcmp(argv[i], "--lock=motion") == 0) useMotionLock = true;
    else if (strncmp(argv[i], "--skip-frames=", 14) == 0) {
      // Drop the first N frames. With --roi pinned this varies ONLY which
      // frame the blink detector starts on, which is how the stability of
      // its open-eye baselines is measured.
      skipFrames = atoi(argv[i] + 14);
    }
    else if (strncmp(argv[i], "--max-frames=", 13) == 0) {
      // Stop after N frames. The ROI lock is decided in the first ~120,
      // so a localizer sweep need not decode a 10k-frame session.
      maxFrames = atoi(argv[i] + 13);
    }
    else if (strncmp(argv[i], "--roi=", 6) == 0) {
      // Pin the ROI centre instead of locking. Mirrors the firmware's
      // EAR_ROI_MANUAL_X/Y build flags, and lets the blink algorithm be
      // tested independently of whether the locator found the eye.
      if (sscanf(argv[i] + 6, "%d,%d", &manualCx, &manualCy) == 2) manualRoi = true;
    }
    // HOG classifier. --hog-dump writes, per locked frame, a record of
    // {uint32 timestamp_ms, float frame_mean, float features[HOG_LEN]} for
    // python/train_blink_classifier.py -- features computed by THIS code, so
    // the weights it trains match what the device computes. --hog-model reads
    // {float weights[HOG_LEN], bias, threshold} (written by that script) and
    // --hog-csv logs the per-frame score and blink events.
    else if (strncmp(argv[i], "--hog-dump=", 11) == 0) hogDumpPath = argv[i] + 11;
    else if (strncmp(argv[i], "--hog-model=", 12) == 0) hogModelPath = argv[i] + 12;
    else if (strncmp(argv[i], "--hog-csv=", 10) == 0) hogCsvPath = argv[i] + 10;
  }
  FILE *hogDump = hogDumpPath ? fopen(hogDumpPath, "wb") : nullptr;
  static float hogModel[EyeBlinkEAR::HOG_LEN + 2];   // weights, bias, threshold
  FILE *hogCsv = nullptr;
  if (hogModelPath) {
    FILE *mf = fopen(hogModelPath, "rb");
    size_t got = mf ? fread(hogModel, sizeof(float), EyeBlinkEAR::HOG_LEN + 2, mf) : 0;
    if (mf) fclose(mf);
    if (got != (size_t)EyeBlinkEAR::HOG_LEN + 2 || !hogCsvPath) {
      fprintf(stderr, "ERROR: --hog-model needs a %d-float model file and --hog-csv\n",
              EyeBlinkEAR::HOG_LEN + 2);
      return 1;
    }
    hogCsv = fopen(hogCsvPath, "w");
    fprintf(hogCsv, "timestamp_ms,frame_mean,score,gated,blink\n");
  }
  EyeBlinkEAR::HogBlinkDetector hogDet;
  hogDet.threshold = hogModel[EyeBlinkEAR::HOG_LEN + 1];
  int hogBlinks = 0;
  printf("Lock mode: %s\n", useMotionLock ? "motion-energy" : "darkest-blob");

  std::vector<FrameFile> frames;
  if (!listFrames(framesDir, frames) || frames.empty()) {
    fprintf(stderr, "ERROR: no .jpg frames found in %s\n", framesDir.c_str());
    return 1;
  }
  printf("Found %zu frames. First ts=%u  Last ts=%u\n",
         frames.size(), frames.front().timestampMs, frames.back().timestampMs);

  FILE *csv = fopen(outCsvPath.c_str(), "w");
  if (!csv) {
    fprintf(stderr, "ERROR: cannot open %s for writing\n", outCsvPath.c_str());
    return 1;
  }
  fprintf(csv, "timestamp_ms,processed,ear,is_blink_event,rolling_rate_bpm,roi_x,roi_y,roi_locked,lock_conf,cand,run_len,brightest,open_bright,min_dark,open_dark,brightened,pupil_gone\n");

  // ── Replay state (mirrors main.cpp's globals for this pipeline) ─────────
  uint8_t *earGrayBuf = nullptr;
  uint8_t *earMaskBuf = nullptr;
  uint16_t *earRowSumBuf = nullptr;
  int earFullW = 0, earFullH = 0;

  // Scratch for isolateLargestComponent(), sized to the locked EAR crop --
  // fixed at compile time, matching main.cpp's static arrays.
  static int16_t  earLabelScratch[EAR_ROI_SIZE * EAR_ROI_SIZE];
  static uint16_t earQueueScratch[EAR_ROI_SIZE * EAR_ROI_SIZE];
  static uint8_t  earIsolatedMask[EAR_ROI_SIZE * EAR_ROI_SIZE];

  uint32_t earFrameCounter = 0;
  uint32_t earDriftCounter = 0;
  bool driftBursting = false;
  std::vector<float> driftSamplesX, driftSamplesY;
  bool havePendingCandidate = false;
  float pendingCandidateX = 0, pendingCandidateY = 0;

  std::vector<float> lockSamplesX, lockSamplesY;
  bool earLockDone = false;
  uint32_t firstFrameTs = frames.front().timestampMs;

#if EAR_LOCALIZER_VERSION >= 4
  // Scratch for v4's darkest-blob re-centre. Static rather than stack: at
  // EAR_ROI_SIZE 48 this is 25 KB, and the host stack is not the place for it.
  static const int REFINE_SIDE = EAR_ROI_SIZE + 2 * EAR_V4_MARGIN;
  static uint8_t  refGray[REFINE_SIDE * REFINE_SIDE];
  static uint8_t  refMask[REFINE_SIDE * REFINE_SIDE];
  static uint16_t refRow [REFINE_SIDE * REFINE_SIDE];
#endif

  EyeBlinkEAR::RoiLock roi;
  EyeBlinkEAR::BlinkDetector blink;
  EyeBlinkEAR::GlintBlinkDetector glint;
  static EyeBlinkEAR::MotionLocator locator;   // ~24 KB, keep off the stack
  int motionTries = 0;
  float lockConfidence = 0.0f;

  int totalBlinkEvents = 0;
  uint32_t lockCompletedAtMs = 0;

  int _skipped = 0;
  for (const auto &f : frames) {
    if (_skipped < skipFrames) { _skipped++; continue; }
    if (maxFrames > 0 && (int)earFrameCounter >= maxFrames) break;
    earFrameCounter++;
    // Boot-lock (not yet locked): process EVERY captured frame -- a more
    // thorough one-time search before steady-state processing begins, since
    // there's no real-time budget pressure yet. Once locked: throttle to
    // keep steady-state CPU cost bounded, same as before.
    if (earLockDone && (earFrameCounter % EAR_THROTTLE_DIV != 0)) continue;

    int w, h, comp;
    uint8_t *rgb = stbi_load(f.path.c_str(), &w, &h, &comp, 3);
    if (!rgb) {
      fprintf(stderr, "WARN: failed to decode %s, skipping\n", f.path.c_str());
      continue;
    }

    if (!earGrayBuf) {
      earGrayBuf = (uint8_t *)malloc((size_t)w * h);
      earMaskBuf = (uint8_t *)malloc((size_t)w * h);
      earRowSumBuf = (uint16_t *)malloc((size_t)w * h * sizeof(uint16_t));
      earFullW = w;
      earFullH = h;
    }
    if (w != earFullW || h != earFullH) {
      stbi_image_free(rgb);
      continue; // frame size changed mid-session, skip (matches main.cpp guard)
    }

    bool wroteRow = false;
    float earValue = -1.0f;
    bool blinkEvent = false;
    float rollingRate = 0.0f;

    if (!earLockDone && manualRoi) {
      float xs[1] = {(float)manualCx}, ys[1] = {(float)manualCy};
      EyeBlinkEAR::lockRoiFromSamples(xs, ys, 1, EAR_ROI_SIZE_V, w, h, roi);
      earLockDone = true;
      lockConfidence = -1.0f;       // -1 = pinned, not measured
      lockCompletedAtMs = f.timestampMs;
      printf("#STATUS: ROI pinned at x=%d y=%d size=%d\n", roi.x, roi.y, roi.size);
    }

    if (!earLockDone && useMotionLock) {
      // Motion-energy lock: accumulate |dI| over a short window, then take
      // the peak. Retries with a fresh window if confidence is too low,
      // rather than committing to a bad lock the way darkest-blob did.
      earExtractGray(rgb, w, h, 0, 0, w, h, earGrayBuf);
      bool pastWarmup = (f.timestampMs - firstFrameTs) >= EAR_LOCK_WARMUP_MS;
      if (pastWarmup) {
#if EAR_LOCALIZER_VERSION >= 2
        locator.setRoiSize(EAR_ROI_SIZE_V);
#endif
#if EAR_LOCALIZER_VERSION >= 3
        locator.addFrame(earGrayBuf, w, h, f.timestampMs);
#else
        locator.addFrame(earGrayBuf, w, h);
#endif
        float mcx, mcy, conf;
        if (locator.peak(w, h, EAR_MOTION_MIN_FRAMES_V, mcx, mcy, conf)) {
          if (conf >= EAR_MOTION_MIN_CONF || ++motionTries >= EAR_MOTION_MAX_TRIES) {
#if EAR_LOCALIZER_VERSION >= 4
            // Motion energy peaks on the moving eyelid; the pupil sits a few
            // pixels off. Re-centre before locking, so the detector's mean
            // brightness and dark fraction are taken over a crop the pupil is
            // in the middle of.
            bool moved = locator.refine(earGrayBuf, w, h,
                                        refGray, refMask, refRow, mcx, mcy);
            printf("#STATUS: refine %s -> (%.0f, %.0f)\n",
                   moved ? "moved" : "rejected", mcx, mcy);
#endif
            float xs[1] = {mcx}, ys[1] = {mcy};
            EyeBlinkEAR::lockRoiFromSamples(xs, ys, 1, EAR_ROI_SIZE_V, w, h, roi);
            earLockDone = true;
            lockConfidence = conf;
            lockCompletedAtMs = f.timestampMs;
            printf("#STATUS: motion lock at x=%d y=%d size=%d conf=%.2f (t=%ums, %d tries)\n",
                   roi.x, roi.y, roi.size, conf, f.timestampMs, motionTries + 1);
          } else {
            printf("#STATUS: motion lock REJECTED conf=%.2f (<%.2f) at t=%ums, retrying\n",
                   conf, EAR_MOTION_MIN_CONF, f.timestampMs);
            locator.reset();
          }
        }
      }
    } else if (!earLockDone) {
      // Full-frame search -- no assumption about where in the frame the eye
      // sits (validated against session_023: the eye was NOT centered).
      earExtractGray(rgb, w, h, 0, 0, w, h, earGrayBuf);
      earPercentileMask(earGrayBuf, earMaskBuf, w * h);

      float fullCx, fullCy;
      bool pastWarmup = (f.timestampMs - firstFrameTs) >= EAR_LOCK_WARMUP_MS;
      bool gotSample = pastWarmup &&
          EyeBlinkEAR::findDarkestWindow(earMaskBuf, w, h, EAR_LOCK_WIN_SIZE,
                                          EAR_LOCK_STRIDE, EAR_LOCK_STRIDE,
                                          earRowSumBuf, fullCx, fullCy);
      if (gotSample && (int)lockSamplesX.size() < EAR_LOCK_SAMPLES) {
        lockSamplesX.push_back(fullCx);
        lockSamplesY.push_back(fullCy);
      }

      if ((int)lockSamplesX.size() >= EAR_LOCK_SAMPLES) {
        EyeBlinkEAR::lockRoiFromSamples(lockSamplesX.data(), lockSamplesY.data(),
                                         (int)lockSamplesX.size(), EAR_ROI_SIZE, w, h, roi);
        earLockDone = true;
        lockCompletedAtMs = f.timestampMs;
        printf("#STATUS: EAR ROI locked at x=%d y=%d size=%d (t=%ums)\n",
               roi.x, roi.y, roi.size, f.timestampMs);
      }
    } else {
      // Periodic drift check: a SINGLE findDarkestWindow sample is too noisy
      // to trust on its own (validated against session_023: every single
      // per-frame drift sample landed 20-70px from the current ROI, purely
      // from single-frame search noise, so the shift gate rejected every
      // one -- the ROI never moved once in the whole 9-minute session, even
      // though it clearly should have). Same fix as the boot-lock: gather a
      // small burst of samples and use their median instead of one frame.
      ++earDriftCounter;
      if (EAR_DRIFT_ENABLED && !driftBursting && earDriftCounter % EAR_DRIFT_PERIOD == 0) {
        driftBursting = true;
        driftSamplesX.clear();
        driftSamplesY.clear();
      }

      if (driftBursting) {
        int dx = roi.x - EAR_DRIFT_MARGIN;
        int dy = roi.y - EAR_DRIFT_MARGIN;
        int dw = roi.size + 2 * EAR_DRIFT_MARGIN;
        int dh = roi.size + 2 * EAR_DRIFT_MARGIN;

        earExtractGray(rgb, w, h, dx, dy, dw, dh, earGrayBuf);
        earPercentileMask(earGrayBuf, earMaskBuf, dw * dh);

        float localCx, localCy;
        if (EyeBlinkEAR::findDarkestWindow(earMaskBuf, dw, dh, EAR_LOCK_WIN_SIZE,
                                            EAR_LOCK_STRIDE, EAR_LOCK_STRIDE,
                                            earRowSumBuf, localCx, localCy)) {
          driftSamplesX.push_back(dx + localCx);
          driftSamplesY.push_back(dy + localCy);
        }

        if ((int)driftSamplesX.size() >= EAR_DRIFT_BURST) {
          driftBursting = false;

          std::vector<float> sortedX = driftSamplesX, sortedY = driftSamplesY;
          std::sort(sortedX.begin(), sortedX.end());
          std::sort(sortedY.begin(), sortedY.end());
          float newFullCx = sortedX[sortedX.size() / 2];
          float newFullCy = sortedY[sortedY.size() / 2];

          float curCx = roi.x + roi.size / 2.0f;
          float curCy = roi.y + roi.size / 2.0f;
          float medShift = sqrtf((newFullCx - curCx) * (newFullCx - curCx) +
                                  (newFullCy - curCy) * (newFullCy - curCy));
          fprintf(stderr, "DRIFTBURST ts=%u cur=(%.1f,%.1f) median=(%.1f,%.1f) shift=%.1f  samples: ",
                  f.timestampMs, curCx, curCy, newFullCx, newFullCy, medShift);
          for (size_t i = 0; i < driftSamplesX.size(); i++) {
            fprintf(stderr, "(%.0f,%.0f) ", driftSamplesX[i], driftSamplesY[i]);
          }
          fprintf(stderr, "\n");

          float blendedCx = 0, blendedCy = 0;
          bool apply = false;

          if (EyeBlinkEAR::driftBlend(curCx, curCy, newFullCx, newFullCy,
                                       EAR_DRIFT_MAX_PX, EAR_DRIFT_ALPHA,
                                       blendedCx, blendedCy)) {
            // Small shift from the current ROI -- trust it immediately.
            apply = true;
            havePendingCandidate = false;
          } else {
            // Large shift from the current ROI. Don't trust a single burst
            // this far off (could be the eye closed mid-blink, a shadow,
            // etc.) -- but if an EARLIER, INDEPENDENT burst already found
            // a closely-agreeing position, two independent confirmations
            // is real signal, not noise. Apply the full correction then.
            float agreeShift = 1e9f;
            if (havePendingCandidate) {
              float ddx = newFullCx - pendingCandidateX;
              float ddy = newFullCy - pendingCandidateY;
              agreeShift = sqrtf(ddx * ddx + ddy * ddy);
            }
            if (havePendingCandidate && agreeShift < EAR_DRIFT_MAX_PX) {
              blendedCx = newFullCx;
              blendedCy = newFullCy;
              apply = true;
              havePendingCandidate = false;
            } else {
              pendingCandidateX = newFullCx;
              pendingCandidateY = newFullCy;
              havePendingCandidate = true;
            }
          }

          if (apply) {
            int rx = (int)(blendedCx - roi.size / 2.0f);
            int ry = (int)(blendedCy - roi.size / 2.0f);
            if (rx + roi.size > w) rx = w - roi.size;
            if (ry + roi.size > h) ry = h - roi.size;
            if (rx < 0) rx = 0;
            if (ry < 0) ry = 0;
            roi.x = rx;
            roi.y = ry;
            fprintf(stderr, "  -> APPLIED new roi=(%d,%d)\n", roi.x, roi.y);
          }
        }
      }

      earExtractGray(rgb, w, h, roi.x, roi.y, roi.size, roi.size, earGrayBuf);

      // Glint-based detection -- same call main.cpp makes.
      int glintPx = 0, roiBrightness = 0;
      EyeBlinkEAR::glintAndBrightness(
          earGrayBuf, roi.size, roi.size,
          EyeBlinkEAR::GlintBlinkDetector::GLINT_LEVEL, glintPx, roiBrightness);
      earValue = (float)glintPx;   // "ear" column now carries the glint count
      const int WIDE_PAD = EAR_ROI_SIZE / 2;
      int ww = roi.size + 2 * WIDE_PAD, wh = roi.size + 2 * WIDE_PAD;
      earExtractGray(rgb, w, h, roi.x - WIDE_PAD, roi.y - WIDE_PAD, ww, wh, earGrayBuf);
      int wideDarkPx = EyeBlinkEAR::countDarkPixels(
          earGrayBuf, ww, wh, EyeBlinkEAR::GlintBlinkDetector::DARK_OFFSET_BELOW_MEAN);
      blinkEvent = glint.update(glintPx, roiBrightness, wideDarkPx, f.timestampMs);
      rollingRate = glint.rollingRateBpm(f.timestampMs);
      if (blinkEvent) totalBlinkEvents++;

      // HOG classifier -- the calls main.cpp will make, fed the same crop.
      if (hogDump || hogCsv) {
        int cx, cy, cw, ch;
        EyeBlinkEAR::hogCropRect(roi, cx, cy, cw, ch);
        earExtractGray(rgb, w, h, cx, cy, cw, ch, earGrayBuf);
        uint8_t small[EyeBlinkEAR::HOG_W * EyeBlinkEAR::HOG_H];
        float feat[EyeBlinkEAR::HOG_LEN];
        EyeBlinkEAR::boxResample(earGrayBuf, cw, ch, small, EyeBlinkEAR::HOG_W, EyeBlinkEAR::HOG_H);
        EyeBlinkEAR::hogFeatures(small, feat);
        // Whole-frame brightness, for designing the exposure/occlusion gate.
        uint64_t sum = 0;
        int n = 0;
        for (int i = 0; i < w * h; i += 4, n++) sum += (rgb[i * 3] + rgb[i * 3 + 1] + rgb[i * 3 + 2]) / 3;
        float frameMean = (float)sum / n;
        if (hogDump) {
          fwrite(&f.timestampMs, sizeof(uint32_t), 1, hogDump);
          fwrite(&frameMean, sizeof(float), 1, hogDump);
          fwrite(feat, sizeof(float), EyeBlinkEAR::HOG_LEN, hogDump);
        }
        if (hogCsv) {
          float score = EyeBlinkEAR::linearScore(feat, hogModel, hogModel[EyeBlinkEAR::HOG_LEN]);
          bool gated = false;
          bool ev = hogDet.update(score, gated);
          if (ev) hogBlinks++;
          fprintf(hogCsv, "%u,%.2f,%.4f,%d,%d\n", f.timestampMs, frameMean, score, gated ? 1 : 0, ev ? 1 : 0);
        }
      }
      wroteRow = true;
    }

    std::string earField = wroteRow ? std::to_string(earValue) : std::string();
    fprintf(csv, "%u,1,%s,%d,%.2f,%d,%d,%d,%.2f,%d,%d,%d,%.1f,%d,%.1f,%d,%d\n",
            f.timestampMs, earField.c_str(),
            blinkEvent ? 1 : 0, rollingRate, roi.x, roi.y, roi.locked ? 1 : 0,
            lockConfidence,
            glint.dbgCandidate ? 1 : 0, glint.dbgRunLen, glint.dbgBrightest,
            glint.dbgOpenBright, glint.dbgMinDark, glint.dbgOpenDark,
            glint.dbgBrightened ? 1 : 0, glint.dbgPupilGone ? 1 : 0);

    stbi_image_free(rgb);
  }

  fclose(csv);
  if (hogDump) fclose(hogDump);
  if (hogCsv) fclose(hogCsv);

  printf("\n=== session_replay summary ===\n");
  printf("Total frames in session : %zu\n", frames.size());
  if (useMotionLock)
    printf("Lock confidence         : %.2f (min %.2f)\n", lockConfidence, EAR_MOTION_MIN_CONF);
  else
    printf("Lock samples collected  : %zu (of %d target)\n", lockSamplesX.size(), EAR_LOCK_SAMPLES);
  printf("ROI locked              : %s", earLockDone ? "yes" : "no");
  if (earLockDone) printf(" (at t=%ums)", lockCompletedAtMs);
  printf("\n");
  printf("Total blink events      : %d\n", totalBlinkEvents);
  if (hogCsv) printf("HOG classifier blinks   : %d\n", hogBlinks);
  printf("Output CSV              : %s\n", outCsvPath.c_str());

  return 0;
}
