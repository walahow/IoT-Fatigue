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
static const int   EAR_THROTTLE_DIV = 3;
static const int   EAR_ROI_SIZE     = 64;
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
    fprintf(stderr, "Usage: %s <frames_dir> <output_csv>\n", argv[0]);
    return 1;
  }
  std::string framesDir = argv[1];
  std::string outCsvPath = argv[2];

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
  fprintf(csv, "timestamp_ms,processed,ear,is_blink_event,rolling_rate_bpm,roi_x,roi_y,roi_locked\n");

  // ── Replay state (mirrors main.cpp's globals for this pipeline) ─────────
  uint8_t *earGrayBuf = nullptr;
  uint8_t *earMaskBuf = nullptr;
  uint16_t *earRowSumBuf = nullptr;
  int earFullW = 0, earFullH = 0;

  uint32_t earFrameCounter = 0;
  uint32_t earDriftCounter = 0;

  std::vector<float> lockSamplesX, lockSamplesY;
  bool earLockDone = false;
  uint32_t firstFrameTs = frames.front().timestampMs;

  EyeBlinkEAR::RoiLock roi;
  EyeBlinkEAR::BlinkDetector blink;

  int totalBlinkEvents = 0;
  uint32_t lockCompletedAtMs = 0;

  for (const auto &f : frames) {
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

    if (!earLockDone) {
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
      if (++earDriftCounter % EAR_DRIFT_PERIOD == 0) {
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
          float newFullCx = dx + localCx;
          float newFullCy = dy + localCy;
          float curCx = roi.x + roi.size / 2.0f;
          float curCy = roi.y + roi.size / 2.0f;

          float blendedCx, blendedCy;
          if (EyeBlinkEAR::driftBlend(curCx, curCy, newFullCx, newFullCy,
                                       EAR_DRIFT_MAX_PX, EAR_DRIFT_ALPHA,
                                       blendedCx, blendedCy)) {
            int rx = (int)(blendedCx - roi.size / 2.0f);
            int ry = (int)(blendedCy - roi.size / 2.0f);
            if (rx + roi.size > w) rx = w - roi.size;
            if (ry + roi.size > h) ry = h - roi.size;
            if (rx < 0) rx = 0;
            if (ry < 0) ry = 0;
            roi.x = rx;
            roi.y = ry;
          }
        }
      }

      earExtractGray(rgb, w, h, roi.x, roi.y, roi.size, roi.size, earGrayBuf);
      earThresholdToMask(earGrayBuf, earMaskBuf, roi.size * roi.size);

      EyeBlinkEAR::EarResult ear = EyeBlinkEAR::computeEAR(earMaskBuf, roi.size, roi.size);
      if (ear.valid) {
        earValue = ear.ear;
        blinkEvent = blink.update(ear.ear, f.timestampMs);
        rollingRate = blink.rollingRateBpm(f.timestampMs);
        if (blinkEvent) totalBlinkEvents++;
        wroteRow = true;
      }
    }

    std::string earField = wroteRow ? std::to_string(earValue) : std::string();
    fprintf(csv, "%u,1,%s,%d,%.2f,%d,%d,%d\n",
            f.timestampMs, earField.c_str(),
            blinkEvent ? 1 : 0, rollingRate, roi.x, roi.y, roi.locked ? 1 : 0);

    stbi_image_free(rgb);
  }

  fclose(csv);

  printf("\n=== session_replay summary ===\n");
  printf("Total frames in session : %zu\n", frames.size());
  printf("Lock samples collected  : %zu (of %d target)\n", lockSamplesX.size(), EAR_LOCK_SAMPLES);
  printf("ROI locked              : %s", earLockDone ? "yes" : "no");
  if (earLockDone) printf(" (at t=%ums)", lockCompletedAtMs);
  printf("\n");
  printf("Total blink events      : %d\n", totalBlinkEvents);
  printf("Output CSV              : %s\n", outCsvPath.c_str());

  return 0;
}
