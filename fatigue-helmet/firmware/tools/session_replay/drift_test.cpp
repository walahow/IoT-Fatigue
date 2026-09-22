// drift_test.cpp
// ============================================================================
// Host-side test of main.cpp's earDriftTick() (the new motion-based drift
// correction) against a real session's already-extracted frames. NOT a
// general-purpose replay tool like session_replay.cpp -- that tool's own
// boot-lock and drift code have drifted out of sync with main.cpp over time
// (different localizer, different old drift algorithm) and reconciling them
// was a bigger job than this test needed. This is a direct transcription of
// earDriftTick() from main.cpp (2026-09-22) -- if the two diverge, this file
// is stale, not main.cpp; re-diff against main.cpp before trusting a result.
//
// Starts the ROI at the session's own recorded metadata.txt roi_x/roi_y/
// roi_size (the same "stored ROI" the real firmware locked onto for this
// session) rather than running the boot lock -- we already know where that
// landed, and only drift correction is under test here.
//
// Usage:
//   drift_test <frames_dir> <metadata.txt> <out.csv>
//
// out.csv columns: timestamp_ms,roi_x,roi_y,event
//   event is empty, "corrected", or "not_confident" -- one row per processed
//   frame either way, so a plot of roi_x/roi_y against timestamp_ms shows the
//   ROI's whole path through the session, not just the moments it moved.
// ============================================================================

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>
#if defined(_WIN32)
#include <windows.h>
#endif

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#include "stb_image.h"

#include "../../src/EyeBlinkEAR.h"

// ── Same constants as main.cpp (kept in sync by hand -- see file header) ───
static const int EAR_ROI_SIZE     = 48;
static const int EAR_DRIFT_MARGIN = 30;
static const float EAR_MOTION_MIN_CONF = 2.0f;
#if EAR_LOCALIZER_VERSION >= 3
static const int EAR_MOTION_MIN_FRAMES = 120;
#else
static const int EAR_MOTION_MIN_FRAMES = 60;
#endif
#if EAR_LOCALIZER_VERSION >= 4
static const int EAR_REFINE_SIDE = EAR_ROI_SIZE + 2 * EAR_V4_MARGIN;
#endif

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

// ── Drift state, transcribed from main.cpp's earDriftTick() ────────────────
enum EarDriftState : uint8_t { EAR_DRIFT_IDLE = 0, EAR_DRIFT_ACCUMULATING };
static EarDriftState g_earDriftState = EAR_DRIFT_IDLE;
static EyeBlinkEAR::MotionLocator g_earLocator;
static uint8_t  *g_earRefGray = nullptr;
static uint8_t  *g_earRefMask = nullptr;
static uint16_t *g_earRefRow  = nullptr;

struct RoiLock { int x, y, size; };
static RoiLock g_earRoi;

// Returns: 0 = still accumulating, 1 = cycle ended not confident, 2 = corrected
static int earDriftTick(const uint8_t *rgb, int w, int h, uint32_t timestampMs,
                         uint8_t *grayScratch, float &outConf) {
  if (g_earDriftState == EAR_DRIFT_IDLE) {
    g_earLocator.reset();
#if EAR_LOCALIZER_VERSION >= 2
    g_earLocator.setRoiSize(EAR_ROI_SIZE);
#endif
#if EAR_LOCALIZER_VERSION >= 3
    g_earLocator.setWindowMs(EAR_V3_WINDOW_MS);
#endif
    g_earDriftState = EAR_DRIFT_ACCUMULATING;
  }

  const int dw = EAR_ROI_SIZE + 2 * EAR_DRIFT_MARGIN;
  const int dh = dw;
  const int dx = g_earRoi.x - EAR_DRIFT_MARGIN;
  const int dy = g_earRoi.y - EAR_DRIFT_MARGIN;

  earExtractGray(rgb, w, h, dx, dy, dw, dh, grayScratch);
#if EAR_LOCALIZER_VERSION >= 3
  g_earLocator.addFrame(grayScratch, dw, dh, timestampMs);
#else
  g_earLocator.addFrame(grayScratch, dw, dh);
#endif

  float peakCx, peakCy, conf;
  if (!g_earLocator.peak(dw, dh, EAR_MOTION_MIN_FRAMES, peakCx, peakCy, conf))
    return 0;

  g_earDriftState = EAR_DRIFT_IDLE;
  outConf = conf;

  if (conf < EAR_MOTION_MIN_CONF) return 1;

#if EAR_LOCALIZER_VERSION >= 4
  g_earLocator.refine(grayScratch, dw, dh,
                      g_earRefGray, g_earRefMask, g_earRefRow, peakCx, peakCy);
#endif

  int newX = (int)(dx + peakCx - g_earRoi.size / 2.0f);
  int newY = (int)(dy + peakCy - g_earRoi.size / 2.0f);
  if (newX + g_earRoi.size > w) newX = w - g_earRoi.size;
  if (newY + g_earRoi.size > h) newY = h - g_earRoi.size;
  if (newX < 0) newX = 0;
  if (newY < 0) newY = 0;

  if (newX == g_earRoi.x && newY == g_earRoi.y) return 1;
  g_earRoi.x = newX;
  g_earRoi.y = newY;
  return 2;
}

// ── Frame loading (same convention as session_replay.cpp) ──────────────────
struct Frame { uint32_t ts; std::string path; };

static std::vector<Frame> collectFrames(const std::string &dir) {
  std::vector<Frame> out;
#if defined(_WIN32)
  WIN32_FIND_DATAA fd;
  std::string pattern = dir + "\\*.jpg";
  HANDLE h = FindFirstFileA(pattern.c_str(), &fd);
  if (h == INVALID_HANDLE_VALUE) return out;
  do {
    std::string name = fd.cFileName;
    uint32_t ts = (uint32_t)strtoul(name.c_str(), nullptr, 10);
    out.push_back({ts, dir + "\\" + name});
  } while (FindNextFileA(h, &fd));
  FindClose(h);
#endif
  std::sort(out.begin(), out.end(), [](const Frame &a, const Frame &b) { return a.ts < b.ts; });
  return out;
}

static bool readMetaRoi(const std::string &path, int &x, int &y, int &size) {
  FILE *f = fopen(path.c_str(), "r");
  if (!f) return false;
  char line[128];
  bool haveX = false, haveY = false, haveSize = false;
  while (fgets(line, sizeof(line), f)) {
    int v;
    if (sscanf(line, "roi_x=%d", &v) == 1) { x = v; haveX = true; }
    else if (sscanf(line, "roi_y=%d", &v) == 1) { y = v; haveY = true; }
    else if (sscanf(line, "roi_size=%d", &v) == 1) { size = v; haveSize = true; }
  }
  fclose(f);
  return haveX && haveY && haveSize;
}

int main(int argc, char **argv) {
  if (argc < 4) {
    fprintf(stderr, "Usage: %s <frames_dir> <metadata.txt> <out.csv>\n", argv[0]);
    return 1;
  }
  std::vector<Frame> frames = collectFrames(argv[1]);
  if (frames.empty()) { fprintf(stderr, "ERROR: no frames in %s\n", argv[1]); return 1; }

  if (!readMetaRoi(argv[2], g_earRoi.x, g_earRoi.y, g_earRoi.size)) {
    fprintf(stderr, "ERROR: could not read roi_x/roi_y/roi_size from %s\n", argv[2]);
    return 1;
  }
  printf("Starting ROI (from metadata): x=%d y=%d size=%d\n", g_earRoi.x, g_earRoi.y, g_earRoi.size);
  printf("Frames: %zu\n", frames.size());

#if EAR_LOCALIZER_VERSION >= 4
  const size_t refN = (size_t)EAR_REFINE_SIDE * EAR_REFINE_SIDE;
  g_earRefGray = (uint8_t *)malloc(refN);
  g_earRefMask = (uint8_t *)malloc(refN);
  g_earRefRow  = (uint16_t *)malloc(refN * sizeof(uint16_t));
#endif

  const int dw = EAR_ROI_SIZE + 2 * EAR_DRIFT_MARGIN;
  uint8_t *grayScratch = (uint8_t *)malloc((size_t)dw * dw);
  uint8_t *rgbBuf = nullptr;
  int rgbW = 0, rgbH = 0;

  FILE *out = fopen(argv[3], "w");
  fprintf(out, "timestamp_ms,roi_x,roi_y,event,conf\n");

  int nCorrected = 0, nNotConfident = 0, nCycles = 0;
  uint32_t t0 = frames.front().ts;

  for (const Frame &f : frames) {
    int w, h, comp;
    uint8_t *img = stbi_load(f.path.c_str(), &w, &h, &comp, 3);
    if (!img) { fprintf(stderr, "WARN: could not decode %s\n", f.path.c_str()); continue; }
    if (!rgbBuf || w != rgbW || h != rgbH) {
      free(rgbBuf);
      rgbBuf = (uint8_t *)malloc((size_t)w * h * 3);
      rgbW = w; rgbH = h;
    }
    memcpy(rgbBuf, img, (size_t)w * h * 3);
    stbi_image_free(img);

    float conf = 0.0f;
    int result = earDriftTick(rgbBuf, w, h, f.ts, grayScratch, conf);
    const char *event = "";
    if (result == 1) { event = "not_confident"; nNotConfident++; nCycles++; }
    else if (result == 2) { event = "corrected"; nCorrected++; nCycles++; }
    fprintf(out, "%u,%d,%d,%s,%.3f\n", f.ts, g_earRoi.x, g_earRoi.y, event, conf);

    if (result != 0) {
      printf("t=%6.1fs  cycle #%-3d  %-14s conf=%.2f  roi=(%d,%d)\n",
             (f.ts - t0) / 1000.0, nCycles, event, conf, g_earRoi.x, g_earRoi.y);
    }
  }

  fclose(out);
  printf("\nDone. %d cycles total: %d corrected, %d not confident.\n",
         nCycles, nCorrected, nNotConfident);
  return 0;
}
