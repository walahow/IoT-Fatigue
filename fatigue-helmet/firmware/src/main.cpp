/**
 * IoT Helmet — Fatigue Detection Firmware (Phase 2)
 * Board  : ESP32-S3 (Freenove WROOM CAM)
 * Sensors: Analog Pulse Sensor @ GPIO 1 (ADC1)
 *          MPU-6050 GY-521 (Accel + Gyro) @ I2C 0x68 (SDA=2, SCL=3)
 * Camera : OV2640 (configurable via build flags, default VGA @ 20 FPS, MJPEG
 * mode)
 *
 * ── Storage modes (select via PlatformIO environment) ──────────────────────
 *
 *   STORAGE_MODE_USB  (env:esp32s3cam)
 *     Camera frames sent as binary packets over USB-UART @ 921600 baud.
 *     Run debug_recorder.py on PC to demux and save:
 *       sessions/session_XXX/sensor_data.csv
 *       sessions/session_XXX/frames/{timestamp_ms}.jpg
 *
 *   STORAGE_MODE_SD   (env:esp32s3cam_sd)
 *     Camera frames + CSV saved directly to microSD card (SD_MMC 1-bit).
 *     SD pins (Freenove ESP32-S3-WROOM CAM, do not modify):
 *       CMD = GPIO 38  |  CLK = GPIO 39  |  D0 = GPIO 40
 *     Session folder structure:
 *       sessions/session_XXX/metadata.txt
 *       sessions/session_XXX/sensor_data.csv
 *       sessions/session_XXX/video.mjpeg   ← sequential JPEG stream (30 FPS)
 *       sessions/session_XXX/video.idx     ← sidecar:
 * frame_index,timestamp_ms,byte_offset,frame_size,crc32_hex
 *
 * Binary frame protocol (USB mode only):
 *   [0xAA 0xBB 0xCC 0xDD]  4 B  magic SOF
 *   [timestamp_ms]          4 B  little-endian uint32
 *   [jpeg_length]           4 B  little-endian uint32
 *   [JPEG data]             N B
 *   [0xDD 0xCC 0xBB 0xAA]  4 B  magic EOF
 */

#include "MPU6050.h" // electroniccats/MPU6050
#include "esp_camera.h"
#include "esp_task_wdt.h"  // TWDT reset to prevent Core 1 reboot
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include <math.h>
#include "FuzzyFatigue.h"  // Mamdani FIS (heap-free, STL-free, header-only)

#if defined(STORAGE_MODE_SD) || defined(EAR_LIVE_DEBUG)
#include "EyeBlinkEAR.h"        // on-device blink detection (spec section 5)
#include "img_converters.h"     // fmt2rgb888() -- esp32-camera
#include "esp_heap_caps.h"      // heap_caps_malloc() / MALLOC_CAP_SPIRAM
#endif

// ── Camera resolution / FPS defaults ─────────────────────────────────────
// Override via build_flags in platformio.ini, e.g.:
//   -DCAMERA_FRAMESIZE=FRAMESIZE_VGA  -DCAMERA_FPS=20
// Supported: FRAMESIZE_QVGA (320x240), FRAMESIZE_CIF (400x296),
//            FRAMESIZE_HVGA (480x320), FRAMESIZE_VGA (640x480)
// NOTE: SD_MMC 1-bit (~3-4 MB/s) limits throughput.
//       VGA@20fps ≈ 15-25 KB/frame x 20 = 300-500 KB/s -- well within budget.
//       SVGA or higher will cause frame drops on SD writes.
#ifndef CAMERA_FRAMESIZE
#define CAMERA_FRAMESIZE FRAMESIZE_VGA
#endif
#ifndef CAMERA_FPS
#define CAMERA_FPS 20
#endif
#ifndef CAMERA_JPEG_QUALITY
#define CAMERA_JPEG_QUALITY 10  // 0=best, 63=worst
#endif

// ── SD_MMC (production mode) ─────────────────────────────────────────────
#if defined(STORAGE_MODE_SD)
#include "SD_MMC.h"
// SD card pins (Freenove ESP32-S3-WROOM CAM — do not modify)
#define SD_MMC_CMD_PIN 38
#define SD_MMC_CLK_PIN 39
#define SD_MMC_D0_PIN 40
#endif

// ── Camera Pins (Freenove ESP32-S3-WROOM CAM) ───────────────────────────
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

// ── Pulse Sensor constants ────────────────────────────────────────────────
#define PULSE_PIN 1             // GPIO 1, ADC1 channel 0 on ESP32-S3
#define SAMPLE_RATE_MS 2        // 2 ms = 500 Hz sampling
#define BPM_BUFFER_SIZE 8       // beats averaged for stable BPM output
#define SIGNAL_LOW_THRESH 200   // ADC < this → likely no skin contact
#define SIGNAL_HIGH_THRESH 3900 // ADC > this → sensor saturated
#define MIN_BEAT_INTERVAL 500   // ms → 120 BPM max (prevents double-triggers)
#define MAX_BEAT_INTERVAL 2000  // ms → 30 BPM min

// ── Alert output ──────────────────────────────────────────────────────────────
// GPIO 14 confirmed free: no conflict with camera (4-18), I2C (2,3), SD (38-40),
// or pulse sensor (1). See fuzzy_walkthrough.md §7 for full GPIO audit.
#define BUZZER_PIN 14
#define MIN_AMPLITUDE 20        // min peak-to-valley swing
#define BEAT_TIMEOUT_MS 5000    // ms with no beat → reset BPM



// ── Session Control ──────────────────────────────────────────────────────────
#define BUTTON_PIN 21

// Session lifecycle. The button no longer flips recording on directly:
// pressing it ARMS the rig, which re-establishes the three things a usable
// session needs -- eye ROI lock, HR baseline, IMU calibration for the pose the
// helmet is ACTUALLY in -- and only then starts writing. Recording with any of
// those missing produces a session that cannot be used for training, which is
// the failure this state machine exists to prevent.
//
//   IDLE --press--> ARMING --all ready | timeout--> RECORDING --press--> IDLE
//                     |                                        (closeSession)
//                     +--press = abort--> IDLE
enum SessionState {
  SESSION_IDLE = 0,
  SESSION_ARMING,
  SESSION_RECORDING
};

// Give up waiting and record anyway after this long. A subsystem that never
// goes ready (eye lock in poor light, bad pulse contact) must not cost the
// whole session -- the readiness flags are written into metadata.txt instead,
// so the recording can be judged afterward rather than silently trusted.
static const uint32_t ARMING_TIMEOUT_MS = 60000;

SessionState g_sessionState = SESSION_IDLE;
uint32_t     g_armStartMs   = 0;
bool         g_armTimedOut  = false;

// Per-subsystem readiness, latched at the moment recording begins.
bool g_readyImu = false;
bool g_readyHr  = false;
bool g_readyEye = false;

// Derived from g_sessionState so the existing consumers (saveJpegToSD, the CSV
// writer, the buzzer gate) keep working unchanged.
#if defined(STORAGE_MODE_USB)
bool g_sessionActive = true;  // Start streaming immediately in USB debug mode
#else
bool g_sessionActive = false; // Start paused in SD card mode
#endif
bool g_lastButtonState = HIGH;
unsigned long g_lastDebounceTime = 0;
int g_sessionBeepState = 0;
unsigned long g_sessionBeepTimer = 0;

// ── I2C & output timing constants ────────────────────────────────────────
const uint8_t PIN_SDA = 2;
const uint8_t PIN_SCL = 3;
const uint32_t LOOP_MS = 1000; // 1 Hz CSV output rate

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ── MPU-6050 constants ────────────────────────────────────────────────────
const uint8_t MPU_ADDR = 0x68;
const float ACCEL_SCALE = 16384.0f; // LSB/g at ±2 g range
const float GYRO_SCALE = 131.0f;    // LSB/(°/s) at ±250 °/s range
const int CALIB_SAMPLES = 200;

// ── Pulse sensor globals ─────────────────────────────────────────────────
// Hardware capacitor handles DC bias removal — only a single low-pass IIR
// needed.
float iirLP = 2048.0f; // single-pole LP state; init at ADC midpoint
long beatIntervals[BPM_BUFFER_SIZE] = {0};
int beatIndex = 0;
long lastBeatTime = 0;
float currentBPM = 0;
bool risingSignal = false;
int dynamicThreshold = 2048;
float peakValue = 2048.0f;
float valleyValue = 2048.0f;
// τ ≈ 200 ms at 500 Hz (0.990 → 1/(1-0.990) × 2ms ≈ 200 ms)
const float ENVELOPE_DECAY = 0.990f;

// Shared output state — written by readPulseSensor(), read in loop output
int lastPulseRaw = 0;
int lastSignalQuality = 0;

// ── No-contact warning state ──────────────────────────────────────────────
int noContactStreak = 0;
const int NO_CONTACT_WARN_N = 10;

// ── MPU-6050 ─────────────────────────────────────────────────────────────
MPU6050 mpu(MPU_ADDR);
bool g_mpuEnabled = false;
bool g_buzzerActive = false;
// Mute the physical buzzer without breaking the I2C-suspend-during-EMI logic:
// setBuzzerState() ANDs this in, so g_buzzerActive only goes true when the
// pin is actually driven HIGH. Re-muted to isolate whether the buzzer being
// on the shared 3V/capacitor rail (with pulse sensor + IMU) is what's
// causing the IMU to drop out in production firmware -- see the
// camera-then-IMU-then-SD boot order + shared-rail discussion.
bool g_buzzerMuted = true;
int16_t ax_off = 0, ay_off = 0, az_off = 0;
int16_t gx_off = 0, gy_off = 0, gz_off = 0;

// ── Serial mutex (USB mode: protects binary frame interleaving) ───────────
SemaphoreHandle_t g_serialMutex = nullptr;

// ── SD card globals (SD mode only) ───────────────────────────────────────
#if defined(STORAGE_MODE_SD)
SemaphoreHandle_t g_sdMutex = nullptr;
File g_csvFile;            // sensor_data.csv — kept open entire session
File g_mjpegFile;          // video.mjpeg     — kept open entire session
File g_idxFile;            // video.idx       — kept open entire session
uint32_t g_byteOffset = 0; // running byte offset into video.mjpeg
uint32_t g_frameIndex = 0; // monotonic frame counter
bool g_sdReady = false;
#endif

// ── On-device EAR blink detection state (SD mode, or USB + EAR_LIVE_DEBUG) ──
// See docs/superpowers/specs/2026-09-01-on-device-fatigue-detection-design.md §5.
// EAR_LIVE_DEBUG (env:esp32s3cam_ear_preview) runs this same pipeline over
// USB video streaming instead of SD writes, at the same QVGA resolution the
// SD build uses -- NOT at the USB build's default VGA. The ~97 ms/frame
// decode cost that's already proven safe at QVGA (see processEarFrame's
// note) would be ~4x worse at VGA and risk the exact watchdog boot-loop
// cameraTask's yield comment describes; that's why esp32s3cam_ear_preview
// overrides CAMERA_FRAMESIZE back down to QVGA in platformio.ini.
#if defined(STORAGE_MODE_SD) || defined(EAR_LIVE_DEBUG)
// Every captured frame is processed. The glint check that replaced the old
// shape math is one compare+increment per pixel, so the per-frame cost is
// now dominated entirely by the JPEG decode that already had to happen.
// This ALSO has to be 1: GlintBlinkDetector's "absent for >= 2 consecutive
// frames" rule counts PROCESSED frames, and this hardware really captures
// at ~8 fps (measured on session_040: 480 frames / 60.9 s), so a typical
// 250-300 ms blink spans only ~2 frames. Any throttling drops it into the
// sampling gap entirely.
static const int   EAR_THROTTLE_DIV   = 1;   // every frame -- affordable at QVGA
                                              // the decode is half-scale (see processEarFrame)
// Sized to the eye region rather than tightly to the pupil, so normal gaze
// movement keeps the corneal reflection inside the crop.
static const int   EAR_ROI_SIZE       = 48;   // QVGA: same physical eye area as 96px at VGA
static const int   EAR_DRIFT_PERIOD   = 40;    // processed frames between drift checks
static const int   EAR_DRIFT_MARGIN   = 30;    // px added around the ROI when searching for drift
static const float EAR_DRIFT_MAX_PX   = 20.0f;
static const float EAR_DRIFT_ALPHA    = 0.3f;
// Retrospective validation against a real session (2026-09-03, see
// session_replay tool) found the periodic drift-check above can confidently
// re-lock onto hair occluding the camera later in a session -- hair is a
// genuinely darker/more stable feature than the eye once it drifts into
// frame, so the Otsu+centroid drift check has no way to tell the difference.
// That's a different problem (occlusion) than what drift-correction was
// built to solve (small physical helmet shift). Disabled until an
// occlusion-robust version exists: ship the validated boot lock, frozen for
// the session, rather than let a "correction" actively track the wrong
// thing.
static const bool  EAR_DRIFT_ENABLED  = false;

// ── Motion-energy boot lock ──────────────────────────────────────────────
// Replaces an earlier "largest Otsu-thresholded blob in the middle 60% of
// the frame" lock, which retrospective validation (session_023) found
// consistently latched onto brow/cheek shadow instead of the eye (59px off
// the real eye centre, 0.24 IoU with a Haar-verified reference box) -- Otsu
// always finds SOME dark region, with no eye-specific prior at all.
//
// The eye is the one region that's normally in the camera's field of view.
// In a helmet-fixed shot, cheek/brow/hair are all rigidly attached to the
// head and move with the camera; only the eyelid moves independently. See
// EyeBlinkEAR::MotionLocator for the implementation; validated on the same
// session: locked within 2.8px of the true eye centre on the first try,
// confidence 5.04 against a 2.0 gate.
#if EAR_LOCALIZER_VERSION >= 3
// v3 gates on EAR_V3_WINDOW_MS of wall clock (20 s), which holds the number of
// blinks in the window constant across frame rates. This is only the frame
// FLOOR beneath that: enough samples to resolve a 100-300 ms blink at all. It
// binds only below 6 fps, where 20 s would yield fewer than 120 frames.
static const int   EAR_MOTION_MIN_FRAMES = 120;
#elif EAR_LOCALIZER_VERSION == 2
// 240 frames is ~20 s at this camera's real ~12 fps, which contains about
// five blinks at a normal 14/min. The old 60 (~5 s) contained roughly ONE,
// and a single blink is indistinguishable from a single stray light
// movement anywhere else in the frame -- which is exactly how session_100
// locked 252 px from the pupil and recorded zero blinks. Measured across
// sessions 023/040/067/100, this one change drops mean lock error from
// 74.6 px to 14.4 px. Do not shorten it without re-running that set.
//
// The arming phase waits up to ARMING_TIMEOUT_MS (60 s) for the eye lock,
// so a ~20 s window fits comfortably inside a normal session start.
static const int   EAR_MOTION_MIN_FRAMES = 240;
#else
static const int   EAR_MOTION_MIN_FRAMES = 60;    // ~3s of processed frames
#endif
static const float EAR_MOTION_MIN_CONF   = 2.0f;  // reject flat/smeared maps
static const int   EAR_MOTION_MAX_TRIES  = 12;    // accept a weak lock rather than never locking

static uint8_t *g_earRgbBuf  = nullptr;  // PSRAM, fullW*fullH*3 (decoded JPEG)
static uint8_t *g_earGrayBuf = nullptr;  // PSRAM, fullW*fullH, reused at partial size
static uint8_t *g_earMaskBuf = nullptr;  // PSRAM, fullW*fullH, reused at partial size
static int      g_earFullW   = 0;
static int      g_earFullH   = 0;

static uint32_t g_earFrameCounter = 0;
static uint32_t g_earDriftCounter = 0;

static bool  g_earLockDone = false;
static bool  g_earDisabled = false;  // set true after a failed buffer allocation; never retried
static int   g_earMotionTries = 0;
static float g_earLockConfidence = 0.0f;

// ~28 KB -- global, not a local/stack variable (see MotionLocator's own note).
static EyeBlinkEAR::MotionLocator g_earLocator;
#if EAR_LOCALIZER_VERSION >= 4
// Scratch for v4's darkest-blob re-centre, allocated in PSRAM alongside the
// other EAR buffers -- 25 KB at EAR_ROI_SIZE 48, which does not belong in the
// 320 KB of internal DRAM when PSRAM is right there. Used once, at lock.
static const int EAR_REFINE_SIDE = EAR_ROI_SIZE + 2 * EAR_V4_MARGIN;
static uint8_t  *g_earRefGray = nullptr;
static uint8_t  *g_earRefMask = nullptr;
static uint16_t *g_earRefRow  = nullptr;
#endif

// Scratch for isolateLargestComponent(), sized to the locked EAR crop
// (EAR_ROI_SIZE^2) -- fixed at compile time, so plain static arrays rather
// than a PSRAM heap_caps_malloc like the full-frame buffers above.
static int16_t  g_earLabelScratch[EAR_ROI_SIZE * EAR_ROI_SIZE];
static uint16_t g_earQueueScratch[EAR_ROI_SIZE * EAR_ROI_SIZE];
static uint8_t  g_earIsolatedMask[EAR_ROI_SIZE * EAR_ROI_SIZE];

EyeBlinkEAR::RoiLock      g_earRoi;
EyeBlinkEAR::BlinkDetector g_earBlink;        // legacy shape-based path, no longer driving alerts
EyeBlinkEAR::GlintBlinkDetector g_earGlint;   // active blink detector
volatile float g_onDeviceBlinkRate = 13.0f;  // read by the g_blinkRate fallback below

// Converts a rectangular region of an RGB888 buffer to grayscale (simple
// average of R,G,B) into a caller-provided buffer sized regionW*regionH.
// Out-of-frame coordinates clamp to the nearest edge.
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
// ─────────────────────────────────────────────────────────────────────────
// earPrintf() — serial output from the EAR pipeline, mutex-protected.
//
// In USB mode this task is also streaming binary JPEG frames, and
// sendJpegFrame() takes g_serialMutex around each one. These status prints
// did not, so a line like "#STATUS: EAR ROI motion-locked at ..." was routinely
// cut in half by frame bytes landing mid-write. The firmware was reporting the
// lock correctly; the line simply never arrived intact, so live_ear_preview.py
// never matched it and a working localizer looked completely dead through two
// preview sessions.
//
// Same mutex, same 20 ms timeout as sendJpegFrame. On timeout the message is
// dropped rather than emitted corrupt -- a lost status line is recoverable, a
// shredded one poisons the parser. In SD mode the mutex still exists (setup()
// creates it in both modes) so this path is identical there.
// ─────────────────────────────────────────────────────────────────────────
static void earPrintf(const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (g_serialMutex == nullptr) {   // before setup() creates it
    Serial.print(buf);
    return;
  }
  if (xSemaphoreTake(g_serialMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    Serial.print(buf);
    xSemaphoreGive(g_serialMutex);
  }
}


void processEarFrame(camera_fb_t *fb, uint32_t timestampMs) {
  int w = (int)fb->width;
  int h = (int)fb->height;

  if (!g_earRgbBuf && !g_earDisabled) {
    // Half-scale RGB565: (w/2)*(h/2)*2 bytes, vs the old w*h*3 full-scale
    // RGB888 -- 6x smaller, and ~4x less decode work (measured below).
    g_earRgbBuf  = (uint8_t *)heap_caps_malloc((size_t)w * h * 3, MALLOC_CAP_SPIRAM);
    g_earGrayBuf = (uint8_t *)heap_caps_malloc((size_t)w * h, MALLOC_CAP_SPIRAM);
    g_earMaskBuf = (uint8_t *)heap_caps_malloc((size_t)w * h, MALLOC_CAP_SPIRAM);
    g_earFullW = w;
    g_earFullH = h;
#if EAR_LOCALIZER_VERSION >= 4
    const size_t refN = (size_t)EAR_REFINE_SIDE * EAR_REFINE_SIDE;
    g_earRefGray = (uint8_t *)heap_caps_malloc(refN, MALLOC_CAP_SPIRAM);
    g_earRefMask = (uint8_t *)heap_caps_malloc(refN, MALLOC_CAP_SPIRAM);
    g_earRefRow  = (uint16_t *)heap_caps_malloc(refN * sizeof(uint16_t),
                                                MALLOC_CAP_SPIRAM);
    if (!g_earRefGray || !g_earRefMask || !g_earRefRow) {
      // Not fatal: refine() no-ops on a null buffer and the lock falls back to
      // the raw motion peak, which is exactly v3's behaviour.
      earPrintf("#WARN: EAR refine buffers unavailable -- v4 re-centre disabled\n");
    }
#endif
    if (!g_earRgbBuf || !g_earGrayBuf || !g_earMaskBuf) {
      earPrintf("#ERROR: EAR buffer alloc failed -- on-device blink detection disabled\n");
      g_earDisabled = true;
      return;
    }
  }
  if (g_earDisabled) return;
  if (w != g_earFullW || h != g_earFullH) return;  // frame size changed mid-session, skip

  // Full-scale RGB888. A half-scale jpg2rgb565(JPG_SCALE_2X) path was tried
  // to cut this cost and MEASURED ON HARDWARE at ~267 ms per call -- nearly
  // 3x SLOWER than this full-scale decode's ~97 ms, not 4x faster as the
  // pixel count suggested. Do not "optimise" it back to a scaled decode
  // without profiling again.
  if (!fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, g_earRgbBuf)) {
    // A decode failure used to return in total silence, so a build where EVERY
    // frame failed to decode looked identical to one that was simply still
    // accumulating: perfect video, perfect sensors, and no eye output ever.
    // Report it, throttled, so that state is visible.
    static uint32_t decodeFails = 0;
    if (++decodeFails % 20 == 1) {
      earPrintf("#WARN: EAR jpeg decode failed (%lu so far)\n",
                    (unsigned long)decodeFails);
    }
    return;  // try again next frame
  }

#if defined(EAR_ROI_MANUAL_X)
  if (!g_earRoi.locked) {
    g_earRoi.x = EAR_ROI_MANUAL_X;
    g_earRoi.y = EAR_ROI_MANUAL_Y;
    g_earRoi.size = EAR_ROI_SIZE;
    g_earRoi.locked = true;
    g_earLockDone = true;
    earPrintf("#STATUS: EAR ROI manually pinned at x=%d y=%d size=%d\n",
                  g_earRoi.x, g_earRoi.y, g_earRoi.size);
  }
#endif

  if (!g_earLockDone) {
    // Progress while the localizer accumulates. Without this the lock phase is
    // completely silent until it succeeds or is rejected, which on a build
    // that never reaches either is indistinguishable from the pipeline not
    // running at all -- exactly the ambiguity that made a live preview session
    // impossible to interpret.
    static uint32_t lastLockReport = 0;
    if (timestampMs - lastLockReport >= 2000) {
      lastLockReport = timestampMs;
      earPrintf("#EARLOCK: accumulating frames=%d elapsed=%lums tries=%d\n",
                    g_earLocator.framesAccumulated,
                    (unsigned long)(g_earLocator.haveTime
                                    ? (g_earLocator.lastMs - g_earLocator.firstMs) : 0),
                    g_earMotionTries);
    }

    // Full-frame motion energy -- no assumption about where in the frame
    // the eye sits (session_023 found it was NOT centered).
    earExtractGray(g_earRgbBuf, w, h, 0, 0, w, h, g_earGrayBuf);
#if EAR_LOCALIZER_VERSION >= 2
    // Tell v2+ the ROI size so it can reject a peak whose ROI would hang off
    // the frame, instead of clamping it inward the way v1 did. Cheap enough
    // to set every frame; v1 has no such setter, by design.
    g_earLocator.setRoiSize(EAR_ROI_SIZE);
#endif
#if EAR_LOCALIZER_VERSION >= 3
    // v3 measures its lock window on the wall clock, so it needs the frame's
    // timestamp. Without one it would silently fall back to frame counting.
    g_earLocator.addFrame(g_earGrayBuf, w, h, timestampMs);
#else
    g_earLocator.addFrame(g_earGrayBuf, w, h);
#endif

    float mcx, mcy, conf;
    if (g_earLocator.peak(w, h, EAR_MOTION_MIN_FRAMES, mcx, mcy, conf)) {
      if (conf >= EAR_MOTION_MIN_CONF || ++g_earMotionTries >= EAR_MOTION_MAX_TRIES) {
#if EAR_LOCALIZER_VERSION >= 4
        // Motion energy peaks on the moving eyelid, a few pixels off the
        // pupil. Re-centre before locking so the detector's cues are taken
        // over a crop the pupil sits in the middle of.
        g_earLocator.refine(g_earGrayBuf, w, h,
                            g_earRefGray, g_earRefMask, g_earRefRow, mcx, mcy);
#endif
        float xs[1] = {mcx}, ys[1] = {mcy};
        EyeBlinkEAR::lockRoiFromSamples(xs, ys, 1, EAR_ROI_SIZE, w, h, g_earRoi);
        g_earLockDone = true;
        g_earLockConfidence = conf;
        earPrintf("#STATUS: EAR ROI motion-locked at x=%d y=%d size=%d conf=%.2f (%d tries)\n",
                      g_earRoi.x, g_earRoi.y, g_earRoi.size, conf, g_earMotionTries + 1);
      } else {
        earPrintf("#STATUS: EAR motion lock rejected conf=%.2f (<%.2f), retrying\n",
                      conf, EAR_MOTION_MIN_CONF);
        g_earLocator.reset();
      }
    }
    return;
  }

  if (EAR_DRIFT_ENABLED && ++g_earDriftCounter % EAR_DRIFT_PERIOD == 0) {
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
  // Blink detection is glint-based, not shape-based. Every shape metric
  // tried before this (whole-mask moments, largest-component moments,
  // contour+fitEllipse) was validated against real footage and found to
  // track lighting gradients across the iris and crop-edge clipping rather
  // than eyelid state -- all of them fired on a wide-open eye. The corneal
  // reflection simply vanishes when the lid covers the cornea, which none
  // of those failure modes affect. See GlintBlinkDetector's doc comment.
  int glintPx = 0, roiBrightness = 0;
  EyeBlinkEAR::glintAndBrightness(
      g_earGrayBuf, g_earRoi.size, g_earRoi.size,
      EyeBlinkEAR::GlintBlinkDetector::GLINT_LEVEL, glintPx, roiBrightness);

  // Cue 3 input: dark-pupil presence in a WIDER neighbourhood than the glint
  // ROI. Distinguishes a real closure (pupil gone) from a gaze shift (pupil
  // merely moved out of the ROI) -- see countDarkPixels()'s doc comment.
  const int WIDE_PAD = EAR_ROI_SIZE / 2;
  int wx = g_earRoi.x - WIDE_PAD, wy = g_earRoi.y - WIDE_PAD;
  int ww = g_earRoi.size + 2 * WIDE_PAD, wh = g_earRoi.size + 2 * WIDE_PAD;
  earExtractGray(g_earRgbBuf, w, h, wx, wy, ww, wh, g_earGrayBuf);
  int wideDarkPx = EyeBlinkEAR::countDarkPixels(
      g_earGrayBuf, ww, wh,
      EyeBlinkEAR::GlintBlinkDetector::DARK_OFFSET_BELOW_MEAN);

  if (g_earGlint.update(glintPx, roiBrightness, wideDarkPx, timestampMs)) {
    earPrintf("#STATUS: blink (glint) t=%u\n", timestampMs);
  }
  g_onDeviceBlinkRate = g_earGlint.rollingRateBpm(timestampMs);
}
#endif  // STORAGE_MODE_SD || EAR_LIVE_DEBUG

// ── Frame injection mode: PC feeds recorded JPEGs one at a time; the ESP
// runs the SAME processEarFrame() a live capture would, on the real chip,
// so pre-recorded footage can validate the actual compiled binary instead
// of a PC recompile of the same algorithm (see session_replay tool for
// that PC-side variant). No live camera task runs in this mode -- Serial
// is exclusively the injection protocol's, so it must not also be drained
// by the "BLINK:%f" text parser in loop() (see that block's own guard).
#if defined(FRAME_INJECT_MODE)
static const uint8_t INJECT_SOF[4] = {0xAA, 0xBB, 0xCC, 0xDD};
static const uint8_t INJECT_EOF[4] = {0xDD, 0xCC, 0xBB, 0xAA};

// Blocking read of exactly n bytes, tolerant of USB CDC arriving in small
// chunks. Returns false on a 5s stall (lets the caller resync on SOF
// rather than wedge forever on a dropped/corrupt packet).
static bool injectReadExact(uint8_t *dst, size_t n) {
  size_t got = 0;
  uint32_t lastProgress = millis();
  while (got < n) {
    int avail = Serial.available();
    if (avail > 0) {
      size_t want = n - got;
      int toRead = (int)((size_t)avail < want ? (size_t)avail : want);
      int r = Serial.readBytes((char *)(dst + got), toRead);
      if (r > 0) {
        got += (size_t)r;
        lastProgress = millis();
      }
    } else {
      // No bytes yet -- yield so IDLE0 gets scheduled (same watchdog trap
      // cameraTask's own yield comment describes: a tight poll loop with
      // no delay starves IDLE0 on Core 0 and the task watchdog aborts).
      vTaskDelay(pdMS_TO_TICKS(1));
      if (millis() - lastProgress > 5000) return false;
    }
  }
  return true;
}

void frameInjectTask(void *arg) {
  Serial.println(F("#STATUS: Frame-injection mode ready. Waiting for frames..."));
  static uint8_t *jpegBuf = nullptr;
  static size_t   jpegBufCap = 0;

  while (true) {
    uint8_t sof[4];
    if (!injectReadExact(sof, 4)) continue;
    if (memcmp(sof, INJECT_SOF, 4) != 0) continue;  // resync byte-by-byte

    uint8_t header[16];  // ts(4) + width(4) + height(4) + jpeg_len(4), all LE u32
    if (!injectReadExact(header, sizeof(header))) continue;
    uint32_t ts, w, h, len;
    memcpy(&ts,  header,      4);
    memcpy(&w,   header + 4,  4);
    memcpy(&h,   header + 8,  4);
    memcpy(&len, header + 12, 4);

    if (len == 0 || len > 400000) {
      Serial.println(F("#ERROR: bad frame length, resyncing"));
      continue;
    }
    if (len > jpegBufCap) {
      if (jpegBuf) free(jpegBuf);
      jpegBuf = (uint8_t *)heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
      jpegBufCap = jpegBuf ? len : 0;
    }
    if (!jpegBuf || !injectReadExact(jpegBuf, len)) {
      Serial.println(F("#ERROR: frame read failed, resyncing"));
      continue;
    }
    uint8_t eof[4];
    if (!injectReadExact(eof, 4) || memcmp(eof, INJECT_EOF, 4) != 0) {
      Serial.println(F("#ERROR: bad EOF marker, resyncing"));
      continue;
    }

    camera_fb_t fb = {};
    fb.buf    = jpegBuf;
    fb.len    = len;
    fb.width  = w;
    fb.height = h;
    fb.format = PIXFORMAT_JPEG;

    processEarFrame(&fb, ts);
    Serial.printf("#FRAME_DONE ts=%u\n", ts);
  }
}
#endif  // FRAME_INJECT_MODE

// ── USB mode: binary frame constants ─────────────────────────────────────
#if defined(STORAGE_MODE_USB)
static const uint8_t FRAME_SOF[4] = {0xAA, 0xBB, 0xCC, 0xDD};
static const uint8_t FRAME_EOF[4] = {0xDD, 0xCC, 0xBB, 0xAA};
#endif

// ─────────────────────────────────────────────────────────────────────────
// Nodding Oscillation Detector
// 6-second rolling pitch buffer at 10 Hz → 60 samples.
// Algorithm: detrend → ZCR → slope gate → clamp(ZCR/4, 0, 1).
// Engineering assumption: nodding frequency 0.5–2 Hz.
// See fuzzy_walkthrough.md §2.3 for full rationale.
// ─────────────────────────────────────────────────────────────────────────
struct NodDetector {
    static const uint8_t N = 60;   // 6 s × 10 Hz
    float   buf[N];
    uint8_t head;
    uint8_t count;   // saturates at N

    NodDetector() : head(0), count(0) { memset(buf, 0, sizeof(buf)); }

    void push(float pitch_deg) {
        buf[head] = pitch_deg;
        head      = (head + 1) % N;
        if (count < N) count++;
    }

    // Returns nodding_score [0..1].
    // Chronological index: count<N → oldest at buf[0]; count==N → oldest at buf[head].
    // Minimum peak-to-peak pitch swing (degrees) required within the 6 s
    // window before the zero-crossing score is trusted at all. Retrospective
    // hardware check (2026-09-04): sitting motionless, pitch noise stayed
    // under 0.5 deg peak-to-peak, yet the zero-crossing rate alone still hit
    // score=1.0 in >50% of samples -- the algorithm was correctly measuring
    // zero crossings, but at this amplitude it's measuring MEMS/quantization
    // noise, not head motion. mf_pitch_Limp (this file's other pitch-based
    // signal) already treats a real sustained head-drop as starting at 20 deg,
    // so a discrete nod -- faster and smaller than a full slump, but still a
    // deliberate real motion -- should clearly clear a couple of degrees.
    // 2.0 deg is a conservative floor: comfortably above the measured noise
    // floor, comfortably below a real nod. Needs retuning against real rider
    // footage, same as every other threshold in this file.
    static constexpr float MIN_SWING_DEG = 2.0f;

    float score() const {
        if (count < 6) return 0.0f;

        // Mean for detrending, and peak-to-peak swing for the noise gate below.
        float mean = 0.0f;
        float lo = buf[(count < N) ? 0 : head];
        float hi = lo;
        for (uint8_t i = 0; i < count; i++) {
            uint8_t idx = (count < N) ? i : (uint8_t)((head + i) % N);
            mean += buf[idx];
            if (buf[idx] < lo) lo = buf[idx];
            if (buf[idx] > hi) hi = buf[idx];
        }
        mean /= (float)count;
        if ((hi - lo) < MIN_SWING_DEG) return 0.0f;  // noise floor gate

        // Zero crossings of detrended signal
        int zcr = 0;
        uint8_t idx0 = (count < N) ? 0 : head;
        float prev = buf[idx0] - mean;
        for (uint8_t i = 1; i < count; i++) {
            uint8_t idx = (count < N) ? i : (uint8_t)((head + i) % N);
            float v = buf[idx] - mean;
            if ((prev < 0.0f) != (v < 0.0f)) zcr++;
            prev = v;
        }
        float zcr_per_s = (float)zcr / ((float)count / 10.0f);

        // Linear regression slope of raw pitch buffer.
        // slope > 0: pitch trending upward = head drooping forward.
        // (fuzzy_walkthrough.md spec uses slope < 0 with opposite sign convention;
        //  here pitch_deg = acos(...) >= 0, so drooping = increasing. Flip to
        //  slope < 0.0f if hardware tests show inverted behaviour.)
        float fn  = (float)count;
        float sx  = fn * (fn - 1.0f) / 2.0f;
        float sx2 = fn * (fn - 1.0f) * (2.0f * fn - 1.0f) / 6.0f;
        float sy  = 0.0f, sxy = 0.0f;
        for (uint8_t i = 0; i < count; i++) {
            uint8_t idx = (count < N) ? i : (uint8_t)((head + i) % N);
            sy  += buf[idx];
            sxy += (float)i * buf[idx];
        }
        float denom = fn * sx2 - sx * sx;
        float slope = (fabsf(denom) > 1e-9f) ? (fn * sxy - sx * sy) / denom : 0.0f;
        float gate  = (slope > 0.0f) ? 1.0f : 0.0f;

        float s = zcr_per_s / 4.0f;
        if (s < 0.0f) s = 0.0f;
        if (s > 1.0f) s = 1.0f;
        return s * gate;
    }
};

// ─────────────────────────────────────────────────────────────────────────
// Gyro Variance Rolling Buffer
// 10-second rolling buffer of head_movement magnitude at 10 Hz → 100 samples.
// variance() feeds gyro_Stable / gyro_Fidgety input MFs of the FIS.
// ─────────────────────────────────────────────────────────────────────────
struct GyroVarBuf {
    static const uint8_t N = 100;   // 10 s × 10 Hz
    float   buf[N];
    uint8_t head;
    uint8_t count;

    GyroVarBuf() : head(0), count(0) { memset(buf, 0, sizeof(buf)); }

    void push(float v) {
        buf[head] = v;
        head      = (head + 1) % N;
        if (count < N) count++;
    }

    float variance() const {
        if (count < 2) return 0.0f;
        float mean = 0.0f;
        for (uint8_t i = 0; i < count; i++) {
            uint8_t idx = (count < N) ? i : (uint8_t)((head + i) % N);
            mean += buf[idx];
        }
        mean /= (float)count;
        float var = 0.0f;
        for (uint8_t i = 0; i < count; i++) {
            uint8_t idx = (count < N) ? i : (uint8_t)((head + i) % N);
            float d = buf[idx] - mean;
            var += d * d;
        }
        return var / (float)count;
    }
};

// ─────────────────────────────────────────────────────────────────────────
// FIS globals
// ─────────────────────────────────────────────────────────────────────────

FuzzyFatigue g_fis;
float g_riskScore  = 0.0f;
int   g_alertLevel = ALERT_SAFE;

// ── Baseline HR (count-based, frozen after N_BASELINE_SAMPLES valid readings) ─
// "valid" = signal_quality==1 AND currentBPM>0 (implicitly 30–120 BPM by
// beat detection: MIN_BEAT_INTERVAL=500 ms, MAX_BEAT_INTERVAL=2000 ms).
// Frozen — not rolling — so progressive drowsiness is NOT normalised out.
static const uint8_t N_BASELINE_SAMPLES = 20;   // ~20 s at 1 Hz with good contact
float    g_baselineSum    = 0.0f;
uint8_t  g_baselineCount  = 0;
float    g_baselineBPM    = 0.0f;
bool     g_baselineFormed = false;

// ── BLINK serial receive state ─────────────────────────────────────────────
// Parser drains UART ring buffer at each 1 Hz tick. Format: "BLINK:<float>\n"
// Default 13.0 bl/min = centre of Normal MF flat-top (8–18) → no false Warning.
// Sticky fallback: hold last valid after BLINK_TIMEOUT_MS of silence.
static const uint32_t BLINK_TIMEOUT_MS = 5000;
float    g_blinkRate        = 13.0f;
float    g_lastValidBlink   = 13.0f;
uint32_t g_lastBlinkRxTime  = 0;
bool     g_blinkEverRx      = false;
char     g_serialLineBuf[32];
uint8_t  g_serialLineBufLen = 0;

// ── Calibration gravity vector (raw ADC units, captured after calibrateMPU) ─
// dot-product pitch formula:
//   pitch = acos(dot(g_calibGrav, g_now) / (|g_calibGrav| × |g_now|)) × (180/π)
// Default assumes vertical Z axis; overwritten by calibrateMPU().
float g_calibGrav[3] = {0.0f, 0.0f, 16384.0f};

// ── Cached 10 Hz IMU snapshot (written by 10 Hz block, read at 1 Hz FIS tick) ─
float g_imu_ax_g      = 0.0f;
float g_imu_ay_g      = 0.0f;
float g_imu_az_g      = 0.0f;
float g_imu_gx_dp     = 0.0f;
float g_imu_gy_dp     = 0.0f;
float g_imu_gz_dp     = 0.0f;
float g_imu_head_mov  = 0.0f;
float g_imu_pitch_deg = 0.0f;
float g_imu_gyro_var  = 0.0f;
float g_imu_nod_score = 0.0f;
// Freshness of the cached snapshot above: false while reads are suspended
// (buzzer active) or the bus is down, meaning g_imu_* are frozen last-known-good
// values rather than a live reading.
bool     g_imuSampleValid   = false;
uint32_t g_lastImuValidTime = 0;

// ── 10 Hz ring buffer instances ───────────────────────────────────────────────
NodDetector g_nodDet;    //  60 samples × 4 B =  240 B BSS
GyroVarBuf  g_gyroVar;   // 100 samples × 4 B =  400 B BSS
                          // Total: ~640 B — negligible on ESP32-S3 (512 KB SRAM)

// ─────────────────────────────────────────────────────────────────────────
// CRC-32 (standard Ethernet polynomial 0xEDB88320)
// Matches Python: binascii.crc32(data) & 0xFFFFFFFF
// On ESP32-S3 @ 240 MHz: ~0.2 ms per 10 KB QVGA frame — negligible.
// ─────────────────────────────────────────────────────────────────────────

static uint32_t crc32_compute(const uint8_t *buf, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int j = 0; j < 8; j++)
      crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
  }
  return ~crc;
}

// ─────────────────────────────────────────────────────────────────────────
// SD Card helpers (SD mode only)
// ─────────────────────────────────────────────────────────────────────────

#if defined(STORAGE_MODE_SD)

bool initSDCard() {
  // Set SD_MMC pins before mounting (ESP32-S3 Arduino core ≥ 2.0.14)
  SD_MMC.setPins(SD_MMC_CLK_PIN, SD_MMC_CMD_PIN, SD_MMC_D0_PIN);

  // Mount in 1-bit mode (D0 only); format_if_fail = false
  if (!SD_MMC.begin("/sdcard", true, false)) {
    Serial.println(F("#ERROR: SD card mount failed — insert card and reset"));
    return false;
  }

  uint64_t cardSizeMB = SD_MMC.cardSize() / (1024ULL * 1024ULL);
  uint64_t freeMB =
      SD_MMC.totalBytes() > SD_MMC.usedBytes()
          ? (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024ULL * 1024ULL)
          : 0;
  Serial.printf("#STATUS: SD card mounted OK — %llu MB total, %llu MB free\n",
                cardSizeMB, freeMB);

  // Session creation deliberately does NOT happen here. It moved to
  // openSession(), called when RECORDING actually begins, so each
  // arm-record-stop cycle gets its own session_XXX folder. Creating the files
  // at boot bound a "session" to a power cycle and made a second recording
  // append into the first one's files, silently corrupting the dataset.
  return true;
}

// ─────────────────────────────────────────────────────────────────────────
// openSession() — create session_XXX and open its three files.
//
// Called at the ARMING -> RECORDING transition, not at boot, so the readiness
// of each subsystem is known and can be recorded in metadata.txt alongside the
// capture parameters. Returns false if any file could not be opened.
// ─────────────────────────────────────────────────────────────────────────
bool openSession() {
  if (!g_sdReady) return false;

  // ── Find next session number ─────────────────────────────────────────
  if (!SD_MMC.exists("/sessions")) {
    SD_MMC.mkdir("/sessions");
  }

  int sessionNum = 1;
  char sessionDir[40];
  while (true) {
    snprintf(sessionDir, sizeof(sessionDir), "/sessions/session_%03d",
             sessionNum);
    if (!SD_MMC.exists(sessionDir))
      break;
    sessionNum++;
  }

  // ── Create session directory (no frames/ subdir needed — MJPEG stream) ─
  SD_MMC.mkdir(sessionDir);
  Serial.printf("#STATUS: Session: %s\n", sessionDir);

  // ── Write metadata.txt ───────────────────────────────────────────────
  char metaPath[52];
  snprintf(metaPath, sizeof(metaPath), "%s/metadata.txt", sessionDir);
  File meta = SD_MMC.open(metaPath, FILE_WRITE);
  if (meta) {
    meta.printf("mode=production_sd\n");
    meta.printf("camera_fps=%d\n", CAMERA_FPS);
    // Resolution string derived from the framesize constant
    const char *resStr =
#if defined(CAMERA_FRAMESIZE) && CAMERA_FRAMESIZE == FRAMESIZE_VGA
        "640x480";
#elif defined(CAMERA_FRAMESIZE) && CAMERA_FRAMESIZE == FRAMESIZE_HVGA
        "480x320";
#elif defined(CAMERA_FRAMESIZE) && CAMERA_FRAMESIZE == FRAMESIZE_CIF
        "400x296";
#else
        "320x240"; // QVGA fallback
#endif
    meta.printf("camera_resolution=%s\n", resStr);
    meta.printf("video_file=video.mjpeg\n");
    meta.printf("index_file=video.idx\n");
    meta.printf("sensor_sample_rate=10Hz\n");
    meta.printf("mpu_address=0x68\n");
    meta.printf("pulse_pin=1\n");
    meta.printf("sd_cmd_pin=%d\n", SD_MMC_CMD_PIN);
    meta.printf("sd_clk_pin=%d\n", SD_MMC_CLK_PIN);
    meta.printf("sd_d0_pin=%d\n", SD_MMC_D0_PIN);

    // Readiness at the moment recording began. A 0 here means that subsystem
    // never went ready within ARMING_TIMEOUT_MS and the session started
    // anyway -- the data is still recorded, but this says which channels to
    // distrust rather than leaving it to be guessed later.
    meta.printf("armed_imu=%d\n", g_readyImu ? 1 : 0);
    meta.printf("armed_hr=%d\n",  g_readyHr  ? 1 : 0);
    meta.printf("armed_eye=%d\n", g_readyEye ? 1 : 0);
    meta.printf("arming_timed_out=%d\n", g_armTimedOut ? 1 : 0);
    meta.printf("arming_duration_ms=%lu\n",
                (unsigned long)(millis() - g_armStartMs));
    meta.close();
  }

  // ── Open sensor_data.csv ─────────────────────────────────────────────
  char csvPath[56];
  snprintf(csvPath, sizeof(csvPath), "%s/sensor_data.csv", sessionDir);
  g_csvFile = SD_MMC.open(csvPath, FILE_WRITE);
  if (!g_csvFile) {
    Serial.println(F("#ERROR: Could not open sensor_data.csv on SD"));
    return false;
  }

  // ── Open video.mjpeg (sequential JPEG stream) ─────────────────────────
  char mjpegPath[56];
  snprintf(mjpegPath, sizeof(mjpegPath), "%s/video.mjpeg", sessionDir);
  g_mjpegFile = SD_MMC.open(mjpegPath, FILE_WRITE);
  if (!g_mjpegFile) {
    Serial.println(F("#ERROR: Could not open video.mjpeg on SD"));
    return false;
  }

  // ── Open video.idx and write header ──────────────────────────────────
  char idxPath[52];
  snprintf(idxPath, sizeof(idxPath), "%s/video.idx", sessionDir);
  g_idxFile = SD_MMC.open(idxPath, FILE_WRITE);
  if (!g_idxFile) {
    Serial.println(F("#ERROR: Could not open video.idx on SD"));
    return false;
  }
  g_idxFile.println(
      F("frame_index,timestamp_ms,byte_offset,frame_size,crc32_hex"));

  // Reset session-level counters
  g_byteOffset = 0;
  g_frameIndex = 0;

  Serial.printf("#STATUS: MJPEG stream: %s\n", mjpegPath);
  Serial.printf("#STATUS: Index file : %s\n", idxPath);

  return true;
}


// ─────────────────────────────────────────────────────────────────────────
// closeSession() — flush + close all SD files before power-off / card removal.
// Call this on any graceful shutdown (GPIO button, low-battery ISR, etc.).
// ─────────────────────────────────────────────────────────────────────────
void closeSession() {
  if (xSemaphoreTake(g_sdMutex, portMAX_DELAY) == pdTRUE) {
    if (g_mjpegFile) {
      g_mjpegFile.flush();
      g_mjpegFile.close();
    }
    if (g_idxFile) {
      g_idxFile.flush();
      g_idxFile.close();
    }
    if (g_csvFile) {
      g_csvFile.flush();
      g_csvFile.close();
    }
    xSemaphoreGive(g_sdMutex);
  }
  Serial.println(F("#STATUS: Session closed — safe to remove SD card."));
}

// ─────────────────────────────────────────────────────────────────────────
// saveJpegToSD() — append one JPEG frame to video.mjpeg, log to video.idx.
// Called from cameraTask on Core 0. Never opens or closes a file per frame.
// ─────────────────────────────────────────────────────────────────────────
void saveJpegToSD(const uint8_t *data, size_t len, uint32_t timestamp_ms) {
  if (!g_sdReady || !g_mjpegFile || !g_idxFile || !g_sessionActive)
    return;
  static int flushCounter = 0;

  uint32_t crc = crc32_compute(data, len);

  if (xSemaphoreTake(g_sdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {

    // 1. Write JPEG bytes sequentially — check every byte was written
    size_t written = g_mjpegFile.write(data, len);
    if (written != len) {
      // SD full or I/O error: do NOT write an idx entry for this partial frame
      Serial.printf("#ERROR: SD write failed (wrote %zu of %zu B) "
                    "— frame %lu dropped. SD full?\n",
                    written, len, g_frameIndex);
      xSemaphoreGive(g_sdMutex);
      return;
    }

    // 2. Write sidecar index line
    //    columns: frame_index, timestamp_ms, byte_offset, frame_size, crc32_hex
    g_idxFile.printf("%lu,%lu,%lu,%zu,%08lx\n", g_frameIndex, timestamp_ms,
                     g_byteOffset, len, (unsigned long)crc);

    // 3. Advance session counters
    g_byteOffset += (uint32_t)len;
    g_frameIndex++;

    // 4. Flush both files every 30 frames (~1 second at 30 FPS)
    //    Flushing less often maximises SD sequential write throughput.
    if (++flushCounter % 30 == 0) {
      g_mjpegFile.flush();
      g_idxFile.flush();
    }

    xSemaphoreGive(g_sdMutex);
  }
  // If mutex not acquired within 50 ms, frame is silently dropped
  // (the camera will capture the next frame on schedule regardless)
}

#endif // STORAGE_MODE_SD

// ─────────────────────────────────────────────────────────────────────────
// USB mode: send JPEG as binary packet over Serial
// ─────────────────────────────────────────────────────────────────────────

#if defined(STORAGE_MODE_USB)
void sendJpegFrame(const uint8_t *data, size_t len, uint32_t timestamp_ms) {
  uint32_t length32 = (uint32_t)len;
  // Timeout 20ms: if sensor output holds the mutex, skip this frame
  if (xSemaphoreTake(g_serialMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    Serial.write(FRAME_SOF, 4);
    Serial.write((const uint8_t *)&timestamp_ms, 4);
    Serial.write((const uint8_t *)&length32, 4);
    
    // Send in 1024-byte chunks and yield to let the USB CDC task run on Core 0
    size_t offset = 0;
    const size_t chunkSize = 1024;
    while (offset < len) {
      size_t toWrite = (len - offset < chunkSize) ? (len - offset) : chunkSize;
      Serial.write(data + offset, toWrite);
      offset += toWrite;
      vTaskDelay(1); // Yields CPU for 1ms
    }
    
    Serial.write(FRAME_EOF, 4);
    xSemaphoreGive(g_serialMutex);
  }
}
#endif

// ─────────────────────────────────────────────────────────────────────────
// Camera task — Core 0 @ 10 fps
// ─────────────────────────────────────────────────────────────────────────

void cameraTask(void *arg) {
  const TickType_t period = pdMS_TO_TICKS(1000 / CAMERA_FPS); // target FPS
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t framesSent = 0;
  uint32_t framesDropped = 0;

  while (true) {
    vTaskDelayUntil(&lastWake, period);
    // vTaskDelayUntil does NOT block once the deadline has already passed,
    // and with fb_count=2 a frame is usually already queued so
    // esp_camera_fb_get() returns immediately too. With per-frame EAR work
    // (~97 ms) exceeding the period, that combination left this task with no
    // yield at all: IDLE0 on CPU 0 never ran and the task watchdog aborted
    // (observed as a boot loop). This unconditional short delay guarantees
    // the idle task gets scheduled every iteration no matter how the frame
    // timing lands.
    vTaskDelay(pdMS_TO_TICKS(5));

    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      if (fb->format == PIXFORMAT_JPEG) {
        uint32_t ts = (uint32_t)millis();

#if defined(STORAGE_MODE_SD)
        // Build with -DEAR_PROFILE to print the per-frame cost breakdown.
        // Off by default: it prints every 40 frames, which is serial noise
        // during a real recording. Kept because the numbers it produced are
        // what identified the watchdog cause (see cameraTask's yield note
        // and processEarFrame's decode-cost note) and will be needed again
        // if the camera resolution or pipeline cost changes.
#if defined(EAR_PROFILE)
        uint32_t tSd0 = (uint32_t)micros();
#endif
        saveJpegToSD(fb->buf, fb->len, ts);
#if defined(EAR_PROFILE)
        uint32_t tSd1 = (uint32_t)micros();
#endif
        if (++g_earFrameCounter % EAR_THROTTLE_DIV == 0) {
          processEarFrame(fb, ts);
        }
#if defined(EAR_PROFILE)
        uint32_t tEar1 = (uint32_t)micros();
        static uint32_t profCount = 0, profSd = 0, profEar = 0;
        profSd  += (tSd1 - tSd0);
        profEar += (tEar1 - tSd1);
        if (++profCount >= 40) {
          Serial.printf("#PROF: sd=%luus ear=%luus total=%luus budget=%dus\n",
                        (unsigned long)(profSd / profCount),
                        (unsigned long)(profEar / profCount),
                        (unsigned long)((profSd + profEar) / profCount),
                        (int)(1000000 / CAMERA_FPS));
          profCount = profSd = profEar = 0;
        }
#endif
#elif defined(STORAGE_MODE_USB)
        sendJpegFrame(fb->buf, fb->len, ts);
#if defined(EAR_LIVE_DEBUG)
        // Same on-device pipeline the SD build runs, just fed from the
        // frame we're also streaming out over serial instead of to SD.
        if (++g_earFrameCounter % EAR_THROTTLE_DIV == 0) {
          processEarFrame(fb, ts);
        }
#endif
#endif
        framesSent++;
      }
      esp_camera_fb_return(fb);
    } else {
      framesDropped++;
      if (framesDropped % 10 == 0) {
#if defined(STORAGE_MODE_USB)
        if (xSemaphoreTake(g_serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
#endif
          Serial.printf("#WARNING: Camera frame drops=%lu\n",
                        (unsigned long)framesDropped);
#if defined(STORAGE_MODE_USB)
          xSemaphoreGive(g_serialMutex);
        }
#endif
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────
// readPulseSensor() — called at 500 Hz
// ─────────────────────────────────────────────────────────────────────────

void readPulseSensor() {
  unsigned long sensorNow = millis();

  int raw = analogRead(PULSE_PIN);
  lastPulseRaw = raw;

  // Single-pole IIR low-pass — α = 0.15 → f_c ≈ 12 Hz @ 500 Hz.
  // Hardware capacitor already removes DC bias, so no HP stage is needed.
  // This passes the PPG fundamental (0.5–3.5 Hz) while killing ADC noise,
  // and preserves the natural waveform shape for clean threshold detection.
  const float LP_ALPHA = 0.15f;
  iirLP += LP_ALPHA * ((float)raw - iirLP);
  int filtered = constrain((int)iirLP, 0, 4095);

  // Adaptive envelope tracker.
  // Peak/valley decay toward midpoint (2048) at ENVELOPE_DECAY rate.
  // τ ≈ 200 ms — fast enough to follow finger-pressure changes.
  if ((float)filtered > peakValue)
    peakValue = (float)filtered;
  else
    peakValue = peakValue * ENVELOPE_DECAY + 2048.0f * (1.0f - ENVELOPE_DECAY);
  if ((float)filtered < valleyValue)
    valleyValue = (float)filtered;
  else
    valleyValue =
        valleyValue * ENVELOPE_DECAY + 2048.0f * (1.0f - ENVELOPE_DECAY);

  dynamicThreshold = (int)((peakValue + valleyValue) / 2.0f);

  // Rising-edge beat detection with refractory period
  bool wasRising = risingSignal;
  risingSignal = (filtered > dynamicThreshold);

  if (!wasRising && risingSignal) {
    long interval = sensorNow - lastBeatTime;
    if (lastBeatTime > 0 && interval >= MIN_BEAT_INTERVAL &&
        interval <= MAX_BEAT_INTERVAL) {
      beatIntervals[beatIndex % BPM_BUFFER_SIZE] = interval;
      beatIndex++;
      int count = (beatIndex < BPM_BUFFER_SIZE) ? beatIndex : BPM_BUFFER_SIZE;
      long totalMs = 0;
      for (int i = 0; i < count; i++)
        totalMs += beatIntervals[i];
      currentBPM = (60000.0f * count) / (float)totalMs;
      lastBeatTime = sensorNow;
    } else if (lastBeatTime == 0) {
      lastBeatTime = sensorNow;
    }
  }

  // Timeout: no beat detected for BEAT_TIMEOUT_MS → reset BPM
  if (lastBeatTime > 0 && (sensorNow - lastBeatTime > BEAT_TIMEOUT_MS)) {
    currentBPM = 0;
    beatIndex = 0;
    lastBeatTime = 0;
  }

  // Signal quality:
  //   0 = no skin contact (raw out of expected ADC window)
  //   0 = weak/noisy signal (amplitude below minimum swing)
  //   1 = good contact with detectable pulse
  int amplitude = (int)(peakValue - valleyValue);
  if (raw < SIGNAL_LOW_THRESH || raw > SIGNAL_HIGH_THRESH) {
    lastSignalQuality = 0; // no contact
  } else if (amplitude >= MIN_AMPLITUDE) {
    lastSignalQuality = 1; // strong enough signal
  } else {
    lastSignalQuality = 0; // weak/noisy — was incorrectly 1 (bug fix)
  }
}

// ─────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────

void blinkLED(int times) {
  if (LED_BUILTIN <= 0 || LED_BUILTIN == PIN_SDA || LED_BUILTIN == PIN_SCL) {
    return; // Avoid corrupting I2C bus pins (GPIO 2 / 3)
  }
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}

void setBuzzerState(bool active) {
  // g_buzzerMuted silences the pin for quiet testing, but g_buzzerActive still
  // reflects real hardware state -- it only goes true when the buzzer is
  // actually drawing power, which is exactly when I2C reads must be suspended.
  g_buzzerActive = active && !g_buzzerMuted;
  digitalWrite(BUZZER_PIN, g_buzzerActive ? HIGH : LOW);
}

void calibrateMPU() {
  Serial.println(F("#STATUS: Keep sensor still for calibration..."));
  for (int c = 3; c > 0; c--) {
    Serial.print(F("#STATUS: Starting in "));
    Serial.print(c);
    Serial.println(F("s..."));
    delay(1000);
  }
  Serial.println(F("#STATUS: Calibrating... keep still"));

  int16_t ax, ay, az, gx, gy, gz;
  long ax_s = 0, ay_s = 0, az_s = 0;
  long gx_s = 0, gy_s = 0, gz_s = 0;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_s += ax;
    ay_s += ay;
    az_s += az;
    gx_s += gx;
    gy_s += gy;
    gz_s += gz;
    delay(5);
  }

  ax_off = ax_s / CALIB_SAMPLES;
  ay_off = ay_s / CALIB_SAMPLES;
  az_off = az_s / CALIB_SAMPLES;
  gx_off = gx_s / CALIB_SAMPLES;
  gy_off = gy_s / CALIB_SAMPLES;
  gz_off = gz_s / CALIB_SAMPLES;

  Serial.print(F("#STATUS: Calibration done. Offsets: ax="));
  Serial.print(ax_off);
  Serial.print(F(" ay="));
  Serial.print(ay_off);
  Serial.print(F(" az="));
  Serial.print(az_off);
  Serial.print(F(" gx="));
  Serial.print(gx_off);
  Serial.print(F(" gy="));
  Serial.print(gy_off);
  Serial.print(F(" gz="));
  Serial.println(gz_off);

  // Capture calibration gravity vector (raw ADC units) for pitch computation.
  // ax_off/ay_off/az_off are the mean raw readings at calibration pose, which
  // include the gravity component — exactly the reference vector we need.
  g_calibGrav[0] = (float)ax_off;
  g_calibGrav[1] = (float)ay_off;
  g_calibGrav[2] = (float)az_off;
  Serial.printf("#STATUS: Calib gravity vector: [%.0f, %.0f, %.0f] raw ADC\n",
                g_calibGrav[0], g_calibGrav[1], g_calibGrav[2]);
}
// ─────────────────────────────────────────────────────────────────────────
// ARMING support — non-blocking IMU calibration + per-session resets
//
// calibrateMPU() above blocks for ~4 s (a 3 s countdown plus 200 x delay(5)).
// That is fine at boot but unusable during ARMING: loop() also drives the
// 500 Hz pulse sampler and feeds the task watchdog, so stalling it for 4 s
// would punch a hole in the pulse record and risk a watchdog reset. This
// accumulates the same 200 samples from the loop cadence instead.
// ─────────────────────────────────────────────────────────────────────────

// Reject a calibration taken while the helmet is moving. Offsets captured
// mid-motion bake that motion into every later reading, and a rider settling
// onto the bike is exactly when arming happens. Thresholds are generous
// against this clone's ~3 dps gyro noise (see IMU_INTEGRATION.md section 2.3).
static const float   ARM_STILL_GYRO_DPS = 20.0f;  // vs running mean (~6 sigma)
static const float   ARM_STILL_AMAG_LO  = 0.85f;  // |A| band, g
static const float   ARM_STILL_AMAG_HI  = 1.15f;
static const uint8_t ARM_STILL_MIN_N    = 30;     // samples before mean is usable

struct ArmCalib {
  bool     running  = false;
  bool     done     = false;
  int      count    = 0;
  long     ax_s = 0, ay_s = 0, az_s = 0;
  long     gx_s = 0, gy_s = 0, gz_s = 0;
  uint32_t lastMs   = 0;
  uint16_t restarts = 0;
};
ArmCalib g_armCalib;

static void armCalibRestart() {
  g_armCalib.count = 0;
  g_armCalib.ax_s = g_armCalib.ay_s = g_armCalib.az_s = 0;
  g_armCalib.gx_s = g_armCalib.gy_s = g_armCalib.gz_s = 0;
}

static void armCalibBegin() {
  armCalibRestart();
  g_armCalib.running  = true;
  g_armCalib.done     = false;
  g_armCalib.restarts = 0;
  g_armCalib.lastMs   = 0;
}

// Call at >= 100 Hz from loop(). Returns true once offsets are committed.
static bool armCalibTick(uint32_t now) {
  if (!g_armCalib.running || g_armCalib.done) return g_armCalib.done;
  if (!g_mpuEnabled) return false;
  if (g_buzzerActive) return false;   // I2C unreliable while buzzer draws current
  if (now - g_armCalib.lastMs < 10) return false;   // 100 Hz
  g_armCalib.lastMs = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // ── Stillness gate ──────────────────────────────────────────────────
  float amag = sqrtf((float)ax * ax + (float)ay * ay + (float)az * az) / ACCEL_SCALE;
  bool moving = (amag < ARM_STILL_AMAG_LO || amag > ARM_STILL_AMAG_HI);

  if (!moving && g_armCalib.count >= ARM_STILL_MIN_N) {
    float n   = (float)g_armCalib.count;
    float mgx = (g_armCalib.gx_s / n) / GYRO_SCALE;
    float mgy = (g_armCalib.gy_s / n) / GYRO_SCALE;
    float mgz = (g_armCalib.gz_s / n) / GYRO_SCALE;
    if (fabsf(gx / GYRO_SCALE - mgx) > ARM_STILL_GYRO_DPS ||
        fabsf(gy / GYRO_SCALE - mgy) > ARM_STILL_GYRO_DPS ||
        fabsf(gz / GYRO_SCALE - mgz) > ARM_STILL_GYRO_DPS) {
      moving = true;
    }
  }

  if (moving) {
    if (g_armCalib.count > 0) {
      g_armCalib.restarts++;
      armCalibRestart();
    }
    return false;
  }

  g_armCalib.ax_s += ax; g_armCalib.ay_s += ay; g_armCalib.az_s += az;
  g_armCalib.gx_s += gx; g_armCalib.gy_s += gy; g_armCalib.gz_s += gz;

  if (++g_armCalib.count < CALIB_SAMPLES) return false;

  ax_off = g_armCalib.ax_s / CALIB_SAMPLES;
  ay_off = g_armCalib.ay_s / CALIB_SAMPLES;
  az_off = g_armCalib.az_s / CALIB_SAMPLES;
  gx_off = g_armCalib.gx_s / CALIB_SAMPLES;
  gy_off = g_armCalib.gy_s / CALIB_SAMPLES;
  gz_off = g_armCalib.gz_s / CALIB_SAMPLES;

  // The mean raw accel at the calibration pose IS the gravity reference the
  // pitch formula needs -- same as calibrateMPU() captures.
  g_calibGrav[0] = (float)ax_off;
  g_calibGrav[1] = (float)ay_off;
  g_calibGrav[2] = (float)az_off;

  g_armCalib.running = false;
  g_armCalib.done    = true;
  Serial.printf("#STATUS: IMU calibrated for this session (%d motion restarts)\n",
                (int)g_armCalib.restarts);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────
// signalState() — the single point every state change passes through.
//
// Serial only for now. LED_BUILTIN is GPIO 2, which is PIN_SDA, so it cannot
// be used as an indicator without corrupting the I2C bus, and the buzzer is
// physically disconnected on this rig. GPIO 47/48 are clear of the camera, SD,
// IMU, pulse and button pins -- wire an indicator there and drive it HERE.
// ─────────────────────────────────────────────────────────────────────────
void signalState(SessionState st) {
  switch (st) {
    case SESSION_IDLE:      Serial.println(F("#STATE: IDLE"));      break;
    case SESSION_ARMING:    Serial.println(F("#STATE: ARMING"));    break;
    case SESSION_RECORDING: Serial.println(F("#STATE: RECORDING")); break;
  }
}

// Enter ARMING. Every per-session quantity is cleared here so the session gets
// its own calibration instead of inheriting boot-time or previous-session values.
void armingBegin() {
  g_armStartMs  = millis();
  g_armTimedOut = false;
  g_readyImu = g_readyHr = g_readyEye = false;

  armCalibBegin();

  // Fresh HR baseline. Carrying the previous session's resting rate over would
  // skew hr_diff_pct for the whole of this recording.
  g_baselineSum    = 0.0f;
  g_baselineCount  = 0;
  g_baselineFormed = false;

#if defined(STORAGE_MODE_SD) || defined(EAR_LIVE_DEBUG)
  // Drop the old ROI so the lock re-acquires against where the eye is now,
  // rather than reusing a crop from a previous head/camera position.
  g_earLockDone    = false;
  g_earRoi.locked  = false;
  g_earMotionTries = 0;
  g_earLocator.reset();
#endif

  g_sessionState  = SESSION_ARMING;
  g_sessionActive = false;
  signalState(SESSION_ARMING);
  Serial.println(F("#STATUS: Arming -- hold still, eye toward camera, finger on pulse sensor"));
}

// Enter RECORDING. Latches readiness, opens the session files, starts writing.
void recordingBegin() {
#if defined(STORAGE_MODE_SD) || defined(EAR_LIVE_DEBUG)
  g_readyEye = g_earLockDone;
#else
  g_readyEye = true;   // no EAR pipeline in this build
#endif
  g_readyImu = g_armCalib.done;
  g_readyHr  = g_baselineFormed;

#if defined(STORAGE_MODE_SD)
  if (!openSession()) {
    Serial.println(F("#ERROR: Could not open session files -- staying idle"));
    g_sessionState  = SESSION_IDLE;
    g_sessionActive = false;
    signalState(SESSION_IDLE);
    return;
  }
#endif

  g_sessionState  = SESSION_RECORDING;
  g_sessionActive = true;
  signalState(SESSION_RECORDING);
  Serial.printf("#STATUS: Recording -- imu=%d hr=%d eye=%d timed_out=%d after %lu ms\n",
                g_readyImu ? 1 : 0, g_readyHr ? 1 : 0, g_readyEye ? 1 : 0,
                g_armTimedOut ? 1 : 0,
                (unsigned long)(millis() - g_armStartMs));
}

// Return to IDLE from any state, closing the session files if they are open.
void sessionStop(const char *reason) {
  bool wasRecording = (g_sessionState == SESSION_RECORDING);

  g_sessionState  = SESSION_IDLE;
  g_sessionActive = false;
  g_armCalib.running = false;

#if defined(STORAGE_MODE_SD)
  if (wasRecording) {
    // Report what actually landed on the card. A session that ran but wrote
    // zero frames or zero CSV rows is a silent failure otherwise.
    Serial.printf("#STATUS: Wrote %lu frames, %lu video bytes\n",
                  (unsigned long)g_frameIndex, (unsigned long)g_byteOffset);
    closeSession();   // flush + close; without this the tail of the recording
                      // is lost when the card is pulled
  }
#else
  (void)wasRecording;
#endif

  signalState(SESSION_IDLE);
  Serial.printf("#STATUS: %s\n", reason);
}

// One press, dispatched by current state. Shared by the physical button on
// GPIO 21 and the "BUTTON" serial command, so a bench test over USB exercises
// exactly the same path as a press on the helmet -- not a parallel one.
void sessionButtonPress() {
  switch (g_sessionState) {
    case SESSION_IDLE:      armingBegin(); break;
    case SESSION_ARMING:    sessionStop("Arming aborted"); break;
    case SESSION_RECORDING: sessionStop("Session stopped"); break;
  }
}



// ─────────────────────────────────────────────────────────────────────────
// tryInitMPU() — probe the two addresses an MPU-6050 can be strapped to
// (0x68/0x69), validate WHO_AM_I, and configure ranges on success.
// Returns the detected address, or 0 if not found / WHO_AM_I mismatch.
// Does NOT calibrate — caller decides whether a (re)calibration is needed.
// ─────────────────────────────────────────────────────────────────────────

uint8_t tryInitMPU() {
  uint8_t foundAddr = 0;
  for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      foundAddr = addr;
      break;
    }
  }
  if (foundAddr == 0) return 0;

  mpu = MPU6050(foundAddr);
  // Force a full device reset (PWR_MGMT_1 bit 7) before waking it, instead
  // of going straight to initialize()'s "gentle" clock-source/sleep-bit
  // writes. A chip left in a confused internal state by an irregular
  // power-up sequence can ACK its address yet still misbehave on register
  // reads/writes until it sees a real reset -- see D:\proj\atttts\imu_test
  // (IMU_INTEGRATION.md), whose manual reset-then-wake sequence recovered
  // this exact chip when our library-only initialize() path could not.
  mpu.reset();
  delay(120);
  mpu.initialize();
  uint8_t whoami = mpu.getDeviceID();
  Serial.printf("#STATUS: MPU at 0x%02X WHO_AM_I = 0x%02X\n", foundAddr, whoami);
  // getDeviceID() returns the 6-bit device ID field (register 0x75 bits
  // 6:1), NOT the raw I2C address -- 0x68/0x69 here were never valid
  // values for it to return. Match the MPU6050 library's own
  // testConnection() whitelist instead: 0x34 is the common value, but
  // 0x0C and 0x3A are documented hardware-revision variants of the same
  // genuine chip (see MPU6050_Base::testConnection() in MPU6050.cpp).
  if (whoami != 0x34 && whoami != 0x0C && whoami != 0x3A) {
    return 0;
  }

  g_mpuEnabled = true;
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  return foundAddr;
}

// ─────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────

void setup() {
  // Claim the buzzer pin FIRST, before anything else -- Serial.begin()'s
  // up-to-3s wait, the fixed 1.5s delay below, camera init, and the LED
  // blink all run before this point used to be reached. A freshly reset
  // ESP32 GPIO floats until pinMode()+digitalWrite() explicitly claim it,
  // and a floating transistor base can pick up enough noise to partially
  // turn the buzzer on for that whole multi-second window. Driving it LOW
  // here shrinks that window to the fixed hardware boot time before
  // setup() runs at all, which we can't reduce further from application
  // code.
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

#if defined(STORAGE_MODE_USB)
  Serial.begin(921600);
#else
  Serial.begin(115200);
#endif
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000)
    delay(10);
  // Native USB-CDC re-enumerates on every reset, so a host-side monitor
  // that was attached before the reset is still reconnecting for a bit
  // after Serial reports ready -- without this, the earliest boot lines
  // (camera/PSRAM/SD status, the ones you most want on a fresh reset) are
  // silently dropped since nothing is listening yet.
  delay(1500);

  Serial.println(F("#STATUS: ---- IoT Fatigue Helmet Phase 2 ----"));
#if defined(STORAGE_MODE_SD)
  Serial.println(F("#STATUS: Mode = PRODUCTION (SD card)"));
#else
  Serial.println(F("#STATUS: Mode = DEBUG (USB streaming)"));
#endif

  // ── Camera Initialization ─────────────────────────────────────────────
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  // Resolution/FPS/quality controlled by CAMERA_FRAMESIZE, CAMERA_FPS,
  // CAMERA_JPEG_QUALITY -- defined at top of file (or via platformio.ini flags)

  config.frame_size = CAMERA_FRAMESIZE;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = CAMERA_JPEG_QUALITY;
  config.fb_count = 1;

  if (psramFound()) {
    config.fb_count = 2; // double-buffer for smoother capture
    config.grab_mode = CAMERA_GRAB_LATEST;
    Serial.println(F("#STATUS: PSRAM found — using 2 frame buffers"));
  } else {
    // No PSRAM: drop to QVGA to fit in internal DRAM (~320 KB available)
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
    Serial.println(F("#WARN: No PSRAM — falling back to QVGA (internal DRAM)"));
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("#ERROR: Camera init failed 0x%x\n", err);
  } else {
    Serial.println(F("#STATUS: Camera initialized successfully"));
  }

  blinkLED(3);

  analogReadResolution(12);
  Serial.println(F("#STATUS: Pulse sensor GPIO 1 (ADC1) — analog mode"));

  // Session Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println(F("#STATUS: Button GPIO 21 initialized (INPUT_PULLUP)"));

  // Buzzer: GPIO 14 (confirmed free — see fuzzy_walkthrough.md §7 GPIO audit).
  // Pin already claimed at the very top of setup() -- see that comment.
  // NOTE: do NOT use LED_BUILTIN (GPIO 2) after Wire.begin() — conflicts with I2C SDA.
  Serial.println(F("#STATUS: Buzzer GPIO 14 initialized"));

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);

  if (tryInitMPU() != 0) {
    Serial.println(F("#STATUS: MPU-6050 initialized OK"));
    calibrateMPU();
  } else {
    g_mpuEnabled = false;
    Serial.println(F("#WARN: MPU-6050 not detected/validated at 0x68/0x69 — running with IMU disabled"));
  }

  // ── SD card initialization (production mode) ─────────────────────────
#if defined(STORAGE_MODE_SD)
  g_sdMutex = xSemaphoreCreateMutex();
  g_sdReady = initSDCard();
  if (!g_sdReady) {
    // NO LED blink here. LED_BUILTIN is GPIO 2, which is PIN_SDA -- driving it
    // after Wire.begin() slams the I2C data line ten times and can wedge the
    // MPU-6050 mid-byte, exactly the failure the comment above warns about
    // and the one sensor_test now carries a bus-recovery routine for. The
    // error is reported on serial only until a real indicator exists on a
    // free GPIO (see signalState()).
    Serial.println(F("#ERROR: SD init failed -- CSV goes to serial only"));
  }
#endif

  Serial.println(
      F("#HEADER:timestamp_ms,hr_bpm,pulse_raw,"
        "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,head_movement,signal_quality,"
        "blink_rate,pitch_deg,gyro_var,nod_score,risk_pct,alert_level,imu_valid"));
  Serial.println(F("#STATUS: Logging started"));

#if defined(STORAGE_MODE_USB)
  Serial.println(F("#STATUS: USB mode — run debug_recorder.py on PC"));
  // ── Create Serial mutex (USB mode: needed to protect binary frame writes) ─
  g_serialMutex = xSemaphoreCreateMutex();
#elif defined(STORAGE_MODE_SD)
  g_serialMutex =
      xSemaphoreCreateMutex(); // still used for clean serial text output
  Serial.println(F("#STATUS: SD mode — data recording to SD card"));
#endif

  // ── Launch camera task on Core 0 ─────────────────────────────────────
  // FRAME_INJECT_MODE replaces live capture with frameInjectTask(), which
  // owns Serial exclusively for the PC->ESP frame protocol -- no live
  // camera task runs alongside it (see that task's own doc comment).
#if defined(FRAME_INJECT_MODE)
  xTaskCreatePinnedToCore(frameInjectTask, "FrameInject",
                          8192,       // stack bytes
                          nullptr, 2, // priority 2
                          nullptr, 0  // Core 0
  );
#else
  xTaskCreatePinnedToCore(cameraTask, "CameraTask",
                          8192,       // stack bytes
                          nullptr, 2, // priority 2
                          nullptr, 0  // Core 0
  );
#endif

  // ── Initial session state ────────────────────────────────────────────
#if defined(STORAGE_MODE_USB)
  // USB debug mode streams immediately: there is no SD session to open and
  // no arming gate, matching the previous behaviour of this build.
  g_sessionState  = SESSION_RECORDING;
  g_sessionActive = true;
#else
  g_sessionState  = SESSION_IDLE;
  g_sessionActive = false;
  Serial.println(F("#STATUS: Idle -- press the button on GPIO 21 to arm a session"));
#endif
  signalState(g_sessionState);
}

// ─────────────────────────────────────────────────────────────────────────
// Main loop — Core 1
//   500 Hz → readPulseSensor()
//   10 Hz → IMU sampling / processing
//     1 Hz → CSV output to SD (and/or Serial) + FIS inference
// ─────────────────────────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

  // ── Session Button Logic (Non-blocking) ──────────────────────────────
  int reading = digitalRead(BUTTON_PIN);
  if (reading != g_lastButtonState) {
    g_lastDebounceTime = now;
  }
  if ((now - g_lastDebounceTime) > 50) {
    static int buttonState = HIGH;
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) { // pressed
        sessionButtonPress();
      }
    }
  }
  g_lastButtonState = reading;

  // ── ARMING progress ──────────────────────────────────────────────────
  // Nothing is written to SD in this state. The three checks run concurrently
  // and independently; recording starts when all are ready, or when the
  // timeout expires (readiness is then recorded in metadata.txt rather than
  // costing the session).
  if (g_sessionState == SESSION_ARMING) {
    armCalibTick(now);

    bool imuOk = g_armCalib.done;
    bool hrOk  = g_baselineFormed;
#if defined(STORAGE_MODE_SD) || defined(EAR_LIVE_DEBUG)
    bool eyeOk = g_earLockDone;
#else
    bool eyeOk = true;   // no EAR pipeline compiled into this build
#endif

    static uint32_t lastArmReport = 0;
    if (now - lastArmReport >= 2000) {
      lastArmReport = now;
      Serial.printf("#ARMING: imu=%d hr=%d(%u/%u) eye=%d  %lu/%lu s\n",
                    imuOk ? 1 : 0,
                    hrOk ? 1 : 0,
                    (unsigned)g_baselineCount, (unsigned)N_BASELINE_SAMPLES,
                    eyeOk ? 1 : 0,
                    (unsigned long)((now - g_armStartMs) / 1000),
                    (unsigned long)(ARMING_TIMEOUT_MS / 1000));
    }

    if (imuOk && hrOk && eyeOk) {
      recordingBegin();
    } else if (now - g_armStartMs >= ARMING_TIMEOUT_MS) {
      g_armTimedOut = true;
      Serial.println(F("#WARNING: Arming timed out -- recording anyway; check armed_* in metadata.txt"));
      recordingBegin();
    }
  }



  // ── Pulse sensor: 500 Hz — must ALWAYS run, never skip ───────────────
  // Keep this first and outside any mutex so it is never starved.
  static unsigned long lastSampleTime = 0;
  if (now - lastSampleTime >= SAMPLE_RATE_MS) {
    lastSampleTime = now;
    readPulseSensor();
  }

  // ── Feed task watchdog so Core 1 is never considered hung ────────────
  esp_task_wdt_reset();

  // ── 10 Hz IMU ring buffer fill ────────────────────────────────────────────────────
  // Pitch, gyro variance, and nodding detector require 10 Hz IMU data.
  // Results cached in g_imu_* are consumed at the 1 Hz FIS tick + CSV output.
  // No mutex needed: both this block and the 1 Hz block run on Core 1 (loop).
  static unsigned long lastImuTime = 0;
  if (now - lastImuTime >= 100) {   // 100 ms = 10 Hz
    lastImuTime = now;

    // Auto-reconnect retry if IMU was not detected at boot or connection was lost
    static unsigned long lastImuRetry = 0;
    if (!g_mpuEnabled && (now - lastImuRetry >= 2000)) {
      lastImuRetry = now;
      Wire.begin(PIN_SDA, PIN_SCL);
      Wire.setClock(100000);
      uint8_t foundAddr = tryInitMPU();
      if (foundAddr != 0) {
        if (ax_off == 0 && ay_off == 0 && az_off == 0) {
          calibrateMPU();
        } else {
          Serial.printf("#STATUS: MPU-6050 I2C connection restored (address 0x%02X)\n", foundAddr);
        }
      }
    }

    // IMU reads are suspended while the buzzer draws current (EMI corrupts the
    // I2C bus). Rather than substituting calibration offsets -- which resolve to
    // an exact "0 deg pitch, 0 movement" upright-and-still reading -- we mark the
    // sample invalid and FREEZE: no ring-buffer push, no g_imu_* update.
    //
    // Why not substitute zeros: a 2 s CRITICAL beep injects 20 fake samples into
    // the 60-sample nod buffer (1/3 of it), collapsing pitch_Limp and suppressing
    // the ZCR that drives nod_score. R3 -- the head-drop rule that fired the alarm
    // -- then reads ~0, so the alarm erases its own trigger and self-cancels while
    // the rider is still drooping. It also writes that fiction into the CSV at
    // exactly the fatigue events the dataset exists to capture.
    //
    // Freezing is safe against the "death loop" the zeros were guarding against:
    // the alert state machine caps buzzer on-time (2 s CRITICAL / 1 s WARNING) and
    // forces a 3 s cooldown during which g_buzzerActive is false, so ~30 real
    // samples land before any re-fire decision. A stuck alarm is structurally
    // impossible regardless of what the frozen values held.
    int16_t ax_r10 = 0, ay_r10 = 0, az_r10 = 0;
    int16_t gx_r10 = 0, gy_r10 = 0, gz_r10 = 0;
    bool imuSampleValid = false;
    if (g_mpuEnabled && !g_buzzerActive) {
      static int i2cErrorCount = 0;
      Wire.beginTransmission(MPU_ADDR);
      if (Wire.endTransmission() != 0) {
        i2cErrorCount++;
        if (i2cErrorCount >= 10) { // Require 10 consecutive errors before marking disabled
          g_mpuEnabled = false;
          Wire.begin(PIN_SDA, PIN_SCL); // Instant I2C bus hardware reset
        }
      } else {
        i2cErrorCount = 0;
        mpu.getMotion6(&ax_r10, &ay_r10, &az_r10, &gx_r10, &gy_r10, &gz_r10);
        imuSampleValid = true;
      }
    }

    // Freeze on invalid sample: leave ring buffers and g_imu_* holding the last
    // real reading. Buffers then contain 100% real data spanning a slightly
    // longer wall-clock window, instead of real data diluted with fabrications.
    if (imuSampleValid) {
      float ax_g10  = (ax_r10 - ax_off) / ACCEL_SCALE;
      float ay_g10  = (ay_r10 - ay_off) / ACCEL_SCALE;
      float az_g10  = (az_r10 - az_off) / ACCEL_SCALE;
      float gx_dp10 = (gx_r10 - gx_off) / GYRO_SCALE;
      float gy_dp10 = (gy_r10 - gy_off) / GYRO_SCALE;
      float gz_dp10 = (gz_r10 - gz_off) / GYRO_SCALE;
      float head_mov10 = sqrtf(gx_dp10*gx_dp10 + gy_dp10*gy_dp10 + gz_dp10*gz_dp10);

      // Pitch: dot-product angle between current accel and calibration gravity.
      // Uses raw (un-offsetted) readings so the calibration reference frame is intact.
      float gx_raw = (float)ax_r10;
      float gy_raw = (float)ay_r10;
      float gz_raw = (float)az_r10;
      float dot    = g_calibGrav[0]*gx_raw + g_calibGrav[1]*gy_raw + g_calibGrav[2]*gz_raw;
      float mag_c  = sqrtf(g_calibGrav[0]*g_calibGrav[0] +
                           g_calibGrav[1]*g_calibGrav[1] +
                           g_calibGrav[2]*g_calibGrav[2]);
      float mag_n  = sqrtf(gx_raw*gx_raw + gy_raw*gy_raw + gz_raw*gz_raw);
      float cos_a  = (mag_c > 1e-3f && mag_n > 1e-3f) ? dot / (mag_c * mag_n) : 1.0f;
      if (cos_a >  1.0f) cos_a =  1.0f;
      if (cos_a < -1.0f) cos_a = -1.0f;
      float pitch10 = acosf(cos_a) * (180.0f / (float)M_PI);

      // Push into ring buffers
      g_nodDet.push(pitch10);
      g_gyroVar.push(head_mov10);

      // Cache latest snapshot for 1 Hz FIS tick and CSV
      g_imu_ax_g      = ax_g10;
      g_imu_ay_g      = ay_g10;
      g_imu_az_g      = az_g10;
      g_imu_gx_dp     = gx_dp10;
      g_imu_gy_dp     = gy_dp10;
      g_imu_gz_dp     = gz_dp10;
      g_imu_head_mov  = head_mov10;
      g_imu_pitch_deg = pitch10;
      g_imu_gyro_var  = g_gyroVar.variance();
      g_imu_nod_score = g_nodDet.score();
    }

    // Track freshness for the 1 Hz CSV row so frozen samples are visibly stale
    // rather than silently passed off as live readings.
    g_imuSampleValid = imuSampleValid;
    if (imuSampleValid) g_lastImuValidTime = now;
  }

  // ── CSV output: 1 Hz ───────────────────────────────────────────────────────────
  static unsigned long lastOutputTime = 0;
  if (now - lastOutputTime < LOOP_MS)
    return;
  lastOutputTime = now;

  // ── Serial BLINK parser (drains UART buffer each 1 Hz tick) ──────────────────
  // Format: "BLINK:<float>\n"  Rate: 1 Hz from Python pipeline.
  // Valid range: 0–60 bl/min. Invalid / out-of-range lines silently discarded.
  // Disabled under FRAME_INJECT_MODE: frameInjectTask() owns Serial reads
  // there (the binary frame-injection protocol), and two tasks racing to
  // read the same UART would each steal the other's bytes.
#if !defined(FRAME_INJECT_MODE)
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      g_serialLineBuf[g_serialLineBufLen] = '\0';
      // Bench affordance: "BUTTON" over serial is treated as one press of the
      // GPIO 21 button, routed through the same sessionButtonPress() dispatch.
      // Lets the arm/record/stop flow be exercised over USB without reaching
      // into the helmet, and cannot collide with the BLINK: format below.
      if (strcmp(g_serialLineBuf, "BUTTON") == 0) {
        g_serialLineBufLen = 0;
        sessionButtonPress();
        continue;
      }
      float blink_val = 0.0f;
      if (sscanf(g_serialLineBuf, "BLINK:%f", &blink_val) == 1
          && blink_val >= 0.0f && blink_val <= 60.0f) {
        g_blinkRate       = blink_val;
        g_lastValidBlink  = blink_val;
        g_lastBlinkRxTime = now;
        g_blinkEverRx     = true;
      }
      g_serialLineBufLen = 0;
    } else if (g_serialLineBufLen < (uint8_t)(sizeof(g_serialLineBuf) - 1)) {
      g_serialLineBuf[g_serialLineBufLen++] = c;
    } else {
      g_serialLineBufLen = 0;   // line too long — discard
    }
  }
#endif  // !FRAME_INJECT_MODE
  // Fallback: sticky last-valid after timeout; on-device EAR rate in SD
  // mode or EAR_LIVE_DEBUG (see processEarFrame() above), or the 13.0 stub
  // in plain USB mode (USB debug mode without EAR_LIVE_DEBUG still relies
  // on the offline eye_ear.py pipeline, see spec non-goals).
  if (g_blinkEverRx && (now - g_lastBlinkRxTime > BLINK_TIMEOUT_MS)) {
    g_blinkRate = g_lastValidBlink;
  } else if (!g_blinkEverRx) {
#if defined(STORAGE_MODE_SD) || defined(EAR_LIVE_DEBUG)
    g_blinkRate = g_onDeviceBlinkRate;
#else
    g_blinkRate = 13.0f;
#endif
  }

  // ── Warn on prolonged no-contact (non-blocking: skip if mutex busy) ────────
  if (lastSignalQuality == 0) {
    if (++noContactStreak >= NO_CONTACT_WARN_N) {
      if (xSemaphoreTake(g_serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        Serial.println(F("#WARNING: Pulse signal low — check sensor contact"));
        xSemaphoreGive(g_serialMutex);
      }
      noContactStreak = 0;
    }
  } else {
    noContactStreak = 0;
  }

  // ── Baseline HR accumulation ───────────────────────────────────────────────────
  // Count-based (not time-based): only valid readings count.
  // Frozen after formed — not rolling — so progressive drowsiness is not
  // normalised out. Resets only on power cycle (new session).
  int hr_out = (lastSignalQuality == 1 && currentBPM > 0) ? (int)currentBPM : 0;

  if (!g_baselineFormed && lastSignalQuality == 1 && currentBPM > 0) {
    g_baselineSum += currentBPM;
    g_baselineCount++;
    if (g_baselineCount >= N_BASELINE_SAMPLES) {
      g_baselineBPM    = g_baselineSum / (float)g_baselineCount;
      g_baselineFormed = true;
      if (xSemaphoreTake(g_serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        Serial.printf("#STATUS: Baseline HR formed: %.1f BPM (%u samples)\n",
                      g_baselineBPM, (unsigned)g_baselineCount);
        xSemaphoreGive(g_serialMutex);
      }
    }
  }

  // ── FIS update (1 Hz) ──────────────────────────────────────────────────────────
  // hr_diff_pct = 0.0 while baseline forming → hr_Stable peaks → FIS relies
  // only on blink + IMU signals during warmup. No false alarms from HR alone.
  float hr_diff_pct = 0.0f;
  if (g_baselineFormed && hr_out > 0) {
    hr_diff_pct = (currentBPM - g_baselineBPM) / g_baselineBPM * 100.0f;
  }

  g_fis.update(hr_diff_pct,
               g_blinkRate,
               g_imu_gyro_var,
               g_imu_pitch_deg,
               g_imu_nod_score,
               g_riskScore,
               g_alertLevel);

  // ── Buzzer alert output (GPIO 14) ─────────────────────────────────────────────
  if (g_sessionBeepState == 0) { // Don't interfere with start/stop beeps
    if (g_sessionActive) {
#if defined(TEST_MODE_HARDWARE)
      // Test mode: Beep buzzer for 200ms every 5 seconds
      static uint32_t lastTestBeep = 0;
      if (now - lastTestBeep >= 5000) {
        setBuzzerState(true);
        lastTestBeep = now;
      } else if (now - lastTestBeep >= 200) {
        setBuzzerState(false);
      }
#else
      // CRITICAL : 2s ON, 3s OFF (cooldown)
      // WARNING  : 1s ON, 3s OFF (cooldown).
      // SAFE     : buzzer off.
#if defined(STORAGE_MODE_USB)
      setBuzzerState(false);
#else
      static uint32_t alertStartTime = 0;
      static bool alertActive = false;
      static uint32_t cooldownStartTime = 0;
      static bool cooldownActive = false;
      static uint32_t currentAlertDuration = 0;
      const uint32_t COOLDOWN_DURATION = 3000;

      if (cooldownActive) {
        if (now - cooldownStartTime > COOLDOWN_DURATION) {
          cooldownActive = false;
        }
      } else if (alertActive) {
        if (now - alertStartTime > currentAlertDuration) {
          alertActive = false;
          cooldownActive = true;
          cooldownStartTime = now;
          setBuzzerState(false);

          // Buzzer just went silent -- try to reconnect right now instead of
          // waiting up to 2 s for the periodic retry in the 10 Hz IMU block.
          // Shrinks the "blind" window right after an alert, when detecting
          // whether the driver actually woke up matters most.
          Wire.begin(PIN_SDA, PIN_SCL);
          Wire.setClock(100000);
          tryInitMPU();
        } else {
          setBuzzerState(true);
        }
      } else {
        // Idle state - wait for new alert
        if (g_alertLevel == ALERT_CRITICAL || g_alertLevel == ALERT_WARNING) {
          // Force a fresh MPU reconnect/re-init right as the buzzer is about
          // to start drawing current, so it enters the noisy window in a
          // known-good state instead of whatever it drifted to since the
          // last successful read. Cheap (a few I2C transactions, no delay())
          // -- does NOT recalibrate, so it's safe to call every alert onset.
          Wire.begin(PIN_SDA, PIN_SCL);
          Wire.setClock(100000);
          tryInitMPU();

          alertActive = true;
          alertStartTime = now;
          currentAlertDuration = (g_alertLevel == ALERT_CRITICAL) ? 2000 : 1000;
          setBuzzerState(true);
        } else {
          setBuzzerState(false);
        }
      }
#endif
#endif
    } else {
      // Keep buzzer quiet while paused (except for the start/stop beeps handled above)
      setBuzzerState(false);
    }
  }

  // ── Serial output (human-readable, both modes) ──────────────────────────────
  // Uses cached 10 Hz IMU snapshot (g_imu_*) — no extra I2C read at 1 Hz.
  if (xSemaphoreTake(g_serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    const char *alertStr = (g_alertLevel == ALERT_CRITICAL) ? "CRITICAL" :
                           (g_alertLevel == ALERT_WARNING)  ? "WARNING"  : "safe";
    Serial.println(F("#------------------------------------"));
    Serial.print(F("# t:"));
    Serial.print(now);
    Serial.print(F("ms  |  HR: "));
    if (hr_out > 0) {
      Serial.print(hr_out);
      Serial.print(F(" BPM"));
      if (g_baselineFormed) {
        Serial.printf("  (diff %.1f%%)", hr_diff_pct);
      } else {
        Serial.print(F("  (baseline forming...)"));
      }
    } else {
      Serial.print(F("-- BPM"));
    }
    Serial.print(F("  PULSE:"));
    Serial.print(lastPulseRaw);
    Serial.println(lastSignalQuality ? F("  [OK]") : F("  [NO CONTACT]"));
    if (!g_mpuEnabled) {
      Serial.println(F("# IMU: [DISCONNECTED] — check MPU-6050 VCC/GND/SDA/SCL wiring"));
    }
    Serial.print(F("# AX:"));
    Serial.print(g_imu_ax_g, 3);
    Serial.print(F("  AY:"));
    Serial.print(g_imu_ay_g, 3);
    Serial.print(F("  AZ:"));
    Serial.print(g_imu_az_g, 3);
    Serial.print(F("  MOV:"));
    Serial.print(g_imu_head_mov, 3);
    Serial.println(F("g"));
    Serial.print(F("# GX:"));
    Serial.print(g_imu_gx_dp, 1);
    Serial.print(F("  GY:"));
    Serial.print(g_imu_gy_dp, 1);
    Serial.print(F("  GZ:"));
    Serial.println(g_imu_gz_dp, 1);
    // "[FROZEN Nms]" marks IMU values held from the last good read (buzzer
    // active or bus down) rather than sampled this tick -- so a suspicious
    // pitch/nod reading during an alert can be told apart from a live one.
    Serial.printf("# BLINK:%.1f  PITCH:%.1fdeg  GVAR:%.0f  NOD:%.2f%s",
                  g_blinkRate, g_imu_pitch_deg, g_imu_gyro_var, g_imu_nod_score,
                  g_imuSampleValid ? "\n" : "");
    if (!g_imuSampleValid) {
      Serial.printf("  [FROZEN %lums]\n",
                    (unsigned long)(now - g_lastImuValidTime));
    }
    Serial.printf("# RISK:%.1f%%  ALERT:%s\n", g_riskScore, alertStr);

#if defined(STORAGE_MODE_USB)
    // USB CSV line for debug_recorder.py
    if (g_sessionActive) {
      // Columns: timestamp_ms, hr_bpm, pulse_raw,
      //          ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, head_movement, signal_quality,
      //          blink_rate, pitch_deg, gyro_var, nod_score, risk_pct, alert_level,
      //          imu_valid
      // imu_valid=0 marks a row whose IMU fields are frozen last-known-good
      // values (reads suspended during buzzer, or bus down) rather than a live
      // sample -- exclude those rows from IMU statistics during analysis.
      Serial.print(now);                Serial.print(',');
      Serial.print(hr_out);             Serial.print(',');
      Serial.print(lastPulseRaw);       Serial.print(',');
      Serial.print(g_imu_ax_g, 4);     Serial.print(',');
      Serial.print(g_imu_ay_g, 4);     Serial.print(',');
      Serial.print(g_imu_az_g, 4);     Serial.print(',');
      Serial.print(g_imu_gx_dp, 4);    Serial.print(',');
      Serial.print(g_imu_gy_dp, 4);    Serial.print(',');
      Serial.print(g_imu_gz_dp, 4);    Serial.print(',');
      Serial.print(g_imu_head_mov, 4); Serial.print(',');
      Serial.print(lastSignalQuality); Serial.print(',');
      Serial.print(g_blinkRate, 2);    Serial.print(',');
      Serial.print(g_imu_pitch_deg, 2); Serial.print(',');
      Serial.print(g_imu_gyro_var, 1); Serial.print(',');
      Serial.print(g_imu_nod_score, 3); Serial.print(',');
      Serial.print(g_riskScore, 2);    Serial.print(',');
      Serial.print(g_alertLevel);      Serial.print(',');
      Serial.println(g_imuSampleValid ? 1 : 0);
    }
#endif

    xSemaphoreGive(g_serialMutex);
  }

  // ── SD mode: write CSV row to SD card ───────────────────────────────────────
  // Column order matches USB CSV above for dataset compatibility.
  // Non-blocking: 20 ms timeout. Skip tick if camera task holds SD mutex.
#if defined(STORAGE_MODE_SD)
  if (g_sdReady && g_csvFile && g_sessionActive) {
    if (xSemaphoreTake(g_sdMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      g_csvFile.printf(
        "%lu,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.2f,%.2f,%.1f,%.3f,%.2f,%d,%d\n",
        now, hr_out, lastPulseRaw,
        g_imu_ax_g, g_imu_ay_g, g_imu_az_g,
        g_imu_gx_dp, g_imu_gy_dp, g_imu_gz_dp,
        g_imu_head_mov, lastSignalQuality,
        g_blinkRate, g_imu_pitch_deg, g_imu_gyro_var,
        g_imu_nod_score, g_riskScore, (int)g_alertLevel,
        g_imuSampleValid ? 1 : 0);
      g_csvFile.flush();
      xSemaphoreGive(g_sdMutex);
    }
  }
#endif
}
