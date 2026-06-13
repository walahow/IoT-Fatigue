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
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

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
#define MIN_AMPLITUDE 20        // min peak-to-valley swing
#define BEAT_TIMEOUT_MS 5000    // ms with no beat → reset BPM

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

// ── USB mode: binary frame constants ─────────────────────────────────────
#if defined(STORAGE_MODE_USB)
static const uint8_t FRAME_SOF[4] = {0xAA, 0xBB, 0xCC, 0xDD};
static const uint8_t FRAME_EOF[4] = {0xDD, 0xCC, 0xBB, 0xAA};
#endif

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
  if (!g_sdReady || !g_mjpegFile || !g_idxFile)
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
    Serial.write(data, len);
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
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
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
}

// ─────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────

void setup() {
#if defined(STORAGE_MODE_USB)
  Serial.begin(921600);
#else
  Serial.begin(115200);
#endif
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000)
    delay(10);

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

  Wire.begin(PIN_SDA, PIN_SCL);
  mpu.initialize();
  uint8_t whoami = mpu.getDeviceID();
  Serial.print(F("#STATUS: MPU WHO_AM_I = 0x"));
  Serial.println(whoami, HEX);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  Serial.println(F("#STATUS: MPU-6050 initialized OK"));

  calibrateMPU();

  // ── SD card initialization (production mode) ─────────────────────────
#if defined(STORAGE_MODE_SD)
  g_sdMutex = xSemaphoreCreateMutex();
  g_sdReady = initSDCard();
  if (!g_sdReady) {
    // Blink rapidly to signal SD error; still continue (CSV to Serial only)
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(80);
      digitalWrite(LED_BUILTIN, LOW);
      delay(80);
    }
  }
#endif

  Serial.println(
      F("#HEADER:timestamp_ms,hr_bpm,pulse_raw,"
        "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,head_movement,signal_quality"));
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
  xTaskCreatePinnedToCore(cameraTask, "CameraTask",
                          8192,       // stack bytes
                          nullptr, 2, // priority 2
                          nullptr, 0  // Core 0
  );
}

// ─────────────────────────────────────────────────────────────────────────
// Main loop — Core 1
//   500 Hz → readPulseSensor()
//     1 Hz → CSV output to SD (and/or Serial)
// ─────────────────────────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

  // ── Pulse sensor: 500 Hz ─────────────────────────────────────────────
  static unsigned long lastSampleTime = 0;
  if (now - lastSampleTime >= SAMPLE_RATE_MS) {
    lastSampleTime = now;
    readPulseSensor();
  }

  // ── CSV output: 1 Hz ─────────────────────────────────────────────────
  static unsigned long lastOutputTime = 0;
  if (now - lastOutputTime < LOOP_MS)
    return;
  lastOutputTime = now;

  // Warn on prolonged no-contact
  if (lastSignalQuality == 0) {
    if (++noContactStreak >= NO_CONTACT_WARN_N) {
      if (xSemaphoreTake(g_serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println(F("#WARNING: Pulse signal low — check sensor contact"));
        xSemaphoreGive(g_serialMutex);
      }
      noContactStreak = 0;
    }
  } else {
    noContactStreak = 0;
  }

  // ── MPU-6050 ──────────────────────────────────────────────────────────
  int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
  mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

  float ax_g = (ax_r - ax_off) / ACCEL_SCALE;
  float ay_g = (ay_r - ay_off) / ACCEL_SCALE;
  float az_g = (az_r - az_off) / ACCEL_SCALE;
  float gx_dp = (gx_r - gx_off) / GYRO_SCALE;
  float gy_dp = (gy_r - gy_off) / GYRO_SCALE;
  float gz_dp = (gz_r - gz_off) / GYRO_SCALE;
  float head_movement = sqrt(gx_dp * gx_dp + gy_dp * gy_dp + gz_dp * gz_dp);

  int hr_out = (lastSignalQuality == 1 && currentBPM > 0) ? (int)currentBPM : 0;

  // ── Serial output (human-readable, both modes) ────────────────────────
  if (xSemaphoreTake(g_serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println(F("#------------------------------------"));
    Serial.print(F("# t:"));
    Serial.print(now);
    Serial.print(F("ms  |  HR: "));
    if (hr_out > 0) {
      Serial.print(hr_out);
      Serial.print(F(" BPM"));
    } else {
      Serial.print(F("-- BPM"));
    }
    Serial.print(F("  PULSE:"));
    Serial.print(lastPulseRaw);
    Serial.println(lastSignalQuality ? F("  [OK]") : F("  [NO CONTACT]"));
    Serial.print(F("# AX:"));
    Serial.print(ax_g, 3);
    Serial.print(F("  AY:"));
    Serial.print(ay_g, 3);
    Serial.print(F("  AZ:"));
    Serial.print(az_g, 3);
    Serial.print(F("  MOV:"));
    Serial.print(head_movement, 3);
    Serial.println(F("g"));
    Serial.print(F("# GX:"));
    Serial.print(gx_dp, 1);
    Serial.print(F("  GY:"));
    Serial.print(gy_dp, 1);
    Serial.print(F("  GZ:"));
    Serial.println(gz_dp, 1);

#if defined(STORAGE_MODE_USB)
    // USB mode: CSV line for debug_recorder.py
    Serial.print(now);
    Serial.print(',');
    Serial.print(hr_out);
    Serial.print(',');
    Serial.print(lastPulseRaw);
    Serial.print(',');
    Serial.print(ax_g, 4);
    Serial.print(',');
    Serial.print(ay_g, 4);
    Serial.print(',');
    Serial.print(az_g, 4);
    Serial.print(',');
    Serial.print(gx_dp, 4);
    Serial.print(',');
    Serial.print(gy_dp, 4);
    Serial.print(',');
    Serial.print(gz_dp, 4);
    Serial.print(',');
    Serial.print(head_movement, 4);
    Serial.print(',');
    Serial.println(lastSignalQuality);
#endif

    xSemaphoreGive(g_serialMutex);
  }

  // ── SD mode: write CSV row to SD card ────────────────────────────────
#if defined(STORAGE_MODE_SD)
  if (g_sdReady && g_csvFile) {
    if (xSemaphoreTake(g_sdMutex, portMAX_DELAY) == pdTRUE) {
      g_csvFile.printf("%lu,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n", now,
                       hr_out, lastPulseRaw, ax_g, ay_g, az_g, gx_dp, gy_dp,
                       gz_dp, head_movement, lastSignalQuality);
      g_csvFile.flush(); // flush every row so data survives power loss
      xSemaphoreGive(g_sdMutex);
    }
  }
#endif
}
