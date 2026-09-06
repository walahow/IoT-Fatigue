/**
 * Sensor Test Utility — Pulse, IMU, Camera stability check
 *
 * This is a standalone diagnostic tool to validate sensor stability
 * and identify connection/calibration issues before running full firmware.
 *
 * Compile with: pio run -e esp32s3cam -t upload --upload-port COM_PORT
 * Then open Serial Monitor @ 921600 baud.
 *
 * Tests:
 *   1. Pulse Sensor (GPIO 1, ADC1): signal quality, baseline, BPM stability
 *   2. MPU-6050 IMU (I2C 0x68): WHO_AM_I, calibration, data range checks
 *   3. Camera (OV2640): frame capture rate, JPEG size consistency
 */

#include "MPU6050.h"
#include "esp_camera.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ── Sensor Pins & Timing ──────────────────────────────────────────────────
#define PULSE_PIN 1
#define SAMPLE_RATE_MS 2
#define SIGNAL_LOW_THRESH 200
#define SIGNAL_HIGH_THRESH 3900
#define MIN_AMPLITUDE 20
#define MIN_BEAT_INTERVAL 500
#define MAX_BEAT_INTERVAL 2000

const uint8_t PIN_SDA = 2;
const uint8_t PIN_SCL = 3;
const uint8_t MPU_ADDR = 0x68;
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE = 131.0f;

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

// ── Test State ────────────────────────────────────────────────────────────
struct SensorStats {
  uint32_t samples = 0;
  uint32_t errors = 0;
  float minVal = 1e9f, maxVal = -1e9f;
  float sumVal = 0.0f;
  uint32_t lastMs = 0;
};

struct PulseStats {
  SensorStats raw, bpm;
  uint32_t validBeats = 0;
  float avgBPM = 0.0f;
};

struct IMUStats {
  SensorStats ax, ay, az;
  SensorStats gx, gy, gz;
  uint32_t validReads = 0;
};

struct CameraStats {
  uint32_t framesCaptured = 0;
  uint32_t framesFailed = 0;
  uint32_t minFrameSize = 0xFFFFFFFF;
  uint32_t maxFrameSize = 0;
  uint32_t avgFrameSize = 0;
  uint32_t lastFrameMs = 0;
};

PulseStats pulseStats;
IMUStats imuStats;
CameraStats cameraStats;
MPU6050 mpu(MPU_ADDR);
bool g_mpuEnabled = false;

// ── Pulse Sensor Globals ──────────────────────────────────────────────────
float iirLP = 2048.0f;
float peakValue = 2048.0f;
float valleyValue = 2048.0f;
int dynamicThreshold = 2048;
bool risingSignal = false;
long lastBeatTime = 0;
float currentBPM = 0.0f;
const float ENVELOPE_DECAY = 0.990f;
const float LP_ALPHA = 0.15f;

// ─────────────────────────────────────────────────────────────────────────
// RAW WAVEFORM CAPTURE — dumps unfiltered ADC samples so the actual signal
// shape can be inspected, since the 10s aggregate stats below hide whether
// a periodic (heartbeat) component exists inside the min/max spread.
// ─────────────────────────────────────────────────────────────────────────

void dumpRawWaveform(uint32_t durationMs, uint32_t sampleIntervalMs) {
  Serial.println(F("\n▶ RAW WAVEFORM CAPTURE"));
  Serial.println(F("──────────────────────────────────────────────────────"));
  Serial.println(F("raw_adc"));

  uint32_t start = millis();
  uint32_t lastSample = 0;
  while (millis() - start < durationMs) {
    uint32_t now = millis();
    if (now - lastSample >= sampleIntervalMs) {
      lastSample = now;
      Serial.println(analogRead(PULSE_PIN));
    }
  }
  Serial.println(F("✓ CAPTURE COMPLETE\n"));
}

// ─────────────────────────────────────────────────────────────────────────
// PULSE SENSOR TEST
// ─────────────────────────────────────────────────────────────────────────

void testPulseSensor() {
  Serial.println(F("\n▶ PULSE SENSOR TEST (15 seconds)"));
  Serial.println(F("──────────────────────────────────────────────────────"));

  uint32_t testStart = millis();
  uint32_t lastReport = testStart;

  while (millis() - testStart < 15000) {
    unsigned long now = millis();
    static unsigned long lastSampleTime = 0;

    if (now - lastSampleTime >= SAMPLE_RATE_MS) {
      lastSampleTime = now;

      int raw = analogRead(PULSE_PIN);
      pulseStats.raw.samples++;
      pulseStats.raw.sumVal += raw;
      pulseStats.raw.minVal = fminf(pulseStats.raw.minVal, (float)raw);
      pulseStats.raw.maxVal = fmaxf(pulseStats.raw.maxVal, (float)raw);

      // Filter
      iirLP += LP_ALPHA * ((float)raw - iirLP);
      int filtered = constrain((int)iirLP, 0, 4095);

      // Envelope tracking
      if ((float)filtered > peakValue)
        peakValue = (float)filtered;
      else
        peakValue = peakValue * ENVELOPE_DECAY + 2048.0f * (1.0f - ENVELOPE_DECAY);
      if ((float)filtered < valleyValue)
        valleyValue = (float)filtered;
      else
        valleyValue = valleyValue * ENVELOPE_DECAY + 2048.0f * (1.0f - ENVELOPE_DECAY);

      dynamicThreshold = (int)((peakValue + valleyValue) / 2.0f);

      // Beat detection
      bool wasRising = risingSignal;
      risingSignal = (filtered > dynamicThreshold);

      if (!wasRising && risingSignal) {
        long interval = now - lastBeatTime;
        if (lastBeatTime > 0 && interval >= MIN_BEAT_INTERVAL && interval <= MAX_BEAT_INTERVAL) {
          currentBPM = 60000.0f / (float)interval;
          pulseStats.bpm.samples++;
          pulseStats.bpm.sumVal += currentBPM;
          pulseStats.bpm.minVal = fminf(pulseStats.bpm.minVal, currentBPM);
          pulseStats.bpm.maxVal = fmaxf(pulseStats.bpm.maxVal, currentBPM);
          pulseStats.validBeats++;
          lastBeatTime = now;
        } else if (lastBeatTime == 0) {
          lastBeatTime = now;
        }
      }

      if (lastBeatTime > 0 && (now - lastBeatTime > 5000)) {
        currentBPM = 0;
        lastBeatTime = 0;
      }

      // Signal quality
      int amplitude = (int)(peakValue - valleyValue);
      bool signalOK = (raw >= SIGNAL_LOW_THRESH && raw <= SIGNAL_HIGH_THRESH && amplitude >= MIN_AMPLITUDE);
      if (!signalOK) pulseStats.raw.errors++;
    }

    // Report every 10 seconds
    if (millis() - lastReport >= 10000) {
      lastReport = millis();
      Serial.printf("• Time: %lus | Raw ADC: [%d..%d] avg=%.0f | Beats: %lu | BPM: [%.1f..%.1f]\n",
        (unsigned long)((lastReport - testStart) / 1000),
        (int)pulseStats.raw.minVal, (int)pulseStats.raw.maxVal,
        pulseStats.raw.sumVal / (float)pulseStats.raw.samples,
        pulseStats.validBeats,
        pulseStats.bpm.minVal == 1e9f ? 0.0f : pulseStats.bpm.minVal,
        pulseStats.bpm.maxVal == -1e9f ? 0.0f : pulseStats.bpm.maxVal);
    }

    delay(1);
  }

  // Summary
  Serial.println(F("\n✓ PULSE SENSOR SUMMARY"));
  Serial.printf("  Samples: %lu | Errors (no contact): %lu | Error rate: %.2f%%\n",
    pulseStats.raw.samples, pulseStats.raw.errors,
    100.0f * (float)pulseStats.raw.errors / (float)pulseStats.raw.samples);
  Serial.printf("  ADC Range: [%d, %d] | Avg: %.0f\n",
    (int)pulseStats.raw.minVal, (int)pulseStats.raw.maxVal,
    pulseStats.raw.sumVal / (float)pulseStats.raw.samples);
  Serial.printf("  Valid Beats: %lu | BPM Range: [%.1f, %.1f]\n",
    pulseStats.validBeats,
    pulseStats.bpm.minVal == 1e9f ? 0.0f : pulseStats.bpm.minVal,
    pulseStats.bpm.maxVal == -1e9f ? 0.0f : pulseStats.bpm.maxVal);

  if (pulseStats.raw.errors > pulseStats.raw.samples / 10) {
    Serial.println(F("  ⚠️  WARNING: High error rate — check sensor contact/placement"));
  } else if (pulseStats.validBeats < 10) {
    Serial.println(F("  ⚠️  WARNING: Few valid beats detected — adjust finger pressure or contact"));
  } else {
    Serial.println(F("  ✓ STABLE"));
  }
}

// ─────────────────────────────────────────────────────────────────────────
// IMU TEST
// ─────────────────────────────────────────────────────────────────────────

uint8_t initMPU() {
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
  mpu.initialize();
  uint8_t whoami = mpu.getDeviceID();
  Serial.printf("  MPU at 0x%02X | WHO_AM_I: 0x%02X\n", foundAddr, whoami);

  if (whoami != 0x68 && whoami != 0x69 && whoami != 0x38 && whoami != 0x70 && whoami != 0x72) {
    return 0;
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  return foundAddr;
}

void testIMU() {
  Serial.println(F("\n▶ IMU (MPU-6050) TEST (30 seconds)"));
  Serial.println(F("──────────────────────────────────────────────────────"));

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  if (initMPU() == 0) {
    Serial.println(F("  ✗ FAILED: MPU-6050 not detected at 0x68/0x69"));
    return;
  }
  g_mpuEnabled = true;

  // Calibrate
  Serial.println(F("  Calibrating (keep still)..."));
  int16_t ax_off = 0, ay_off = 0, az_off = 0;
  int16_t gx_off = 0, gy_off = 0, gz_off = 0;
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_off += ax; ay_off += ay; az_off += az;
    gx_off += gx; gy_off += gy; gz_off += gz;
    delay(5);
  }
  ax_off /= 100; ay_off /= 100; az_off /= 100;
  gx_off /= 100; gy_off /= 100; gz_off /= 100;

  Serial.printf("  Offsets: ax=%d ay=%d az=%d | gx=%d gy=%d gz=%d\n",
    ax_off, ay_off, az_off, gx_off, gy_off, gz_off);

  uint32_t testStart = millis();
  uint32_t lastReport = testStart;

  while (millis() - testStart < 15000) {
    int16_t ax, ay, az, gx, gy, gz;
    Wire.beginTransmission(MPU_ADDR);
    bool i2cOk = (Wire.endTransmission() == 0);
    if (i2cOk) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      imuStats.validReads++;

      float ax_g = (ax - ax_off) / ACCEL_SCALE;
      float ay_g = (ay - ay_off) / ACCEL_SCALE;
      float az_g = (az - az_off) / ACCEL_SCALE;

      imuStats.ax.samples++;
      imuStats.ax.sumVal += ax_g;
      imuStats.ax.minVal = fminf(imuStats.ax.minVal, ax_g);
      imuStats.ax.maxVal = fmaxf(imuStats.ax.maxVal, ax_g);

      imuStats.ay.samples++;
      imuStats.ay.sumVal += ay_g;
      imuStats.ay.minVal = fminf(imuStats.ay.minVal, ay_g);
      imuStats.ay.maxVal = fmaxf(imuStats.ay.maxVal, ay_g);

      imuStats.az.samples++;
      imuStats.az.sumVal += az_g;
      imuStats.az.minVal = fminf(imuStats.az.minVal, az_g);
      imuStats.az.maxVal = fmaxf(imuStats.az.maxVal, az_g);

      float gx_dps = (gx - gx_off) / GYRO_SCALE;
      float gy_dps = (gy - gy_off) / GYRO_SCALE;
      float gz_dps = (gz - gz_off) / GYRO_SCALE;

      imuStats.gx.samples++;
      imuStats.gx.sumVal += gx_dps;
      imuStats.gx.minVal = fminf(imuStats.gx.minVal, gx_dps);
      imuStats.gx.maxVal = fmaxf(imuStats.gx.maxVal, gx_dps);

      imuStats.gy.samples++;
      imuStats.gy.sumVal += gy_dps;
      imuStats.gy.minVal = fminf(imuStats.gy.minVal, gy_dps);
      imuStats.gy.maxVal = fmaxf(imuStats.gy.maxVal, gy_dps);

      imuStats.gz.samples++;
      imuStats.gz.sumVal += gz_dps;
      imuStats.gz.minVal = fminf(imuStats.gz.minVal, gz_dps);
      imuStats.gz.maxVal = fmaxf(imuStats.gz.maxVal, gz_dps);
    } else {
      imuStats.ax.errors++;
    }

    if (millis() - lastReport >= 10000) {
      lastReport = millis();
      Serial.printf("• Time: %lus | Reads: %lu | Errors: %lu\n",
        (unsigned long)((lastReport - testStart) / 1000),
        imuStats.validReads, imuStats.ax.errors);
    }

    delay(100); // ~10 Hz
  }

  // Summary
  Serial.println(F("\n✓ IMU SUMMARY"));
  Serial.printf("  Valid Reads: %lu | Failed: %lu\n", imuStats.validReads, imuStats.ax.errors);
  Serial.printf("  Accel (g): AX[%.3f, %.3f] AY[%.3f, %.3f] AZ[%.3f, %.3f]\n",
    imuStats.ax.minVal, imuStats.ax.maxVal,
    imuStats.ay.minVal, imuStats.ay.maxVal,
    imuStats.az.minVal, imuStats.az.maxVal);
  Serial.printf("  Gyro (dps): GX[%.1f, %.1f] GY[%.1f, %.1f] GZ[%.1f, %.1f]\n",
    imuStats.gx.minVal, imuStats.gx.maxVal,
    imuStats.gy.minVal, imuStats.gy.maxVal,
    imuStats.gz.minVal, imuStats.gz.maxVal);

  if (imuStats.ax.errors > 5) {
    Serial.println(F("  ⚠️  WARNING: Multiple I2C read failures — check wiring & pull-ups"));
  } else if (imuStats.validReads < 250) {
    Serial.println(F("  ⚠️  WARNING: Expected ~300 samples (10 Hz × 30s) — slow I2C or timeout"));
  } else {
    Serial.println(F("  ✓ STABLE"));
  }
}

// ─────────────────────────────────────────────────────────────────────────
// CAMERA TEST
// ─────────────────────────────────────────────────────────────────────────

void testCamera() {
  Serial.println(F("\n▶ CAMERA (OV2640) TEST (30 seconds)"));
  Serial.println(F("──────────────────────────────────────────────────────"));

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

  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  if (psramFound()) {
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    Serial.println(F("  PSRAM found — using 2 frame buffers"));
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
    Serial.println(F("  ⚠️  No PSRAM — falling back to QVGA"));
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("  ✗ FAILED: Camera init error 0x%x\n", err);
    return;
  }
  Serial.println(F("  Camera initialized OK"));

  uint32_t testStart = millis();
  uint32_t lastReport = testStart;

  while (millis() - testStart < 30000) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      if (fb->format == PIXFORMAT_JPEG) {
        cameraStats.framesCaptured++;
        uint32_t frameSize = fb->len;

        cameraStats.minFrameSize = fminf(cameraStats.minFrameSize, frameSize);
        cameraStats.maxFrameSize = fmaxf(cameraStats.maxFrameSize, frameSize);
        cameraStats.avgFrameSize = (cameraStats.avgFrameSize * (cameraStats.framesCaptured - 1) + frameSize) / cameraStats.framesCaptured;
      }
      esp_camera_fb_return(fb);
    } else {
      cameraStats.framesFailed++;
    }

    if (millis() - lastReport >= 10000) {
      lastReport = millis();
      uint32_t elapsed = lastReport - testStart;
      float fps = (float)cameraStats.framesCaptured / ((float)elapsed / 1000.0f);
      Serial.printf("• Time: %lus | Frames: %lu | Dropped: %lu | FPS: %.1f\n",
        elapsed / 1000, cameraStats.framesCaptured, cameraStats.framesFailed, fps);
    }

    delay(50); // ~20 FPS
  }

  // Summary
  Serial.println(F("\n✓ CAMERA SUMMARY"));
  uint32_t totalFrames = cameraStats.framesCaptured + cameraStats.framesFailed;
  float fps = (float)cameraStats.framesCaptured / 30.0f;
  float dropRate = totalFrames > 0 ? 100.0f * (float)cameraStats.framesFailed / (float)totalFrames : 0.0f;

  Serial.printf("  Frames: %lu captured | %lu dropped | Drop rate: %.2f%%\n",
    cameraStats.framesCaptured, cameraStats.framesFailed, dropRate);
  Serial.printf("  Effective FPS: %.1f\n", fps);
  Serial.printf("  JPEG Size: min=%lu B, max=%lu B, avg=%lu B\n",
    cameraStats.minFrameSize, cameraStats.maxFrameSize, cameraStats.avgFrameSize);

  if (cameraStats.framesFailed > cameraStats.framesCaptured / 5) {
    Serial.println(F("  ⚠️  WARNING: High drop rate — check PSRAM, SD, or I2S bus conflicts"));
  } else if (fps < 18.0f) {
    Serial.println(F("  ⚠️  WARNING: FPS below target (20) — possible bus contention"));
  } else {
    Serial.println(F("  ✓ STABLE"));
  }
}

// ─────────────────────────────────────────────────────────────────────────
// SETUP & LOOP
// ─────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(921600);
  delay(1000);

  analogReadResolution(12);

  Serial.println(F("\n╔════════════════════════════════════════════════════════╗"));
  Serial.println(F("║         IoT FATIGUE HELMET — SENSOR TEST SUITE         ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════╝"));
  Serial.println(F("\nThis test validates pulse sensor, IMU, and camera stability."));
  Serial.println(F("Each subsystem runs for 30 seconds independently.\n"));

  Serial.println(F("IMU-only test (pulse sensor/camera skipped)."));
  Serial.println(F("Keep the board still, starting shortly."));
  for (int i = 15; i > 0; i--) {
    Serial.printf("  Starting in %d...\n", i);
    delay(1000);
  }

  testIMU();

  Serial.println(F("\n╔════════════════════════════════════════════════════════╗"));
  Serial.println(F("║                   TEST COMPLETE                        ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════╝\n"));
}

void loop() {
  delay(1000);
}
