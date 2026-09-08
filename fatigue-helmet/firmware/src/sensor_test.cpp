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
#define BUZZER_PIN 14
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
  // Force a full device reset before waking it -- see the identical fix
  // (and the reasoning behind it) in main.cpp's tryInitMPU().
  mpu.reset();
  delay(120);
  mpu.initialize();
  uint8_t whoami = mpu.getDeviceID();
  Serial.printf("  MPU at 0x%02X | WHO_AM_I: 0x%02X\n", foundAddr, whoami);

  // getDeviceID() returns the 6-bit device ID field (register 0x75 bits
  // 6:1), NOT the raw I2C address -- 0x68/0x69 here were never valid
  // values for it to return. Match the MPU6050 library's own
  // testConnection() whitelist instead: 0x34 is the common value, but
  // 0x0C and 0x3A are documented hardware-revision variants of the same
  // genuine chip (see MPU6050_Base::testConnection() in MPU6050.cpp).
  if (whoami != 0x34 && whoami != 0x0C && whoami != 0x3A) {
    return 0;
  }

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  return foundAddr;
}

// Free a wedged I2C bus before touching Wire.
//
// If a slave was interrupted mid-byte (MCU reset during a read, or stray
// edges on the lines) it keeps driving SDA low waiting for the clocks that
// finish its transfer. An MCU reset does NOT clear that -- the slave is a
// separate chip -- so the ESP32 I2C peripheral can never even issue a START
// and every transaction returns endTransmission()=5 (timeout), which reads
// as "sensor missing" when the sensor is perfectly healthy.
//
// The standard recovery is to bit-bang up to 9 clock pulses on SCL, which
// walks the stuck slave through the rest of its byte until it releases SDA,
// then issue a manual STOP. Returns true if SDA came back high.
static bool i2cBusRecover(uint8_t sda, uint8_t scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delayMicroseconds(10);

  if (digitalRead(sda) == HIGH) return true;   // bus already idle

  Serial.println(F("  SDA held LOW -- bus is wedged; clocking it free..."));
  for (int i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
    pinMode(scl, OUTPUT);
    digitalWrite(scl, LOW);
    delayMicroseconds(5);
    pinMode(scl, INPUT_PULLUP);          // release, let pull-up raise it
    delayMicroseconds(5);
  }

  // Manual STOP: SDA low -> high while SCL is high.
  pinMode(sda, OUTPUT); digitalWrite(sda, LOW);  delayMicroseconds(5);
  pinMode(scl, INPUT_PULLUP);                    delayMicroseconds(5);
  pinMode(sda, INPUT_PULLUP);                    delayMicroseconds(5);

  bool ok = digitalRead(sda) == HIGH;
  Serial.printf("  Bus recovery %s (SDA now %s)\n",
                ok ? "OK" : "FAILED", ok ? "HIGH" : "still LOW");
  return ok;
}

void testIMU() {
  Serial.println(F("\n▶ IMU (MPU-6050) TEST (15 seconds)"));
  Serial.println(F("──────────────────────────────────────────────────────"));

  // Clear any wedged-bus condition left by a previous run before Wire
  // takes the pins over -- otherwise a stuck slave makes a healthy
  // sensor look absent.
  i2cBusRecover(PIN_SDA, PIN_SCL);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);   // bound each transaction; default can stall the scan

  if (initMPU() == 0) {
    Serial.println(F("  ✗ FAILED: MPU-6050 not detected at 0x68/0x69"));

    // Report the RAW endTransmission() error code for the two real
    // addresses, not just pass/fail -- code 2 (NACK on address) is a
    // clean "nobody answered," code 4/5 (bus error/timeout) means the
    // bus itself isn't behaving, which is a different problem than "the
    // chip is absent." See Arduino Wire docs for the code meanings.
    for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      const char *meaning =
          (err == 0) ? "success (shouldn't happen here)" :
          (err == 1) ? "data too long" :
          (err == 2) ? "NACK on address -- bus OK, nothing answered" :
          (err == 3) ? "NACK on data" :
          (err == 4) ? "other error" :
          (err == 5) ? "timeout -- bus itself may be stuck" : "unknown";
      Serial.printf("    0x%02X -> endTransmission()=%d (%s)\n", addr, err, meaning);
    }

    Serial.println(F("  Running full I2C bus scan (0x01-0x7F) for diagnosis..."));
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      if (err == 0) {
        Serial.printf("    Device found at 0x%02X\n", addr);
        found++;
      }
    }
    if (found == 0) {
      Serial.println(F("    No I2C devices found at all -- check wiring/power, not just address."));
    }

    // ── GPIO wiggle test: prove the ESP32 pins themselves can be driven,
    // independent of I2C/Wire entirely. Detaches from I2C, drives each pin
    // as a plain digital output through a slow HIGH/LOW/HIGH/LOW pattern
    // for 6 seconds so a multimeter (or even watching an LED/scope) can
    // confirm the pins are alive on the ESP32 side, isolating "ESP32 GPIO
    // is dead" from "the MPU-6050 chip is dead."
    Serial.println(F("  GPIO wiggle test: watch SDA (pin 2) and SCL (pin 3) with your"));
    Serial.println(F("  multimeter now -- each should swing between ~0V and ~3.3V,"));
    Serial.println(F("  alternating, for the next 6 seconds."));
    pinMode(PIN_SDA, OUTPUT);
    pinMode(PIN_SCL, OUTPUT);
    for (int i = 0; i < 12; i++) {
      digitalWrite(PIN_SDA, i % 2);
      digitalWrite(PIN_SCL, (i + 1) % 2);
      Serial.printf("    t=%ds  SDA=%s  SCL=%s\n", i / 2 + 1,
                    (i % 2) ? "HIGH" : "LOW", ((i + 1) % 2) ? "HIGH" : "LOW");
      delay(500);
    }
    // Hand the pins back as inputs. Leaving them as push-pull outputs
    // (the loop above exits with SCL driven LOW) clamps the clock line
    // and wedges the bus for every later run -- the exact failure this
    // diagnostic exists to explain.
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);

    Serial.println(F("  Wiggle test complete. If you saw NO voltage change on either pin,"));
    Serial.println(F("  the ESP32 GPIO itself is the problem, not the MPU-6050."));
    Serial.println(F("  If both pins DID swing, the ESP32 side is confirmed fine and"));
    Serial.println(F("  the MPU-6050 chip is the remaining suspect."));
    return;
  }
  g_mpuEnabled = true;

  // Calibrate -- GYRO BIAS ONLY.
  //
  // Accumulators are int32_t, not int16_t. A single raw accel sample at
  // 1 g is 16384 LSB, so summing 100 of them reaches ~1.6e6 and wraps an
  // int16_t (max 32767) many times over. That overflow silently produced
  // garbage offsets -- measured az_off=273 when the true mean was ~2894
  // (100*2894 mod 65536 / 100 = 272.6) -- which is why the documented
  // +4.2 dps gyro-X bias survived "calibration" completely untouched.
  //
  // Accel is deliberately NOT offset-corrected. Gravity is the reference
  // signal: the acceptance test is |A| = 1 g at rest, which is only
  // meaningful with gravity left in. Zeroing it removes the very quantity
  // being validated.
  Serial.println(F("  Calibrating gyro bias (keep still)..."));
  int32_t gx_acc = 0, gy_acc = 0, gz_acc = 0;
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gx_acc += gx; gy_acc += gy; gz_acc += gz;
    delay(5);
  }
  const int16_t ax_off = 0, ay_off = 0, az_off = 0;   // gravity preserved
  int16_t gx_off = (int16_t)(gx_acc / 100);
  int16_t gy_off = (int16_t)(gy_acc / 100);
  int16_t gz_off = (int16_t)(gz_acc / 100);

  Serial.printf("  Gyro bias: raw[%d %d %d] = %+.2f %+.2f %+.2f dps\n",
    gx_off, gy_off, gz_off,
    gx_off / GYRO_SCALE, gy_off / GYRO_SCALE, gz_off / GYRO_SCALE);

  float amagSum = 0.0f, amagMin = 1e9f, amagMax = -1e9f;

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

      float amag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
      amagSum += amag;
      amagMin = fminf(amagMin, amag);
      amagMax = fmaxf(amagMax, amag);

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

  // Loop runs for 15000ms at ~10 Hz (100ms delay/iter) -> ~150 samples
  // expected. 130 (~87%) leaves headroom for I2C/scheduling jitter
  // without masking a genuine slow-bus problem.
  // Physics acceptance test from IMU_INTEGRATION.md section 5: at rest,
  // in ANY orientation, |A| must land in 0.98-1.03 g. This is the only
  // check that validates scale and wiring rather than mere liveness --
  // and it is why accel offsets are no longer subtracted above.
  float amagAvg = imuStats.validReads ? amagSum / (float)imuStats.validReads : 0.0f;
  Serial.printf("  |A| (g): avg=%.4f  min=%.4f  max=%.4f\n",
    amagAvg, amagMin, amagMax);

  bool gravityOk = (amagAvg >= 0.98f && amagAvg <= 1.03f);

  if (imuStats.ax.errors > 5) {
    Serial.println(F("  ⚠️  WARNING: Multiple I2C read failures — check wiring & pull-ups"));
  } else if (!gravityOk) {
    Serial.println(F("  ⚠️  WARNING: |A| outside 0.98-1.03 g at rest — check the accel"));
    Serial.println(F("     full-scale range setting or the wiring, not the formula."));
  } else if (imuStats.validReads < 130) {
    Serial.println(F("  ⚠️  WARNING: Expected ~150 samples (10 Hz × 15s) — slow I2C or timeout"));
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
// BUZZER TEST — direct GPIO14 drive, independent of the alert state
// machine/mute flag in main.cpp. Isolates "is it wired correctly" from
// any firmware logic (g_buzzerMuted, EMI-suspend rules, etc).
// ─────────────────────────────────────────────────────────────────────────

void testBuzzer() {
  Serial.println(F("\n▶ BUZZER TEST (GPIO 14)"));
  Serial.println(F("──────────────────────────────────────────────────────"));
  Serial.println(F("  Listen now: 3 beeps, 500ms on / 500ms off."));
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  for (int i = 0; i < 3; i++) {
    Serial.printf("    beep %d: ON\n", i + 1);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    Serial.printf("    beep %d: OFF\n", i + 1);
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);
  }
  Serial.println(F("  Buzzer test complete. If you heard 3 beeps, GPIO14 wiring is"));
  Serial.println(F("  correct. If silent, check the buzzer's wiring/polarity/driver"));
  Serial.println(F("  transistor (see the buzzer wiring discussion) before assuming"));
  Serial.println(F("  a firmware problem -- this test bypasses all alert logic."));
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

  // Buzzer test disabled: the buzzer VCC/GND are disconnected on this rig
  // and it is out of current test scope. testBuzzer() is left defined so
  // it can be re-enabled by uncommenting the line below.
  // testBuzzer();

  Serial.println(F("Full suite: pulse sensor, IMU, camera."));
  Serial.println(F("Have a finger ready for the pulse sensor and keep the board"));
  Serial.println(F("still for the IMU. Starting shortly."));
  for (int i = 15; i > 0; i--) {
    Serial.printf("  Starting in %d...\n", i);
    delay(1000);
  }

  // Dump the unfiltered ADC stream BEFORE the beat detector runs. The
  // aggregate min/max/avg cannot distinguish "a real heartbeat is present"
  // from "wideband noise of the same amplitude" -- only the waveform shape
  // can, and 0 detected beats is exactly the case where that matters.
  dumpRawWaveform(20000, 10);   // 20 s @ 100 Hz = 2000 samples

  testPulseSensor();
  testIMU();
  testCamera();

  Serial.println(F("\n╔════════════════════════════════════════════════════════╗"));
  Serial.println(F("║                   TEST COMPLETE                        ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════╝\n"));
}

void loop() {
  delay(1000);
}
