/**
 * IoT Helmet — Fatigue Detection Firmware (Phase 1)
 * Board  : ESP32-S3 generic dev board
 * Sensors: Analog Pulse Sensor @ GPIO 1 (ADC1)
 *          MPU-6050 GY-521 (Accel + Gyro) @ I2C 0x68
 *
 * Output : CSV lines over Serial at 10 Hz
 *          Lines starting with # are status/error (Python logger ignores them)
 */

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"   // electroniccats/MPU6050

// ── Pulse Sensor constants ────────────────────────────────────────────────
#define PULSE_PIN           1     // GPIO 1, ADC1 channel 0 on ESP32-S3
#define SAMPLE_RATE_MS      10    // 10 ms = 100 Hz sampling for beat detection
#define FILTER_SIZE         10    // rolling average window size
#define BPM_BUFFER_SIZE     5     // beats averaged for stable BPM output
#define MIN_BPM             30    // reject intervals longer than 2000 ms
#define MAX_BPM             200   // reject intervals shorter than 300 ms
#define SIGNAL_LOW_THRESH   500   // ADC < this → likely no skin contact
#define SIGNAL_HIGH_THRESH  3500  // ADC > this → sensor saturated / too much ambient light

// ── I2C & output timing constants ────────────────────────────────────────
const uint8_t  PIN_SDA       = 8;
const uint8_t  PIN_SCL       = 9;
const uint32_t LOOP_MS       = 100;  // 10 Hz CSV output rate

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2   // adjust if your ESP32-S3 board uses a different GPIO
#endif

// ── MPU-6050 constants ────────────────────────────────────────────────────
const uint8_t MPU_ADDR      = 0x68;
const float   ACCEL_SCALE   = 16384.0f;  // LSB/g at ±2 g range
const float   GYRO_SCALE    = 131.0f;    // LSB/(°/s) at ±250 °/s range
const int     CALIB_SAMPLES = 200;

// ── Pulse sensor peak-detection globals ───────────────────────────────────
int   filterBuffer[FILTER_SIZE] = {0};
int   filterIndex      = 0;
long  beatIntervals[BPM_BUFFER_SIZE] = {0};
int   beatIndex        = 0;
long  lastBeatTime     = 0;
float currentBPM       = 0;
bool  risingSignal     = false;   // true when last sample was above threshold
int   dynamicThreshold = 2048;    // auto-updated midpoint of peak/valley
int   peakValue        = 0;       // tracks signal ceiling (decays slowly)
int   valleyValue      = 4095;    // tracks signal floor (decays slowly)

// Shared output state — written by readPulseSensor(), read in loop output
int lastPulseRaw      = 0;
int lastSignalQuality = 0;

// ── No-contact warning state ──────────────────────────────────────────────
int noContactStreak = 0;
const int NO_CONTACT_WARN_N = 10;  // consecutive 10 Hz output ticks before warning

// ── MPU-6050 ─────────────────────────────────────────────────────────────
MPU6050 mpu(MPU_ADDR);
int16_t ax_off = 0, ay_off = 0, az_off = 0;
int16_t gx_off = 0, gy_off = 0, gz_off = 0;

// ─────────────────────────────────────────────────────────────────────────
// readPulseSensor() — called at 100 Hz
// Updates globals: lastPulseRaw, currentBPM, lastSignalQuality
// ─────────────────────────────────────────────────────────────────────────

void readPulseSensor() {
    // 1. Raw ADC (12-bit: 0–4095 on ESP32-S3)
    int raw = analogRead(PULSE_PIN);
    lastPulseRaw = raw;

    // 2. Rolling average to suppress high-frequency noise
    filterBuffer[filterIndex] = raw;
    filterIndex = (filterIndex + 1) % FILTER_SIZE;
    long filterSum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) filterSum += filterBuffer[i];
    int filtered = (int)(filterSum / FILTER_SIZE);

    // 3. Adaptive threshold: track signal peak and valley; threshold = midpoint.
    // Slow 1-LSB decay per sample lets the threshold follow slow baseline drift
    // without reacting to each individual heartbeat.
    if (filtered > peakValue)    peakValue   = filtered;
    else                          peakValue  -= 1;

    if (filtered < valleyValue)  valleyValue  = filtered;
    else                          valleyValue += 1;

    dynamicThreshold = (peakValue + valleyValue) / 2;

    // 4. Rising-edge beat detection — fire once when signal crosses threshold upward
    bool wasRising = risingSignal;
    risingSignal   = (filtered > dynamicThreshold);

    if (!wasRising && risingSignal) {   // rising edge: one heartbeat
        long now      = millis();
        long interval = now - lastBeatTime;

        // Only store intervals in the physiologically valid range (MIN_BPM–MAX_BPM)
        if (lastBeatTime > 0 && interval >= 300 && interval <= 2000) {
            beatIntervals[beatIndex % BPM_BUFFER_SIZE] = interval;
            beatIndex++;

            // Average the last N intervals for stable BPM
            int count = (beatIndex < BPM_BUFFER_SIZE) ? beatIndex : BPM_BUFFER_SIZE;
            long totalMs = 0;
            for (int i = 0; i < count; i++) totalMs += beatIntervals[i];
            // BPM = 60 000 ms/min ÷ average_interval_ms
            currentBPM = (60000.0f * count) / (float)totalMs;
        }
        lastBeatTime = now;
    }

    // 5. Signal quality: bad if out-of-range or pulsatile amplitude too small
    int amplitude = peakValue - valleyValue;
    if (raw < SIGNAL_LOW_THRESH || raw > SIGNAL_HIGH_THRESH) {
        lastSignalQuality = 0;  // no contact or saturated by ambient light
    } else if (amplitude > 300) {
        lastSignalQuality = 1;  // clear pulsatile waveform
    } else {
        lastSignalQuality = 0;  // signal is flat — poor skin contact or wrong placement
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────

void blinkLED(int times) {
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(150);
        digitalWrite(LED_BUILTIN, LOW);  delay(150);
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
        ax_s += ax; ay_s += ay; az_s += az;
        gx_s += gx; gy_s += gy; gz_s += gz;
        delay(5); // 5 ms/sample × 200 samples = 1 s calibration window
    }

    ax_off = ax_s / CALIB_SAMPLES;
    ay_off = ay_s / CALIB_SAMPLES;
    az_off = az_s / CALIB_SAMPLES;
    gx_off = gx_s / CALIB_SAMPLES;
    gy_off = gy_s / CALIB_SAMPLES;
    gz_off = gz_s / CALIB_SAMPLES;

    Serial.print(F("#STATUS: Calibration done. Offsets: ax="));
    Serial.print(ax_off); Serial.print(F(" ay="));
    Serial.print(ay_off); Serial.print(F(" az="));
    Serial.print(az_off); Serial.print(F(" gx="));
    Serial.print(gx_off); Serial.print(F(" gy="));
    Serial.print(gy_off); Serial.print(F(" gz="));
    Serial.println(gz_off);
}

// ─────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    // Brief wait for USB-CDC to enumerate; avoids dropping early status lines
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);

    // 3 LED blinks confirm firmware loaded successfully
    blinkLED(3);

    // Pulse sensor is analog-only — no library or begin() needed
    analogReadResolution(12);  // 12-bit: 0–4095 (ESP32-S3 default, set explicitly)
    Serial.println(F("#STATUS: Pulse sensor on GPIO 1 (ADC1) — analog mode"));
    Serial.println(F("#STATUS: Sampling at 100Hz, BPM averaged over 5 beats"));

    // I2C only needed for MPU-6050 (pulse sensor is analog)
    Wire.begin(PIN_SDA, PIN_SCL);

    // ── MPU-6050 ──────────────────────────────────────────────────────────
    mpu.initialize();
    uint8_t whoami = mpu.getDeviceID();
    Serial.print(F("#STATUS: MPU WHO_AM_I = 0x"));
    Serial.println(whoami, HEX);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // ±2 g  → 16384 LSB/g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);    // ±250 °/s → 131 LSB/(°/s)
    Serial.println(F("#STATUS: MPU-6050 initialized OK"));

    calibrateMPU();

    // Header comment so Python logger can identify columns without hardcoding them
    Serial.println(F("#HEADER:timestamp_ms,hr_bpm,pulse_raw,"
                     "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,head_movement,signal_quality"));
    Serial.println(F("#STATUS: Logging started. Format: CSV"));
}

// ─────────────────────────────────────────────────────────────────────────
// Main loop — two independent timing tiers:
//   100 Hz → readPulseSensor() for accurate peak/beat detection
//    10 Hz → CSV output + MPU-6050 read
// ─────────────────────────────────────────────────────────────────────────

void loop() {
    unsigned long now = millis();

    // ── Pulse sensor: sample at 100 Hz ───────────────────────────────────
    static unsigned long lastSampleTime = 0;
    if (now - lastSampleTime >= SAMPLE_RATE_MS) {
        lastSampleTime = now;
        readPulseSensor();
    }

    // ── CSV output: 10 Hz ────────────────────────────────────────────────
    static unsigned long lastOutputTime = 0;
    if (now - lastOutputTime < LOOP_MS) return;
    lastOutputTime = now;

    // Warn after 10 consecutive bad-signal output ticks (~1 second of no contact)
    if (lastSignalQuality == 0) {
        if (++noContactStreak >= NO_CONTACT_WARN_N) {
            Serial.println(F("#WARNING: Pulse signal low — check sensor contact"));
            noContactStreak = 0;
        }
    } else {
        noContactStreak = 0;
    }

    // ── MPU-6050 ──────────────────────────────────────────────────────────
    int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
    mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);

    // Subtract calibration offsets — zeroes gravity so head_movement ≈ 0 at rest
    float ax_g  = (ax_r - ax_off) / ACCEL_SCALE;
    float ay_g  = (ay_r - ay_off) / ACCEL_SCALE;
    float az_g  = (az_r - az_off) / ACCEL_SCALE;
    float gx_dp = (gx_r - gx_off) / GYRO_SCALE;
    float gy_dp = (gy_r - gy_off) / GYRO_SCALE;
    float gz_dp = (gz_r - gz_off) / GYRO_SCALE;

    float head_movement = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

    // Zero BPM output when signal is bad or no beats have been accumulated yet
    int hr_out = (lastSignalQuality == 1 && currentBPM > 0) ? (int)currentBPM : 0;

    // ── CSV output — 11 fields ────────────────────────────────────────────
    Serial.print(now);              Serial.print(',');
    Serial.print(hr_out);           Serial.print(',');
    Serial.print(lastPulseRaw);     Serial.print(',');
    Serial.print(ax_g,  4);         Serial.print(',');
    Serial.print(ay_g,  4);         Serial.print(',');
    Serial.print(az_g,  4);         Serial.print(',');
    Serial.print(gx_dp, 4);         Serial.print(',');
    Serial.print(gy_dp, 4);         Serial.print(',');
    Serial.print(gz_dp, 4);         Serial.print(',');
    Serial.print(head_movement, 4); Serial.print(',');
    Serial.println(lastSignalQuality);
}
