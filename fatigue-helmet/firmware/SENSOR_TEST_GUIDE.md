# Sensor Stability Test Guide

A standalone diagnostic tool to validate **pulse sensor**, **IMU**, and **camera** stability before running the full firmware.

## Quick Start

### 1. **Compile & Upload**

```bash
cd fatigue-helmet/firmware
pio run -e sensor_test -t upload --upload-port COM_PORT
```

Replace `COM_PORT` with your board's serial port (e.g., `COM3` on Windows, `/dev/ttyUSB0` on Linux).

### 2. **Open Monitor**

```bash
pio device monitor -b 921600 -p COM_PORT
```

The test will run automatically and produce a detailed stability report.

---

## What Gets Tested

### **Pulse Sensor (GPIO 1, ADC @ 500 Hz)**

**Duration:** 30 seconds  
**Checks:**
- Raw ADC range (expected 200–3900)
- Signal quality (peak-valley amplitude ≥ 20)
- Beat detection stability (BPM range)
- Contact signal loss rate

**Pass Criteria:**
- ✓ Error rate < 10% (allow some dropout if finger shifts)
- ✓ At least 10 valid beats detected
- ✓ Steady BPM range (no wild spikes)

**Troubleshooting:**
| Issue | Cause | Fix |
|-------|-------|-----|
| High error rate (>20%) | No skin contact | Ensure good fingertip pressure on sensor |
| Few valid beats (<5) | Weak signal | Adjust finger angle, try different area of fingertip |
| Raw ADC stuck at 0 or 4095 | ADC clip / wiring issue | Check GPIO 1 wiring, verify ADC resolution = 12-bit |
| Erratic BPM jumps | Noise, loose connector | Clean sensor, check cable contacts |

---

### **IMU: MPU-6050 (I2C @ 0x68, 10 Hz)**

**Duration:** 30 seconds (~300 samples)  
**Checks:**
- WHO_AM_I register (must be 0x68, 0x69, 0x38, 0x70, or 0x72)
- Calibration offsets computed
- Accel range in expected window (±2 g after offset)
- Gyro range in expected window (±250 °/s after offset)
- Read success rate

**Pass Criteria:**
- ✓ 0–5 I2C errors over 30 seconds
- ✓ ≥ 290 valid samples (10 Hz × 30s)
- ✓ Accel magnitude within reasonable bounds (~1 g if level)
- ✓ Gyro values reasonable (low noise when still, detectable motion when moved)

**Troubleshooting:**
| Issue | Cause | Fix |
|-------|-------|-----|
| "MPU-6050 not detected" | Device not on I2C bus | Check VCC/GND, SDA (GPIO 2), SCL (GPIO 3) wiring |
| Multiple I2C read failures | Loose connections, pull-up resistors | Verify 4.7kΩ pull-ups on SDA/SCL; check solder joints |
| Accel out of range (>2g steady) | Calibration issue or sensor tilted | Redo calibration with sensor perfectly still & level |
| Gyro readings stuck at 0 | I2C timeout or sensor hang | Power-cycle, check I2C clock (100 kHz expected) |

---

### **Camera: OV2640 (JPEG @ 20 FPS)**

**Duration:** 30 seconds (~600 frames)  
**Checks:**
- Frame capture success rate
- Frame drop tracking
- JPEG size consistency (all valid frames should be ±10% of mean)
- Effective FPS

**Pass Criteria:**
- ✓ Drop rate < 5% (allow brief pauses)
- ✓ Effective FPS ≥ 18 (allows for occasional congestion)
- ✓ JPEG sizes within ±20% of average (indicates stable compression)

**Troubleshooting:**
| Issue | Cause | Fix |
|-------|-------|-----|
| "Camera init error 0x..." | Wiring or pin conflict | Verify camera cable, check GPIO assignments (see main.cpp lines 74–90) |
| High drop rate (>10%) | SD write contention, low PSRAM | If using SD: ensure PSRAM is available; reduce FPS to 10 |
| FPS < 15 | I2S bus congestion with other sensors | Try turning off SD writes during test; check for competing I2C traffic |
| JPEG size wildly inconsistent | Compression instability | Check JPEG_QUALITY=10 (line 62 main.cpp); lower number = better |

---

## Sample Output

```
╔════════════════════════════════════════════════════════╗
║         IoT FATIGUE HELMET — SENSOR TEST SUITE         ║
╚════════════════════════════════════════════════════════╝

▶ PULSE SENSOR TEST (30 seconds)
──────────────────────────────────────────────────────
• Time: 0s | Raw ADC: [1234..3456] avg=2345 | Beats: 0 | BPM: [0.0..0.0]
• Time: 10s | Raw ADC: [1200..3500] avg=2400 | Beats: 12 | BPM: [58.8..65.2]
• Time: 20s | Raw ADC: [1150..3520] avg=2410 | Beats: 24 | BPM: [59.5..64.1]
• Time: 30s | Raw ADC: [1100..3500] avg=2405 | Beats: 35 | BPM: [60.0..63.5]

✓ PULSE SENSOR SUMMARY
  Samples: 15000 | Errors (no contact): 120 | Error rate: 0.80%
  ADC Range: [1100, 3520] | Avg: 2405
  Valid Beats: 35 | BPM Range: [58.8, 65.2]
  ✓ STABLE

▶ IMU (MPU-6050) TEST (30 seconds)
──────────────────────────────────────────────────────
  MPU at 0x68 | WHO_AM_I: 0x68
  Calibrating (keep still)...
  Offsets: ax=512 ay=-256 az=16384 | gx=12 gy=-8 gz=5
• Time: 0s | Reads: 3 | Errors: 0
• Time: 10s | Reads: 102 | Errors: 0
• Time: 20s | Reads: 201 | Errors: 0
• Time: 30s | Reads: 300 | Errors: 0

✓ IMU SUMMARY
  Valid Reads: 300 | Failed: 0
  Accel (g): AX[-0.031, 0.025] AY[-0.012, 0.038] AZ[0.987, 1.025]
  Gyro (dps): GX[-2.3, 1.8] GY[-1.5, 2.1] GZ[-1.0, 0.9]
  ✓ STABLE

▶ CAMERA (OV2640) TEST (30 seconds)
──────────────────────────────────────────────────────
  PSRAM found — using 2 frame buffers
  Camera initialized OK
• Time: 0s | Frames: 6 | Dropped: 0 | FPS: 20.0
• Time: 10s | Frames: 200 | Dropped: 1 | FPS: 20.1
• Time: 20s | Frames: 400 | Dropped: 2 | FPS: 20.0
• Time: 30s | Frames: 600 | Dropped: 3 | FPS: 20.0

✓ CAMERA SUMMARY
  Frames: 600 captured | 3 dropped | Drop rate: 0.50%
  Effective FPS: 20.0
  JPEG Size: min=8192 B, max=9856 B, avg=8945 B
  ✓ STABLE

╔════════════════════════════════════════════════════════╗
║                   TEST COMPLETE                        ║
╚════════════════════════════════════════════════════════╝
```

---

## Interpreting Results

### All Sensors ✓ STABLE
Your hardware is ready for full firmware. No further troubleshooting needed.

### Some Sensors Show ⚠️ WARNING
Review the specific sensor's troubleshooting table above. Common causes:
- **Pulse:** Insufficient finger pressure, sensor misalignment
- **IMU:** Loose I2C connections, missing pull-up resistors
- **Camera:** SD/PSRAM bandwidth contention, frame buffer sizing

### Any Sensor ✗ FAILED
Check wiring, power supply, and GPIO pin assignments. Refer to:
- Pulse: `main.cpp` lines 92–107
- IMU: `main.cpp` lines 123–135
- Camera: `main.cpp` lines 74–90

---

## Advanced: Running Full Firmware After Test

Once sensors pass, return to your normal build environment:

```bash
# Debug mode (USB streaming)
pio run -e esp32s3cam -t upload --upload-port COM_PORT

# Or production mode (SD card)
pio run -e esp32s3cam_sd -t upload --upload-port COM_PORT
```

---

## Notes

- **Pulse Sensor:** Requires you to place a fingertip on the sensor *before* the test starts. No beat detection expected in the first 5–10 seconds (envelope tracking needs data).
- **IMU:** Will calibrate for ~2 seconds at the start. **Keep the device perfectly still** during calibration.
- **Camera:** PSRAM presence is auto-detected. If not available, test drops to QVGA resolution (internal DRAM buffer).

---

## Sensor Pinout Reference

| Component | Pin(s) | Mode |
|-----------|--------|------|
| Pulse Sensor | GPIO 1 | ADC1 (analog read) |
| MPU-6050 SDA | GPIO 2 | I2C |
| MPU-6050 SCL | GPIO 3 | I2C |
| Camera Data | GPIO 8–18 | Parallel I2S |
| Camera Sync | GPIO 6, 7, 13, 15 | I2S control |

For full pinout, see `main.cpp` lines 74–90.
