# IoT Helmet Fatigue Detection — Phase 2: Camera Recording Brief

**Project:** Smart helmet fatigue detection system  
**Phase:** Phase 2 — Dataset Collection with Camera  
**Date:** June 2026  
**Status:** Ready for implementation

---

## 1. Overview

Upgrade dari Phase 1 (sensor only) ke Phase 2 (sensor + camera). Hardware: single **ESP32-S3-CAM** board yang menangani:
- Camera JPEG recording
- MPU-6050 (accelerometer + gyroscope) via I2C
- Analog pulse sensor
- Data synchronization dan storage

**Goal:** Koleksi dataset lengkap (video + physiological sensor) untuk desain fuzzy fatigue detection system.

---

## 2. Hardware Configuration

### Board
- **ESP32-S3-CAM**: Espressif ESP32-S3 + OV2640 camera built-in + 8MB PSRAM

### Sensors (I2C Bus, GPIO 8=SDA, GPIO 9=SCL)
| Sensor | Address | Notes |
|--------|---------|-------|
| MPU-6050 GY-521 | 0x68 | Accel + Gyro, AD0 → GND |
| Pulse Sensor (analog) | — | GPIO 1 (ADC1) |

### Storage
- **microSD card** (16-32GB, Class 10 or U3)
- Format: FAT32

---

## 3. Recording Approach

### 3.1 Camera — QVGA 15fps JPEG to SD Card

```
Resolution  : 320×240 (QVGA) — balance antara storage + MediaPipe compatibility
Format      : JPEG (lossy compression)
Frame rate  : 15 fps → 67ms per frame
File naming : {timestamp_ms}.jpg
Example     : 123456789.jpg (recorded at t=123456789 ms from boot)

Storage rate: ~10-15 KB/frame
            : 150-225 KB/sec
            : ~9-13.5 MB/min
            : ~27-40 MB per 30-min session
```

**SD Card usage for 30-min session:**
- 8GB → dapat ~6-7 sessions
- 16GB → dapat ~12-15 sessions
- 32GB → dapat ~24-30 sessions

### 3.2 Sensor Data — CSV 10Hz to SD Card

**Kontinu logging ke serial + SD card.**

Same format sebagai Phase 1:
```
timestamp_ms, hr_bpm, pulse_raw, 
ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, 
head_movement, signal_quality
```

Update rate: **10 Hz** (setiap 100ms)

**SD Card usage for 30-min session:**
```
~1 KB per 100ms = 10 KB/sec = 600 KB/min = ~18 MB per 30-min
```

---

## 4. Synchronization Strategy

### 4.1 Timestamp-Based Sync (Recommended)

Keduanya (camera + sensor) gunakan **`millis()`** dari ESP32-S3 boot. Sinkronisasi dijamin kalau keduanya start pada waktu yang sama.

```
Boot ESP32-S3-CAM
    ↓
t=0 ms: Kedua subsystem (camera + sensor) start simultaneously
    ↓
Camera saves JPEG: 0.jpg, 67.jpg, 134.jpg, 201.jpg, ...
Sensor logs CSV : 0ms, 100ms, 200ms, 300ms, ...
    ↓
PC post-processing: match frame ke sensor row by nearest timestamp
```

**Timestamp accuracy:** ±67ms (1 frame period) acceptable untuk fatigue detection.

### 4.2 Metadata File (Optional)

Buat file `session_metadata.txt` di root SD card setiap session:
```
session_start_ms=0
session_duration_ms=1800000
camera_fps=15
camera_resolution=320x240
sensor_sample_rate=10Hz
mpu_address=0x68
pulse_pin=1
```

---

## 5. SD Card Directory Structure

```
SD Card Root/
├── sessions/
│   ├── session_001/
│   │   ├── metadata.txt
│   │   ├── sensor_data.csv
│   │   ├── frames/
│   │   │   ├── 0.jpg
│   │   │   ├── 67.jpg
│   │   │   ├── 134.jpg
│   │   │   └── ... (1800 frames untuk 30-min @ 15fps)
│   │
│   ├── session_002/
│   │   └── (same structure)
│   └── ...
```

Alternatif (flat structure untuk simplicity):
```
SD Card Root/
├── session_001_sensor.csv
├── session_001_0.jpg
├── session_001_67.jpg
├── session_001_134.jpg
├── ...
├── session_002_sensor.csv
├── session_002_0.jpg
└── ...
```

---

## 6. Firmware Architecture

### Dual-Task Approach

```cpp
// Task 1: Camera — capture @ 15fps (67ms interval)
void cameraTask() {
    while (true) {
        camera_fb_t* fb = esp_camera_fb_get();
        
        if (fb) {
            // Save to SD: {millis()}.jpg
            char filename[32];
            sprintf(filename, "/%lld.jpg", esp_timer_get_time() / 1000);
            // write fb->buf to SD
            esp_camera_fb_return(fb);
        }
        
        vTaskDelay(67 / portTICK_PERIOD_MS);  // ~15fps
    }
}

// Task 2: Sensors — 100Hz sampling, 10Hz CSV output
void sensorTask() {
    while (true) {
        // Read pulse @ 100Hz
        readPulseSensor();
        
        // Read MPU @ 100Hz
        readMPU();
        
        // Output CSV @ 10Hz
        if (now % 100 == 0) {
            printCSVLine();
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);  // 100Hz sampling
    }
}

// Main: init both, start dual-core execution
void setup() {
    initCamera();
    initSDCard();
    initMPU();
    initSerial();
    
    xTaskCreatePinnedToCore(cameraTask, "Camera", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(sensorTask, "Sensor", 4096, NULL, 2, NULL, 1);
}
```

---

## 7. Implementation Checklist

### Hardware Setup
- [ ] ESP32-S3-CAM + OV2640 camera initialized
- [ ] MPU-6050 connected (I2C GPIO 8/9, address 0x68, AD0→GND)
- [ ] Pulse sensor connected (GPIO 1 ADC)
- [ ] microSD card formatted FAT32, tested

### Firmware
- [ ] Camera initialization (QVGA, 15fps, JPEG)
- [ ] SD card SPI initialization
- [ ] JPEG frame save logic with `millis()` filename
- [ ] MPU-6050 I2C + pulse sensor ADC (same as Phase 1)
- [ ] CSV logging to SD card + Serial
- [ ] Dual-task scheduling (camera on core 0, sensor on core 1)
- [ ] Graceful SD card full handling (optional: stop or wrap around)
- [ ] Serial output format with `#` status lines (ignored by logger)

### Testing
- [ ] Camera captures frames at 15fps (check framerate)
- [ ] JPEG files readable on PC
- [ ] Sensor CSV logged correctly
- [ ] Timestamps match (frame 67.jpg ≈ sensor row at ~67ms)
- [ ] SD card write speed stable (~150-200 KB/s)
- [ ] 30-min full session test (storage, no frame drops)

### Data Collection Protocol
- [ ] Insert SD card, power on board
- [ ] Wait for 3 LED blinks + calibration (~4s)
- [ ] Press record/start (manual or pin-triggered)
- [ ] Record 20-30 min riding simulation
- [ ] Power off, extract SD card
- [ ] Transfer files to PC for processing

---

## 8. Post-Processing on PC

After recording session:

1. **Extract files from SD card**
   ```
   session_001/
   ├── sensor_data.csv
   └── frames/
       ├── 0.jpg
       ├── 67.jpg
       └── ...
   ```

2. **Process with MediaPipe** (Python script)
   - Loop all JPEG files
   - Extract eye landmarks (468 points)
   - Calculate EAR per frame
   - Detect blinks
   - Calculate PERCLOS (per 60s window)
   - Output: `camera_features.csv`

3. **Merge** sensor CSV + camera features CSV by timestamp

4. **Label** with KSS (Karolinska Sleepiness Scale, 1-9)

5. **Design fuzzy** membership functions from merged dataset

---

## 9. Known Constraints & Solutions

| Issue | Constraint | Solution |
|-------|-----------|----------|
| SD I2C conflict | ESP32-CAM: SD GPIO12/13 = I2C? | Use 1-bit SD mode; only S3-CAM has this resolved |
| Frame rate jitter | 15fps nominal, actual ±1fps | Accept ±67ms jitter; post-process timestamps |
| Large dataset | 30min @ 15fps = 27000 JPEG + CSV | 32GB SD handles ~20-30 sessions |
| USB CDC Serial | Slow for 10Hz CSV + camera streaming | Log to SD instead; Serial for debug only |
| Memory (PSRAM) | 8MB limited | Camera frame buffer ~80KB, ok for 1 frame at a time |

---

## 10. Deliverables

### From ESP32-S3-CAM
- SD card with `session_XXX/` folders
  - `sensor_data.csv` (10Hz, 11 columns)
  - `frames/` folder (15fps QVGA JPEG, 1800+ frames per 30min)
  - `metadata.txt` (session info)

### Processing Output
- `camera_features.csv` (timestamp, ear, blink_rate, perclos, ...)
- `dataset_merged.csv` (all features + label column, ready for fuzzy design)

---

## 11. Timeline & Phases

**Phase 2A** (now): Implement firmware, test on bench  
**Phase 2B** (next): Collect 5-10 labeled sessions (rested + fatigued)  
**Phase 2C** (design): MediaPipe processing + fuzzy design  
**Phase 3** (deploy): Real-time fuzzy inference on ESP32 (optional: ESP-WHO for on-device camera feature extraction)

---

## 12. References & Notes

- **EAR calculation:** OpenCV + MediaPipe FaceMesh, 6-point per eye
- **PERCLOS window:** 60s rolling, update every 1s (industry standard, NHTSA)
- **Blink detection:** EAR < 0.20 for 2+ consecutive frames = 1 blink
- **Fuzzy system:** Mamdani inference, designed from dataset distributions
- **On-device future:** ESP-WHO for face detection + 5-point eye landmark (alternative to PC MediaPipe)

---

**Document Status:** Ready for implementation  
**Next Step:** Begin Phase 2A firmware development
