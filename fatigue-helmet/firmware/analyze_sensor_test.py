#!/usr/bin/env python3
"""
Sensor Test Result Analyzer

Parse output from sensor_test.cpp and generate a stability report.

Usage:
    # Capture serial output to file
    pio device monitor -b 921600 -p COM_PORT > test_result.txt 2>&1

    # Analyze results
    python analyze_sensor_test.py test_result.txt

    # Or pipe directly (Linux/Mac)
    pio device monitor -b 921600 | python analyze_sensor_test.py
"""

import sys
import re
from dataclasses import dataclass
from typing import Optional

@dataclass
class SensorResult:
    name: str
    status: str  # "STABLE", "WARNING", "FAILED"
    samples: int = 0
    errors: int = 0
    min_val: float = None
    max_val: float = None
    avg_val: float = None
    notes: list = None

    def __post_init__(self):
        if self.notes is None:
            self.notes = []

class SensorTestAnalyzer:
    def __init__(self):
        self.pulse = SensorResult("Pulse Sensor", "UNKNOWN")
        self.imu = SensorResult("IMU (MPU-6050)", "UNKNOWN")
        self.camera = SensorResult("Camera (OV2640)", "UNKNOWN")
        self.summary = []

    def parse(self, text: str):
        """Parse sensor test output."""
        lines = text.split('\n')

        for i, line in enumerate(lines):
            # ── Pulse Sensor Parsing ──
            if "PULSE SENSOR SUMMARY" in line:
                self._parse_pulse(lines[i:i+10])

            # ── IMU Parsing ──
            elif "IMU SUMMARY" in line:
                self._parse_imu(lines[i:i+10])

            # ── Camera Parsing ──
            elif "CAMERA SUMMARY" in line:
                self._parse_camera(lines[i:i+10])

    def _parse_pulse(self, lines):
        """Extract pulse sensor metrics."""
        for line in lines:
            if "Samples:" in line and "Errors" in line:
                # "Samples: 15000 | Errors (no contact): 120 | Error rate: 0.80%"
                m = re.search(r'Samples: (\d+).*Errors.*?(\d+).*Error rate: ([\d.]+)%', line)
                if m:
                    self.pulse.samples = int(m.group(1))
                    self.pulse.errors = int(m.group(2))
                    error_rate = float(m.group(3))

                    if error_rate < 10 and self.pulse.errors < self.pulse.samples // 10:
                        self.pulse.status = "STABLE"
                    elif error_rate < 20:
                        self.pulse.status = "WARNING"
                        self.pulse.notes.append(f"Error rate {error_rate:.2f}% (>10%) — check finger contact")
                    else:
                        self.pulse.status = "FAILED"
                        self.pulse.notes.append(f"High error rate {error_rate:.2f}%")

            elif "ADC Range:" in line:
                # "ADC Range: [1100, 3520] | Avg: 2405"
                m = re.search(r'\[(\d+), (\d+)\].*Avg: ([\d.]+)', line)
                if m:
                    self.pulse.min_val = float(m.group(1))
                    self.pulse.max_val = float(m.group(2))
                    self.pulse.avg_val = float(m.group(3))

            elif "Valid Beats:" in line:
                # "Valid Beats: 35 | BPM Range: [58.8, 65.2]"
                m = re.search(r'Valid Beats: (\d+).*\[([\d.]+), ([\d.]+)\]', line)
                if m:
                    beats = int(m.group(1))
                    if beats < 10:
                        self.pulse.status = "WARNING"
                        self.pulse.notes.append(f"Only {beats} valid beats (expect >10) — weak signal")

            elif "STABLE" in line and "PULSE" not in line:
                if "✓ STABLE" in line:
                    if self.pulse.status == "UNKNOWN":
                        self.pulse.status = "STABLE"
                elif "⚠️" in line or "WARNING" in line:
                    self.pulse.status = "WARNING"
                elif "✗" in line or "FAILED" in line:
                    self.pulse.status = "FAILED"

    def _parse_imu(self, lines):
        """Extract IMU metrics."""
        for line in lines:
            if "Valid Reads:" in line:
                # "Valid Reads: 300 | Failed: 0"
                m = re.search(r'Valid Reads: (\d+).*Failed: (\d+)', line)
                if m:
                    self.imu.samples = int(m.group(1))
                    self.imu.errors = int(m.group(2))

                    if self.imu.errors <= 5 and self.imu.samples >= 250:
                        self.imu.status = "STABLE"
                    elif self.imu.errors > 10:
                        self.imu.status = "FAILED"
                        self.imu.notes.append(f"{self.imu.errors} I2C errors — check wiring")
                    elif self.imu.samples < 250:
                        self.imu.status = "WARNING"
                        self.imu.notes.append(f"Only {self.imu.samples} samples (expect ~300) — slow I2C?")

            elif "Accel (g):" in line:
                # "Accel (g): AX[-0.031, 0.025] AY[-0.012, 0.038] AZ[0.987, 1.025]"
                # Just note range; deep analysis skipped for brevity
                pass

            elif "STABLE" in line and "IMU" not in line:
                if "✓ STABLE" in line:
                    if self.imu.status == "UNKNOWN":
                        self.imu.status = "STABLE"
                elif "⚠️" in line or "WARNING" in line:
                    self.imu.status = "WARNING"
                elif "✗" in line:
                    self.imu.status = "FAILED"

    def _parse_camera(self, lines):
        """Extract camera metrics."""
        for line in lines:
            if "Frames:" in line and "captured" in line:
                # "Frames: 600 captured | 3 dropped | Drop rate: 0.50%"
                m = re.search(r'Frames: (\d+).*dropped \| Drop rate: ([\d.]+)%', line)
                if m:
                    captured = int(m.group(1))
                    drop_rate = float(m.group(2))
                    self.camera.samples = captured

                    if drop_rate < 5:
                        self.camera.status = "STABLE"
                    elif drop_rate < 10:
                        self.camera.status = "WARNING"
                        self.camera.notes.append(f"Drop rate {drop_rate:.2f}% (5-10%) — monitor SD contention")
                    else:
                        self.camera.status = "FAILED"
                        self.camera.notes.append(f"High drop rate {drop_rate:.2f}%")

            elif "Effective FPS:" in line:
                # "Effective FPS: 20.0"
                m = re.search(r'Effective FPS: ([\d.]+)', line)
                if m:
                    fps = float(m.group(1))
                    if fps < 18 and self.camera.status == "STABLE":
                        self.camera.status = "WARNING"
                        self.camera.notes.append(f"FPS {fps:.1f} below target 20 — bus congestion?")

            elif "STABLE" in line and "CAMERA" not in line:
                if "✓ STABLE" in line:
                    if self.camera.status == "UNKNOWN":
                        self.camera.status = "STABLE"
                elif "⚠️" in line or "WARNING" in line:
                    self.camera.status = "WARNING"
                elif "✗" in line:
                    self.camera.status = "FAILED"

    def report(self):
        """Generate a summary report."""
        print("\n" + "=" * 70)
        print("SENSOR STABILITY TEST REPORT")
        print("=" * 70)

        sensors = [self.pulse, self.imu, self.camera]
        for sensor in sensors:
            icon = {"STABLE": "✓", "WARNING": "⚠️", "FAILED": "✗", "UNKNOWN": "?"}
            status_icon = icon.get(sensor.status, "?")
            print(f"\n{status_icon} {sensor.name}: {sensor.status}")

            if sensor.samples > 0:
                print(f"   Samples/Reads: {sensor.samples} | Errors: {sensor.errors}")
                error_pct = 100.0 * sensor.errors / max(sensor.samples, 1)
                print(f"   Error Rate: {error_pct:.2f}%")

            if sensor.min_val is not None and sensor.max_val is not None:
                print(f"   Range: [{sensor.min_val:.2f}, {sensor.max_val:.2f}]")

            if sensor.avg_val is not None:
                print(f"   Average: {sensor.avg_val:.2f}")

            if sensor.notes:
                for note in sensor.notes:
                    print(f"   ℹ️  {note}")

        # Summary
        print("\n" + "=" * 70)

        statuses = [self.pulse.status, self.imu.status, self.camera.status]
        if all(s == "STABLE" for s in statuses):
            print("✓ ALL SENSORS STABLE — Ready for full firmware deployment")
            return 0
        elif any(s == "FAILED" for s in statuses):
            print("✗ CRITICAL: At least one sensor FAILED — See troubleshooting guide")
            return 2
        else:
            print("⚠️  SOME WARNINGS — Review notes above, consider re-testing after adjustments")
            return 1

def main():
    analyzer = SensorTestAnalyzer()

    if len(sys.argv) > 1:
        # Read from file
        with open(sys.argv[1], 'r') as f:
            text = f.read()
    else:
        # Read from stdin
        text = sys.stdin.read()

    analyzer.parse(text)
    exit_code = analyzer.report()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
