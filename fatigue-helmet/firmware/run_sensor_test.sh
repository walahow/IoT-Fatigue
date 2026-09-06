#!/bin/bash
# Sensor Test Automation Script (Linux/macOS)
# Finds your ESP32-S3 port, compiles, uploads, and captures output

set -e

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║      IoT FATIGUE HELMET — SENSOR TEST AUTOMATION            ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Detect COM port
if [ -n "$1" ]; then
    COM_PORT="$1"
else
    # Try to find ESP32 automatically
    echo "Scanning for ESP32-S3 serial port..."

    # Linux
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        if [ -e /dev/ttyUSB0 ]; then
            COM_PORT="/dev/ttyUSB0"
        elif [ -e /dev/ttyACM0 ]; then
            COM_PORT="/dev/ttyACM0"
        fi
    # macOS
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        if [ -e /dev/cu.usbserial-* ]; then
            COM_PORT=$(ls /dev/cu.usbserial-* 2>/dev/null | head -1)
        elif [ -e /dev/cu.SLAB_USBtoUART ]; then
            COM_PORT="/dev/cu.SLAB_USBtoUART"
        fi
    fi
fi

if [ -z "$COM_PORT" ]; then
    echo ""
    echo "ERROR: Could not auto-detect serial port."
    echo "Usage: ./run_sensor_test.sh /dev/ttyUSB0"
    echo ""
    exit 1
fi

echo "Using serial port: $COM_PORT"
echo ""

# Step 1: Compile & Upload
echo "═══════════════════════════════════════════════════════════"
echo "Step 1/3: COMPILING SENSOR TEST FIRMWARE"
echo "═══════════════════════════════════════════════════════════"
pio run -e sensor_test -t upload --upload-port "$COM_PORT"

echo ""
echo "✓ Upload complete. Waiting 3 seconds for board to reset..."
sleep 3

# Step 2: Capture Output
echo ""
echo "═══════════════════════════════════════════════════════════"
echo "Step 2/3: RUNNING SENSOR TEST (~100 seconds)"
echo "═══════════════════════════════════════════════════════════"
echo ""

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_FILE="sensor_test_result_${TIMESTAMP}.txt"

echo "Capturing output to: $OUTPUT_FILE"
echo ""

timeout 120 pio device monitor -b 921600 -p "$COM_PORT" > "$OUTPUT_FILE" 2>&1 || true

sleep 2

# Step 3: Analyze
echo ""
echo "═══════════════════════════════════════════════════════════"
echo "Step 3/3: ANALYZING RESULTS"
echo "═══════════════════════════════════════════════════════════"
echo ""

if [ -f "$OUTPUT_FILE" ]; then
    echo "Parsing results..."
    python3 analyze_sensor_test.py "$OUTPUT_FILE"
    echo ""
    echo "Full output saved to: $OUTPUT_FILE"
    echo ""

    # Try to open with default text editor
    if command -v xdg-open &> /dev/null; then
        xdg-open "$OUTPUT_FILE"
    elif command -v open &> /dev/null; then
        open "$OUTPUT_FILE"
    fi
else
    echo "ERROR: Output file not created"
    exit 1
fi

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║                  TEST COMPLETE                             ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
