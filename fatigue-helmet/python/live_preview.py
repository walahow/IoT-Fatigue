#!/usr/bin/env python3
"""
live_preview.py
================
Reads the live binary UART stream from the ESP32 (in debug mode)
and displays a live video feed using OpenCV. It also runs the 
eye-tracking Haar cascade so you can align the camera perfectly!

Usage:
    python live_preview.py --port COM4
"""

import argparse
import struct
import sys
import cv2
import numpy as np

try:
    import serial
except ImportError:
    print("[ERROR] pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

MAGIC_SOF   = bytes([0xAA, 0xBB, 0xCC, 0xDD])
MAGIC_EOF   = bytes([0xDD, 0xCC, 0xBB, 0xAA])
HEADER_LEN  = 12
MAX_FRAME_B = 500_000   # Increased to 500 KB to support VGA resolution!

def main():
    parser = argparse.ArgumentParser(description='Live Camera Placement Preview')
    parser.add_argument('--port', default='COM4', help='Serial port (default: COM4)')
    parser.add_argument('--baud', type=int, default=921600, help='Baud rate (default: 921600)')
    args = parser.parse_args()

    # Load the Haar Cascade for live tracking preview
    cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_eye.xml")

    print(f"Opening {args.port} @ {args.baud} baud...")
    try:
        ser = serial.Serial()
        ser.port = args.port
        ser.baudrate = args.baud
        ser.dtr = False
        ser.rts = False
        ser.timeout = 0.05
        ser.open()
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {args.port}: {e}")
        sys.exit(1)

    # Show a loading screen immediately so the user knows it's running
    loading_img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(loading_img, "Waiting for camera feed...", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.imshow("Helmet Camera Setup", loading_img)
    cv2.waitKey(1)
    
    buf = b''
    print("Waiting for camera stream... (Make sure ESP32 is flashed with [env:esp32s3cam])")
    
    try:
        while True:
            chunk = ser.read(8192)
            if chunk:
                buf += chunk

            while buf:
                sof_idx = buf.find(MAGIC_SOF)

                # Print text/noise before SOF to console
                if sof_idx > 0:
                    print(buf[:sof_idx].decode('ascii', errors='ignore'), end='', flush=True)
                    buf = buf[sof_idx:]
                elif sof_idx == -1:
                    nl = buf.rfind(b'\n')
                    if nl >= 0:
                        print(buf[:nl + 1].decode('ascii', errors='ignore'), end='', flush=True)
                        buf = buf[nl + 1:]
                    break

                if len(buf) < HEADER_LEN:
                    break

                timestamp_ms, jpeg_len = struct.unpack_from('<II', buf, 4)

                if jpeg_len > MAX_FRAME_B:
                    print(f"[WARN] Frame too large ({jpeg_len} B) - dropping 1 byte to resync")
                    buf = buf[1:]
                    continue

                total_needed = HEADER_LEN + jpeg_len + 4
                if len(buf) < total_needed:
                    break

                jpeg_data = buf[HEADER_LEN : HEADER_LEN + jpeg_len]
                eof_marker = buf[HEADER_LEN + jpeg_len : total_needed]

                if eof_marker == MAGIC_EOF:
                    # Decode the JPEG
                    np_arr = np.frombuffer(jpeg_data, np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    
                    if img is not None:
                        # Rotate 180 degrees (since camera is mounted upside down)
                        img = cv2.rotate(img, cv2.ROTATE_180)
                        
                        # Run quick eye detection for placement help
                        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                        eyes = cascade.detectMultiScale(gray, scaleFactor=1.05, minNeighbors=3, minSize=(20,10))
                        
                        for (x, y, w, h) in eyes:
                            # Draw a green box around the detected eye
                            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 100), 2)
                            cv2.putText(img, "EYE DETECTED", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 2)
                        
                        cv2.putText(img, "Live Placement Preview", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                        
                        cv2.imshow("Helmet Camera Setup", img)
                        
                buf = buf[total_needed:]

            # Keep the OpenCV window responsive even if no frames arrive
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Exiting...")
                ser.close()
                cv2.destroyAllWindows()
                return

    except KeyboardInterrupt:
        pass

    ser.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
