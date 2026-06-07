#!/usr/bin/env python3
"""
debug_recorder.py  —  Phase 2 Debug Recorder
=============================================
Receives sensor CSV data (text) AND camera JPEG frames (binary) from the
ESP32 over a single USB-UART connection at 921600 baud.

Saves everything in the same folder structure as the SD card brief:
    sessions/
        session_001/
            metadata.txt
            sensor_data.csv
            frames/
                {timestamp_ms}.jpg
                ...

Usage:
    python debug_recorder.py
    python debug_recorder.py --port COM4
    python debug_recorder.py --port COM4 --baud 921600

Binary frame protocol (ESP32 → PC):
    [0xAA 0xBB 0xCC 0xDD]  4 B  magic SOF  (all > 0x7F, never in ASCII text)
    [timestamp_ms]          4 B  little-endian uint32
    [jpeg_length]           4 B  little-endian uint32
    [JPEG data]             N B
    [0xDD 0xCC 0xBB 0xAA]  4 B  magic EOF
"""

import argparse
import os
import struct
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    print("[ERROR] pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

# ── Binary frame protocol ────────────────────────────────────────────────────
MAGIC_SOF   = bytes([0xAA, 0xBB, 0xCC, 0xDD])
MAGIC_EOF   = bytes([0xDD, 0xCC, 0xBB, 0xAA])
HEADER_LEN  = 12   # SOF(4) + timestamp(4) + length(4)
MAX_FRAME_B = 80_000  # safety cap: reject frames > 80 KB (corrupt length field)

# ── Session directory  ────────────────────────────────────────────────────────
SESSIONS_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sessions')


def next_session_dir() -> str:
    os.makedirs(SESSIONS_ROOT, exist_ok=True)
    n = 1
    while True:
        d = os.path.join(SESSIONS_ROOT, f'session_{n:03d}')
        if not os.path.exists(d):
            return d
        n += 1


def write_metadata(path: str, args, start_iso: str) -> None:
    with open(path, 'w') as f:
        f.write(f'session_start_wall={start_iso}\n')
        f.write(f'camera_fps=10\n')
        f.write(f'camera_resolution=320x240\n')
        f.write( 'sensor_sample_rate=1Hz\n')
        f.write( 'mpu_address=0x68\n')
        f.write( 'pulse_pin=1\n')
        f.write(f'baud_rate={args.baud}\n')
        f.write(f'port={args.port}\n')
        f.write( 'mode=debug_usb\n')


def process_text(raw: bytes, csv_file, stats: dict) -> None:
    """Route ASCII lines to console (monitor) and sensor CSV file."""
    try:
        text = raw.decode('ascii', errors='replace')
    except Exception:
        return
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if not line.isascii():
            continue
        if line.startswith('#'):
            print(f'[ESP32] {line}')
        else:
            # Validate CSV row format (11 columns) to reject startup/serial noise
            parts = line.split(',')
            if len(parts) == 11:
                print(f'[DATA]  {line}')
                csv_file.write(line + '\n')
                csv_file.flush()
                stats['csv_rows'] += 1


def run(args) -> None:
    session_dir = next_session_dir()
    frames_dir  = os.path.join(session_dir, 'frames')
    os.makedirs(frames_dir, exist_ok=True)

    start_iso  = datetime.now().isoformat(timespec='seconds')
    start_wall = time.time()

    write_metadata(os.path.join(session_dir, 'metadata.txt'), args, start_iso)

    print(f'\n[RECORDER] Session directory : {session_dir}')
    print(f'[RECORDER] Opening {args.port} @ {args.baud} baud ...')

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        print(f'[ERROR] Cannot open {args.port}: {e}')
        sys.exit(1)

    stats = {'csv_rows': 0, 'frames': 0, 'bad_frames': 0,
             'last_report': start_wall}

    csv_path = os.path.join(session_dir, 'sensor_data.csv')
    buf      = b''

    print(f'[RECORDER] Recording. Press Ctrl+C to stop.\n')

    with open(csv_path, 'w', newline='', encoding='utf-8') as csv_file:
        try:
            while True:
                chunk = ser.read(8192)
                if chunk:
                    buf += chunk

                # ── Demux loop ────────────────────────────────────────────
                while buf:
                    sof_idx = buf.find(MAGIC_SOF)

                    # ── No magic header yet: everything is text ──────────
                    if sof_idx == -1:
                        nl = buf.rfind(b'\n')
                        if nl >= 0:
                            process_text(buf[:nl + 1], csv_file, stats)
                            buf = buf[nl + 1:]
                        break  # wait for more data

                    # ── Text before the SOF marker ───────────────────────
                    if sof_idx > 0:
                        process_text(buf[:sof_idx], csv_file, stats)
                        buf = buf[sof_idx:]

                    # ── Try to parse the binary JPEG packet ──────────────
                    if len(buf) < HEADER_LEN:
                        break  # need more bytes

                    timestamp_ms, jpeg_len = struct.unpack_from('<II', buf, 4)

                    # Safety: reject implausibly large lengths (bit error)
                    if jpeg_len > MAX_FRAME_B:
                        print(f'[WARN] Implausible frame length {jpeg_len:,} B '
                              f'at ts={timestamp_ms} — skipping 1 byte')
                        buf = buf[1:]   # advance 1 byte and re-scan
                        stats['bad_frames'] += 1
                        continue

                    total_needed = HEADER_LEN + jpeg_len + 4  # +4 for EOF
                    if len(buf) < total_needed:
                        break  # wait for more data

                    jpeg_data  = buf[HEADER_LEN : HEADER_LEN + jpeg_len]
                    eof_marker = buf[HEADER_LEN + jpeg_len : total_needed]

                    if eof_marker == MAGIC_EOF:
                        # Valid frame — save it
                        fname = os.path.join(frames_dir, f'{timestamp_ms}.jpg')
                        with open(fname, 'wb') as f:
                            f.write(jpeg_data)
                        stats['frames'] += 1
                        print(f'[IMG]  {timestamp_ms}.jpg  '
                              f'({jpeg_len:,} B)  [total: {stats["frames"]}]')
                    else:
                        print(f'[WARN] Bad EOF at ts={timestamp_ms}, '
                              f'discarding frame')
                        stats['bad_frames'] += 1

                    buf = buf[total_needed:]

                # ── Periodic stats line ───────────────────────────────────
                now = time.time()
                if now - stats['last_report'] >= 30.0:
                    elapsed = now - start_wall
                    fps = stats['frames'] / elapsed if elapsed > 0 else 0
                    print(f'\n[STATS] {elapsed:6.0f}s  |  '
                          f'CSV rows: {stats["csv_rows"]}  |  '
                          f'Frames: {stats["frames"]}  |  '
                          f'FPS: {fps:.1f}  |  '
                          f'Bad frames: {stats["bad_frames"]}\n')
                    stats['last_report'] = now

        except KeyboardInterrupt:
            pass

    ser.close()
    elapsed = time.time() - start_wall

    # Append final stats to metadata
    with open(os.path.join(session_dir, 'metadata.txt'), 'a') as f:
        f.write(f'session_duration_s={elapsed:.1f}\n')
        f.write(f'total_csv_rows={stats["csv_rows"]}\n')
        f.write(f'total_frames={stats["frames"]}\n')
        f.write(f'bad_frames={stats["bad_frames"]}\n')

    print(f'\n[RECORDER] Session ended after {elapsed:.1f}s')
    print(f'[RECORDER] CSV rows  : {stats["csv_rows"]}')
    print(f'[RECORDER] Frames    : {stats["frames"]}')
    print(f'[RECORDER] Bad frames: {stats["bad_frames"]}')
    print(f'[RECORDER] Saved to  : {session_dir}')


def main() -> None:
    parser = argparse.ArgumentParser(
        description='ESP32 Phase 2 debug recorder — saves camera frames + sensor CSV')
    parser.add_argument('--port', default='COM4',
                        help='Serial port (default: COM4)')
    parser.add_argument('--baud', type=int, default=921600,
                        help='Baud rate (default: 921600)')
    run(parser.parse_args())


if __name__ == '__main__':
    main()
