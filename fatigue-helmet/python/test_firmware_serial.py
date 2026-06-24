import serial
import time
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="Inject BLINK data into ESP32 to test FIS edge cases.")
    parser.add_argument("--port", type=str, required=True, help="Serial port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--blink", type=float, default=3.0, help="Blink rate to inject (default 3.0 blinks/min for Critical testing)")
    
    args = parser.parse_args()

    print(f"[*] Opening serial port {args.port} at {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print(f"[!] Failed to open port: {e}")
        sys.exit(1)

    print("\n" + "="*50)
    print(f"[*] INJECTING BLINK RATE: {args.blink} blinks/min")
    print("[*] INSTRUCTIONS: To test Critical Fatigue (Rule 3 - Head Drop):")
    print("    1. Keep the board steady (Gyro Variance will be low).")
    print(f"    2. This script is sending BLINK:{args.blink} to simulate eyes closing.")
    print("    3. Physically tilt the board forward by > 25 degrees.")
    print("    4. Watch the serial output below and listen for the continuous BUZZER alarm.")
    print("="*50 + "\n")

    # Clear buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    last_send_time = 0

    try:
        while True:
            now = time.time()
            
            # Send the BLINK command at 1 Hz (matching the Python pipeline rate)
            if now - last_send_time >= 1.0:
                payload = f"BLINK:{args.blink}\n"
                ser.write(payload.encode('ascii'))
                last_send_time = now
            
            # Read and display incoming serial from ESP32
            if ser.in_waiting > 0:
                line = ser.readline().decode('ascii', errors='replace').strip()
                if line:
                    # Optional: highlight the RISK and ALERT line for easier reading
                    if "RISK:" in line and "ALERT:" in line:
                        print(f"    ---> {line}")
                    else:
                        print(line)
                        
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n[*] Exiting script...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
