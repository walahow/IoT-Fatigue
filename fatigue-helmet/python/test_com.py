import serial, sys

def main():
    port = 'COM4'
    print(f"Opening {port}...")
    try:
        ser = serial.Serial(port, 921600, timeout=1.0)
        # ser.dtr = False
        # ser.rts = False
    except Exception as e:
        print(f"Error opening {port}: {e}")
        return

    print("Listening for data... (Press Ctrl+C to stop)")
    try:
        while True:
            chunk = ser.read(1024)
            if chunk:
                print(f"Received {len(chunk)} bytes!")
                # print snippet of hex
                print(chunk[:16].hex())
            else:
                print("No data... (timeout)")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == '__main__':
    main()
