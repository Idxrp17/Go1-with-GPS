import serial

# Check your port by writing this in terminal: /dev/tty/ACM*
port = "/dev/ttyACM0" 
baud = 9600

try:
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Reading from {port} at {baud} baud...")
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
except serial.SerialException as e:
    print(f"Error: {e}")
except KeyboardInterrupt:
    print("\nStopped.")
