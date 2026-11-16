import serial
import time

PORT = "/dev/isimm_esp32"
BAUD = 115200

print(f"Opening {PORT} at {BAUD} baud...")

ser = serial.Serial(PORT, BAUD, timeout=1)

time.sleep(2)
ser.reset_input_buffer()

print("Listening for raw serial data... CTRL+C to stop")

try:
    while True:
        data = ser.readline()

        if data:
            print(repr(data))

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    ser.close()
