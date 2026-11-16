
import serial
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--port", default="/dev/isimm_esp32")
args = parser.parse_args()

ser = serial.Serial(args.port, 115200, timeout=2)

time.sleep(2)
ser.reset_input_buffer()

print("ESP32 CONNECTED")
print("Reading IMU data... CTRL+C to stop\n")

try:
    while True:
        line = ser.readline().decode(
            "utf-8", errors="replace"
        ).strip()

        if not line:
            continue

        if line.startswith("I,"):
            parts = line.split(",")

            if len(parts) != 8:
                print("INVALID:", line)
                continue

            try:
                ax, ay, az, gx, gy, gz, temp = map(
                    float, parts[1:]
                )

                print(
                    f"ACC [{ax:+.3f}, {ay:+.3f}, {az:+.3f}] | "
                    f"GYRO [{gx:+.3f}, {gy:+.3f}, {gz:+.3f}] | "
                    f"TEMP {temp:.1f} C"
                )

            except ValueError:
                print("INVALID DATA:", line)
        else:
            print("STATUS:", line)

except KeyboardInterrupt:
    print("\nMonitoring stopped.")

finally:
    ser.close()
