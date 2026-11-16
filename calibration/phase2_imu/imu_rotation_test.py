
import serial
import time

PORT = "/dev/isimm_esp32"
BAUD = 115200

BIAS_X = -0.11921464
BIAS_Y =  0.01997005
BIAS_Z =  0.00630116

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(2)
ser.reset_input_buffer()

print("\n===== IMU ROTATION TEST =====")
print("Keep the robot flat.")
print("First observe it without moving.")
print("Then rotate it slowly COUNTERCLOCKWISE.")
print("Then rotate it slowly CLOCKWISE.")
print("Press CTRL+C to stop.\n")

try:
    while True:
        line = ser.readline().decode(
            "utf-8", errors="replace"
        ).strip()

        if not line.startswith("I,"):
            continue

        parts = line.split(",")

        if len(parts) != 8:
            continue

        try:
            values = [float(v) for v in parts[1:]]
        except ValueError:
            continue

        ax, ay, az, gx, gy, gz, temp = values

        gx_corrected = gx - BIAS_X
        gy_corrected = gy - BIAS_Y
        gz_corrected = gz - BIAS_Z

        print(
            f"GYRO corrected [rad/s] "
            f"X:{gx_corrected:+.3f} "
            f"Y:{gy_corrected:+.3f} "
            f"Z:{gz_corrected:+.3f} "
            f"| TEMP:{temp:.1f} C"
        )

except KeyboardInterrupt:
    print("\nTest stopped.")

finally:
    ser.close()
