
import serial
import time
import numpy as np

PORT = "/dev/isimm_esp32"
BAUD = 115200
DURATION = 30

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(2)
ser.reset_input_buffer()

acc_data = []
gyro_data = []
temperatures = []

print("\nMPU6050 STATIC CALIBRATION")
print("==========================")
print(f"Duration: {DURATION} seconds")
print("Keep the robot COMPLETELY STILL.")
print("Do not touch the IMU or move the robot.\n")

start = time.time()

try:
    while time.time() - start < DURATION:
        line = ser.readline().decode(
            "utf-8", errors="replace"
        ).strip()

        if not line.startswith("I,"):
            continue

        parts = line.split(",")

        if len(parts) != 8:
            continue

        try:
            values = np.array(
                [float(v) for v in parts[1:]],
                dtype=float
            )
        except ValueError:
            continue

        ax, ay, az, gx, gy, gz, temp = values

        acc_data.append([ax, ay, az])
        gyro_data.append([gx, gy, gz])
        temperatures.append(temp)

        elapsed = time.time() - start
        print(
            f"\rProgress: {elapsed:5.1f}/{DURATION}s "
            f"| Samples: {len(gyro_data)}",
            end=""
        )

except KeyboardInterrupt:
    print("\nCalibration interrupted.")

finally:
    ser.close()

if len(gyro_data) < 100:
    print("\nERROR: Not enough samples.")
    raise SystemExit(1)

acc = np.array(acc_data)
gyro = np.array(gyro_data)

print("\n\n===== ACCELEROMETER =====")
print("Mean [X,Y,Z]:", np.mean(acc, axis=0))
print("Std  [X,Y,Z]:", np.std(acc, axis=0))
print("Gravity magnitude:",
      np.linalg.norm(np.mean(acc, axis=0)), "m/s²")

print("\n===== GYROSCOPE =====")
print("Bias [X,Y,Z]:", np.mean(gyro, axis=0))
print("Std  [X,Y,Z]:", np.std(gyro, axis=0))
print("Bias in deg/s:",
      np.degrees(np.mean(gyro, axis=0)))

print("\n===== TEMPERATURE =====")
print("Mean:", np.mean(temperatures), "°C")

print("\nSamples collected:", len(gyro))
print("Estimated frequency:",
      len(gyro) / DURATION, "Hz")

print("\nCalibration measurement complete.")

