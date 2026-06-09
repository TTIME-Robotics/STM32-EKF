import numpy as np
import serial
import time
import csv

ser = serial.Serial(
    "COM6",
    baudrate=115200
)


def collect_measurements(samples):
    data = []

    def safe_parse(line):
        parts = line.strip().split(",")

        # Must have exactly ax,ay,az,gx,gy,gz
        if len(parts) != 6:
            return None

        try:
            return [float(p) for p in parts]
        except ValueError:
            return None

    collected = 0

    while collected < samples:
        try:
            line = ser.readline().decode(errors="ignore").strip()

            row = safe_parse(line)

            if row is None:
                continue

            data.append(row)
            collected += 1

        except Exception:
            # ignore UART glitches
            continue

    return np.array(data, dtype=float)

def main():
    all_data = []
    while True:
        user_inp = input("enter to record new samples, X to exit    ")
        if user_inp == "":
            print("Collecting")
            samples = collect_measurements(100)
            all_data.extend(samples)
        elif user_inp == "X" or user_inp == "x":
            break

    print(all_data)
    t = time.time()
    np.savetxt(f"Samples {t}.csv", all_data, delimiter=",")


main()
