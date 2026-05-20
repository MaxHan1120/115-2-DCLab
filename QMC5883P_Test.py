import serial
import re
import time
import math
import threading
from collections import deque
import matplotlib.pyplot as plt

# =========================
# USER SETTINGS
# =========================

PORT_1 = "COM7"
PORT_2 = "COM8"
BAUD = 115200

MAX_POINTS = 300
CALIBRATION_SECONDS = 20

# =========================
# SERIAL LINE PATTERN
# =========================

pattern = re.compile(
    r"Raw X:(-?\d+)\s*Y:(-?\d+)\s*Z:(-?\d+)\s*"
    r"\|\s*Gauss X:(-?\d+\.\d+)\s*Y:(-?\d+\.\d+)\s*Z:(-?\d+\.\d+)\s*"
    r"\|\s*\|B\|:(-?\d+\.\d+)"
)

# =========================
# GLOBAL DATA
# =========================

data_lock = threading.Lock()

sensor1_data = deque(maxlen=MAX_POINTS)
sensor2_data = deque(maxlen=MAX_POINTS)

sensor1_calib_samples = []
sensor2_calib_samples = []

sensor1_calib = None
sensor2_calib = None

start_time = time.time()

# =========================
# CALIBRATION FUNCTION
# =========================

def calculate_calibration(samples):
    xs = [s["gx"] for s in samples]
    ys = [s["gy"] for s in samples]
    zs = [s["gz"] for s in samples]

    offset_x = (max(xs) + min(xs)) / 2.0
    offset_y = (max(ys) + min(ys)) / 2.0
    offset_z = (max(zs) + min(zs)) / 2.0

    scale_x = (max(xs) - min(xs)) / 2.0
    scale_y = (max(ys) - min(ys)) / 2.0
    scale_z = (max(zs) - min(zs)) / 2.0

    avg_scale = (scale_x + scale_y + scale_z) / 3.0

    return {
        "offset_x": offset_x,
        "offset_y": offset_y,
        "offset_z": offset_z,
        "scale_x": avg_scale / scale_x if scale_x != 0 else 1,
        "scale_y": avg_scale / scale_y if scale_y != 0 else 1,
        "scale_z": avg_scale / scale_z if scale_z != 0 else 1,
    }

def apply_calibration(gx, gy, gz, calib):
    cx = (gx - calib["offset_x"]) * calib["scale_x"]
    cy = (gy - calib["offset_y"]) * calib["scale_y"]
    cz = (gz - calib["offset_z"]) * calib["scale_z"]

    b_cal = math.sqrt(cx * cx + cy * cy + cz * cz)

    return cx, cy, cz, b_cal

# =========================
# READER THREAD
# =========================

def reader_thread(port, storage, calib_samples, name):
    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(2)

    print(f"{name} connected on {port}")

    while True:
        line = ser.readline().decode(errors="ignore").strip()

        match = pattern.search(line)
        if not match:
            continue

        raw_x = int(match.group(1))
        raw_y = int(match.group(2))
        raw_z = int(match.group(3))

        gx = float(match.group(4))
        gy = float(match.group(5))
        gz = float(match.group(6))

        b_raw = float(match.group(7))
        t = time.time() - start_time

        sample = {
            "t": t,
            "raw_x": raw_x,
            "raw_y": raw_y,
            "raw_z": raw_z,
            "gx": gx,
            "gy": gy,
            "gz": gz,
            "b_raw": b_raw,
            "cx": None,
            "cy": None,
            "cz": None,
            "b_cal": None,
        }

        with data_lock:
            if t < CALIBRATION_SECONDS:
                calib_samples.append(sample)

            storage.append(sample)

# =========================
# START THREADS
# =========================

threads = [
    threading.Thread(
        target=reader_thread,
        args=(PORT_1, sensor1_data, sensor1_calib_samples, "Sensor1"),
        daemon=True
    ),
    threading.Thread(
        target=reader_thread,
        args=(PORT_2, sensor2_data, sensor2_calib_samples, "Sensor2"),
        daemon=True
    ),
]

for th in threads:
    th.start()

# =========================
# MAIN PROGRAM
# =========================

plt.ion()
fig, ax = plt.subplots()

print("Reading two sensors...")
print("Calibration started.")
print("Rotate each sensor slowly in all directions.")
print(f"Calibration time: {CALIBRATION_SECONDS} seconds")

calibration_done = False

while True:
    now = time.time() - start_time

    with data_lock:
        s1 = list(sensor1_data)
        s2 = list(sensor2_data)

        if not calibration_done and now >= CALIBRATION_SECONDS:
            if len(sensor1_calib_samples) > 10:
                sensor1_calib = calculate_calibration(sensor1_calib_samples)
                print("Sensor1 calibration:")
                print(sensor1_calib)

            if len(sensor2_calib_samples) > 10:
                sensor2_calib = calculate_calibration(sensor2_calib_samples)
                print("Sensor2 calibration:")
                print(sensor2_calib)

            calibration_done = True
            print("Calibration finished.")

        if calibration_done:
            for d in sensor1_data:
                if d["b_cal"] is None and sensor1_calib is not None:
                    d["cx"], d["cy"], d["cz"], d["b_cal"] = apply_calibration(
                        d["gx"], d["gy"], d["gz"], sensor1_calib
                    )

            for d in sensor2_data:
                if d["b_cal"] is None and sensor2_calib is not None:
                    d["cx"], d["cy"], d["cz"], d["b_cal"] = apply_calibration(
                        d["gx"], d["gy"], d["gz"], sensor2_calib
                    )

        s1 = list(sensor1_data)
        s2 = list(sensor2_data)

    if s1:
        d1 = s1[-1]
        if d1["b_cal"] is not None:
            print(
                f"Sensor1 Raw |B|={d1['b_raw']:.4f} "
                f"Cal |B|={d1['b_cal']:.4f}"
            )

    if s2:
        d2 = s2[-1]
        if d2["b_cal"] is not None:
            print(
                f"Sensor2 Raw |B|={d2['b_raw']:.4f} "
                f"Cal |B|={d2['b_cal']:.4f}"
            )

    ax.clear()

    if s1:
        t1 = [d["t"] for d in s1]
        b1_raw = [d["b_raw"] for d in s1]
        ax.plot(t1, b1_raw, label="Sensor1 Raw |B|")

        if calibration_done:
            b1_cal = [d["b_cal"] for d in s1 if d["b_cal"] is not None]
            t1_cal = [d["t"] for d in s1 if d["b_cal"] is not None]
            ax.plot(t1_cal, b1_cal, label="Sensor1 Calibrated |B|")

    if s2:
        t2 = [d["t"] for d in s2]
        b2_raw = [d["b_raw"] for d in s2]
        ax.plot(t2, b2_raw, label="Sensor2 Raw |B|")

        if calibration_done:
            b2_cal = [d["b_cal"] for d in s2 if d["b_cal"] is not None]
            t2_cal = [d["t"] for d in s2 if d["b_cal"] is not None]
            ax.plot(t2_cal, b2_cal, label="Sensor2 Calibrated |B|")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("|B| (Gauss)")
    ax.set_title("Two QMC5883P Sensors: Raw vs Calibrated Magnetic Field")
    ax.grid(True)
    ax.legend()

    plt.pause(0.05)
