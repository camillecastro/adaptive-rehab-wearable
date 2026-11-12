"""Record the Nucleo's MPU-6050 stream (sketch MODE 0 = binary, 500 Hz) to a CSV file, then plot it.

    pip install pyserial matplotlib
    python log_imu.py COM3 20          port, seconds   (Mac ports look like /dev/cu.usbmodemXXXX)

Close the Arduino Serial Monitor / Plotter first: only one program can use the port.
"""
import csv
import os
import struct
import sys
import time

import serial
import matplotlib.pyplot as plt

ACC_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0
PKT = 19


def parse(buf):
    """Pull complete, checksum-valid packets out of buf. Returns (rows, leftover bytes)."""
    rows, i = [], 0
    while True:
        j = buf.find(b"\xA5\x5A", i)
        if j < 0:
            return rows, buf[-1:]          # keep a possible half header
        if j + PKT > len(buf):
            return rows, buf[j:]
        body = buf[j + 2:j + 18]
        cs = 0
        for b in body:
            cs ^= b
        if cs == buf[j + 18]:
            t, ax, ay, az, gx, gy, gz = struct.unpack("<I6h", body)
            rows.append([t, ax / ACC_LSB_PER_G, ay / ACC_LSB_PER_G, az / ACC_LSB_PER_G,
                         gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS])
            i = j + PKT
        else:
            i = j + 1


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 20
    here = os.path.dirname(os.path.abspath(__file__))          # recordings go next to this script
    out = sys.argv[3] if len(sys.argv) > 3 else time.strftime("imu_%Y%m%d_%H%M%S.csv")
    if not os.path.isabs(out):
        out = os.path.join(here, out)
    baud = int(sys.argv[4]) if len(sys.argv) > 4 else 115200

    rows, buf = [], b""
    with serial.Serial(port, baud, timeout=0.2) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()
        print(f"Recording {seconds:.0f} s from {port} ...")
        t_end = time.time() + seconds
        while time.time() < t_end:
            buf += ser.read(4096)
            new, buf = parse(buf)
            rows += new

    if not rows:
        print("No data received. Check: sketch uploaded with MODE 0, right COM port, Serial Monitor closed.")
        return

    # unwrap the microsecond counter (it rolls over every ~71 minutes)
    t0, prev, off, ts = rows[0][0], rows[0][0], 0, []
    for r in rows:
        if r[0] < prev:
            off += 2 ** 32
        prev = r[0]
        ts.append((r[0] + off - t0) / 1e6)

    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"])
        for t, r in zip(ts, rows):
            w.writerow([f"{t:.6f}"] + [f"{x:.5f}" for x in r[1:]])

    dur = ts[-1] if ts[-1] > 0 else 1
    gaps = sum(1 for a, b in zip(ts, ts[1:]) if b - a > 0.003)
    print(f"Saved {len(rows)} samples (~{len(rows) / dur:.0f} Hz, {gaps} gaps) to:\n  {out}")

    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(11, 6))
    for k, lab in ((1, "ax"), (2, "ay"), (3, "az")):
        a1.plot(ts, [r[k] for r in rows], lw=0.8, label=lab)
    for k, lab in ((4, "gx"), (5, "gy"), (6, "gz")):
        a2.plot(ts, [r[k] for r in rows], lw=0.8, label=lab)
    a1.set_ylabel("acceleration (g)")
    a2.set_ylabel("rotation (deg/s)")
    a2.set_xlabel("time (s)")
    a1.legend(loc="upper right")
    a2.legend(loc="upper right")
    a1.set_title(os.path.basename(out))
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
