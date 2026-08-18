#!/usr/bin/env python3
"""Identify the pitch pivot axis in the BNO085 frame.

Run on the Pi with the trms-io service stopped:
    sudo systemctl stop trms-io
    python3 calib_pivot.py

The arm rotates about a fixed axis, so in the sensor frame the gravity vector
sweeps a circle whose normal is that axis. Fitting the plane gives the pivot
axis without any assumption on how the IMU is mounted, which removes the Euler
projection error and its saturation.

Prints the constants to paste into trms_io_daemon.py and writes the raw sweep
to calib_pivot.csv for traceability.
"""

import csv
import math
import sys
import time

import numpy as np

IMU_REPORT_INTERVAL_US = 10000
SAMPLE_HZ = 100.0
SWEEP_S = 25.0
STILL_N = 100

MIN_SPAN_DEG = 25.0      # refuse a sweep too short to condition the fit
MAX_RESIDUAL = 0.02      # out-of-plane spread above this means the mount moved
MIN_RADIUS = 0.30        # circle radius; collapses if the pivot were vertical


def patch_bno08x_batch_fault():
    """Step over SHTP report 0x7B, absent from the Adafruit lookup table."""
    try:
        import adafruit_bno08x as bno
        reports = bno._AVAIL_SENSOR_REPORTS
    except Exception as exc:
        print("BNO08x patch skipped: %s" % exc)
        return False
    if 0x7B not in reports:
        reports[0x7B] = (1, 0, 5)
    return True


def connect():
    import board
    import busio
    from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
    from adafruit_bno08x.i2c import BNO08X_I2C

    patch_bno08x_batch_fault()
    dev = BNO08X_I2C(busio.I2C(board.SCL, board.SDA))
    dev.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US)
    dev.enable_feature(BNO_REPORT_GYROSCOPE, IMU_REPORT_INTERVAL_US)
    time.sleep(0.5)
    return dev


def gravity_in_sensor(qx, qy, qz, qw):
    """World vertical expressed in the sensor frame: third row of R(q)."""
    return np.array([2.0 * (qx * qz - qw * qy),
                     2.0 * (qy * qz + qw * qx),
                     1.0 - 2.0 * (qx * qx + qy * qy)])


def grab(dev, n, label):
    """Collect n valid gravity samples, discarding null quaternions."""
    out, gyro, dt = [], [], 1.0 / SAMPLE_HZ
    t_next = time.perf_counter()
    while len(out) < n:
        t_next += dt
        try:
            q = dev.game_quaternion
            w = dev.gyro
        except Exception:
            q = w = None
        if q is not None and w is not None:
            qx, qy, qz, qw = q
            if qx * qx + qy * qy + qz * qz + qw * qw > 0.5:
                out.append(gravity_in_sensor(qx, qy, qz, qw))
                gyro.append(np.array(w))
        if len(out) % 100 == 0 and out:
            sys.stdout.write("\r  %s %d/%d" % (label, len(out), n))
            sys.stdout.flush()
        sleep = t_next - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            t_next = time.perf_counter()
    print("\r  %s %d/%d  done" % (label, n, n))
    return np.array(out), np.array(gyro)


def main():
    print("Pivot axis calibration\n")
    print("Stop the service first: sudo systemctl stop trms-io")
    print("Motors must be unpowered. The arm is moved by hand only.\n")

    dev = connect()

    input("1/3  Sweep: press Enter, then move the arm slowly over its full "
          "travel, several times, for %.0f s." % SWEEP_S)
    G, W = grab(dev, int(SWEEP_S * SAMPLE_HZ), "sweep")

    # Plane fit. The smallest right-singular vector of the centred samples is
    # the plane normal, i.e. the pivot axis in the sensor frame.
    centre = G.mean(axis=0)
    _, S, Vt = np.linalg.svd(G - centre)
    n = Vt[-1] / np.linalg.norm(Vt[-1])

    offset = float((G @ n).mean())
    residual = float((G @ n).std())
    radius = math.sqrt(max(0.0, 1.0 - offset * offset))

    e1 = G[0] - (G[0] @ n) * n
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    # Unwrap before measuring the span: max - min on a raw atan2 caps at 360 by
    # construction and hides a sweep that crossed the cut several times.
    span = np.degrees(np.unwrap(np.arctan2(G @ e2, G @ e1)))
    swept = float(span.max() - span.min())

    print("\n  singular values   %s" % np.round(S / S[0], 4))
    print("  swept angle       %.1f deg" % swept)
    print("  out-of-plane rms  %.4f" % residual)
    print("  circle radius     %.4f" % radius)
    print("  pivot tilt        %.1f deg from horizontal"
          % math.degrees(math.asin(min(1.0, abs(offset)))))

    bad = []
    if swept < MIN_SPAN_DEG:
        bad.append("sweep too short (%.1f < %.1f deg): the fit is ill-conditioned"
                   % (swept, MIN_SPAN_DEG))
    if residual > MAX_RESIDUAL:
        bad.append("out-of-plane rms %.4f > %.4f: the IMU or its bracket moved "
                   "during the sweep, or the arm is not on a single axis"
                   % (residual, MAX_RESIDUAL))
    if radius < MIN_RADIUS:
        bad.append("circle radius %.3f < %.3f: the pivot axis is near vertical "
                   "and gravity cannot resolve the angle" % (radius, MIN_RADIUS))
    if bad:
        print("\nREJECTED")
        for b in bad:
            print("  - %s" % b)
        return 1

    input("\n2/3  Rest: let the arm settle, hands off, then press Enter.")
    G_rest, _ = grab(dev, STILL_N, "rest ")
    g_rest = G_rest.mean(axis=0)

    e1 = g_rest - (g_rest @ n) * n
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)

    input("\n3/3  Sign: hold the arm clearly raised on the MAIN ROTOR side, "
          "then press Enter.")
    G_up, _ = grab(dev, STILL_N, "up   ")
    g_up = G_up.mean(axis=0)
    theta_up = math.degrees(math.atan2(g_up @ e2, g_up @ e1))

    if abs(theta_up) < 10.0:
        print("\nREJECTED\n  - raised reading is only %.1f deg: raise the arm "
              "further and rerun" % theta_up)
        return 1
    sign = 1.0 if theta_up > 0 else -1.0

    # Cross-check: gyro projected on the fitted axis against the derivative of
    # the reconstructed angle over the sweep. Agreement validates the axis.
    th = sign * np.degrees(np.arctan2(G @ e2, G @ e1))
    th = np.unwrap(np.radians(th))
    rate_geom = np.degrees(np.gradient(th, 1.0 / SAMPLE_HZ))
    rate_gyro = sign * np.degrees(W @ n)
    k = np.polyfit(rate_gyro, rate_geom, 1)[0]
    # Compare magnitudes: the SVD returns the normal up to sign, so a negative
    # slope only reflects which way n came out and is not a fault.
    print("\n  gyro / geometry rate slope  %+.4f  (magnitude 1.000 expected)" % k)
    if abs(abs(k) - 1.0) > 0.15:
        print("  WARNING: axis fit and gyro disagree by more than 15 %")

    with open("calib_pivot.csv", "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(["gx", "gy", "gz", "wx", "wy", "wz"])
        for g, w in zip(G, W):
            wr.writerow(["%.6f" % v for v in list(g) + list(w)])

    print("\nPaste into trms_io_daemon.py:\n")
    print("PIVOT_N    = (%+.6f, %+.6f, %+.6f)" % tuple(n))
    print("PIVOT_E1   = (%+.6f, %+.6f, %+.6f)" % tuple(e1))
    print("PIVOT_E2   = (%+.6f, %+.6f, %+.6f)" % tuple(e2))
    print("PIVOT_SIGN = %+.1f" % sign)
    print("\nRaw sweep written to calib_pivot.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())