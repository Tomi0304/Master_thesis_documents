#!/usr/bin/env python3
"""Pitch-axis angle from a BNO085 in UART-RVC mode.

Same geometry as pivot_imu.py, different link. RVC drops SHTP entirely: the
sensor pushes fixed 19-byte frames at 100 Hz and is never addressed, so none
of the I2C failure modes exist. No unknown report ids, no cached readings
served as fresh, no clock stretching, no polling that starves the sensor.

    AA AA  idx  yaw  pitch  roll  ax  ay  az  rsv rsv rsv  csum
     0  1   2   3 4   5  6   7 8  9 10 11 12 13 14  15..17   18

    angles  int16 little-endian, 0.01 deg per count
    accel   int16 little-endian, 0.0098 m/s^2 per count
    csum    sum of bytes 2..17, modulo 256

The Euler angles are unusable on this bench: with the IMU on its edge the
pitch sits at 86 deg with the arm at rest, four degrees from gimbal lock. The
angle therefore comes from the acceleration vector, which is gravity when the
sensor sits on the pivot axis, projected onto the calibrated plane exactly as
the quaternion was.

RVC carries no gyroscope, so the rate is estimated from the angle. That is
the one thing this link costs.

    from pivot_imu_rvc import PivotRVC

    imu = PivotRVC('pivot_calib.json')
    while True:
        imu.read()
        if imu.healthy:
            use(imu.angle, imu.rate)

Wiring: sensor SDA to the Pi RX (GPIO 15), P0 to 3.3 V, and the console off
the port (raspi-config, login shell No, hardware serial Yes).

    python3 pivot_imu_rvc.py calibrate
    python3 pivot_imu_rvc.py
"""

import math
import os
import struct
import sys
import time

from pivot_imu import (CalibrationError, PivotCalibration,
                       F_OK, F_NOZERO, F_STALE, F_DRIFT, F_NOCAL)

__all__ = ["PivotRVC", "RVCFrame", "read_frames"]

PORT = "/dev/serial0"
BAUD = 115200
FRAME_LEN = 19
HEADER = b"\xAA\xAA"

ANGLE_SCALE = 0.01          # deg per count
ACCEL_SCALE = 0.0098        # m/s^2 per count

DEFAULT_STALE_S = 0.2       # 20 frames at 100 Hz
DEFAULT_ZERO_N = 50
DEFAULT_ZERO_SPREAD = 1.5
DEFAULT_DRIFT_WARN = 0.02
# Time constant of the rate estimate. The raw accelerometer noise is about
# 0.02 m/s^2 per axis, i.e. 0.11 deg after projection; differentiating that at
# 100 Hz would give some 15 deg/s of noise. At 0.05 s it comes down to a few
# deg/s, while staying far faster than the 6 s pendulum period.
DEFAULT_RATE_TAU = 0.05


class RVCFrame(object):
    __slots__ = ("index", "yaw", "pitch", "roll", "accel", "t")

    def __init__(self, index, yaw, pitch, roll, accel, t):
        self.index = index
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.accel = accel
        self.t = t


def read_frames(port, buf):
    """Pull every complete frame waiting on the port.

    Returns (frames, buf). Draining matters: the sensor emits at 100 Hz and a
    control loop runs slower, so frames pile up. Taking the oldest would feed
    the loop a measurement that ages by one frame every cycle until the buffer
    overflows. The caller keeps the last.
    """
    n = port.in_waiting
    if n:
        buf += port.read(n)
    out = []
    while True:
        i = buf.find(HEADER)
        if i < 0:
            # Keep one byte: a header split across two reads would be lost.
            buf = buf[-1:]
            break
        if len(buf) - i < FRAME_LEN:
            buf = buf[i:]
            break
        raw = buf[i:i + FRAME_LEN]
        buf = buf[i + FRAME_LEN:]
        if (sum(raw[2:18]) & 0xFF) != raw[18]:
            continue
        idx = raw[2]
        y, p, r, ax, ay, az = struct.unpack("<hhhhhh", raw[3:15])
        out.append(RVCFrame(idx, y * ANGLE_SCALE, p * ANGLE_SCALE,
                            r * ANGLE_SCALE,
                            (ax * ACCEL_SCALE, ay * ACCEL_SCALE,
                             az * ACCEL_SCALE),
                            time.monotonic()))
    return out, buf


class PivotRVC(object):
    """Pitch angle and estimated rate about a calibrated pivot axis."""

    def __init__(self, calibration=None, port=PORT, baud=BAUD,
                 stale_s=DEFAULT_STALE_S, zero_n=DEFAULT_ZERO_N,
                 zero_spread_deg=DEFAULT_ZERO_SPREAD,
                 drift_warn=DEFAULT_DRIFT_WARN,
                 rate_tau=DEFAULT_RATE_TAU, verbose=True):
        if calibration is None:
            self.cal = PivotCalibration.identity_frame()
        elif isinstance(calibration, PivotCalibration):
            self.cal = calibration
            self.cal.validate()
        else:
            self.cal = PivotCalibration.load(calibration)

        self.stale_s = stale_s
        self.zero_n = zero_n
        self.zero_spread_deg = zero_spread_deg
        self.drift_warn = drift_warn
        self.rate_tau = rate_tau
        self.verbose = verbose

        self.angle = 0.0
        self.rate = 0.0
        self.resid = 0.0
        self.offset = 0.0
        self.frame = None
        self.flags = F_NOZERO | (F_NOCAL if self.cal.identity else 0)

        self._buf = b""
        self._zero_buf = []
        self._zeroed = False
        self._prev_angle = None
        self._prev_t = None
        self._last_fresh = time.monotonic()
        self._counts = {"reads": 0, "frames": 0, "bad_csum": 0, "dropped": 0}

        import serial
        self._port = serial.Serial(port, baud, timeout=0)
        self._port.reset_input_buffer()

    def _log(self, msg):
        if self.verbose:
            print("[pivot_rvc] %s" % msg)

    def close(self):
        try:
            self._port.close()
        except Exception:
            pass

    def request_zero(self):
        self._zeroed = False
        self._zero_buf = []

    @property
    def healthy(self):
        return (self.flags & F_OK) and not (self.flags & (F_NOZERO | F_STALE))

    def read(self):
        """Consume everything waiting and keep the newest frame."""
        self._counts["reads"] += 1
        frames, self._buf = read_frames(self._port, self._buf)

        if not frames:
            if time.monotonic() - self._last_fresh > self.stale_s:
                self.flags = (self.flags | F_STALE) & ~F_OK
            return False

        self._counts["frames"] += len(frames)
        self._counts["dropped"] += len(frames) - 1
        f = frames[-1]
        self.frame = f
        self._last_fresh = f.t
        self.flags &= ~F_STALE

        ax, ay, az = f.accel
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1.0:
            # A plausible reading can never be this short; treat as invalid
            # rather than normalising noise into a confident direction.
            self.flags &= ~F_OK
            return False
        v = (ax / norm, ay / norm, az / norm)

        e1, e2, n = self.cal.e1, self.cal.e2, self.cal.n
        p1 = v[0] * e1[0] + v[1] * e1[1] + v[2] * e1[2]
        p2 = v[0] * e2[0] + v[1] * e2[1] + v[2] * e2[2]
        raw = self.cal.sign * math.degrees(math.atan2(p2, p1))

        self.resid = v[0] * n[0] + v[1] * n[1] + v[2] * n[2]
        if self.cal.resid_ref is not None:
            if abs(self.resid - self.cal.resid_ref) > self.drift_warn:
                self.flags |= F_DRIFT
            else:
                self.flags &= ~F_DRIFT

        if not self._zeroed:
            self._zero_buf.append(raw)
            if len(self._zero_buf) > self.zero_n:
                self._zero_buf.pop(0)
            if len(self._zero_buf) == self.zero_n:
                spread = max(self._zero_buf) - min(self._zero_buf)
                if spread < self.zero_spread_deg:
                    self.offset = sum(self._zero_buf) / self.zero_n
                    self._zeroed = True
                    self._log("zero %+.2f deg, out-of-plane %+.4f"
                              % (self.offset, self.resid))
        self.flags = (self.flags & ~F_NOZERO) if self._zeroed \
            else (self.flags | F_NOZERO)

        angle = (raw - self.offset + 180.0) % 360.0 - 180.0

        # RVC has no gyroscope. Differentiate, then low-pass: the raw
        # difference of a noisy angle at 100 Hz is unusable as a derivative
        # term, and the pendulum is slow enough that the lag costs nothing.
        if self._prev_t is not None:
            dt = f.t - self._prev_t
            if dt > 1e-4:
                d = (angle - self._prev_angle + 180.0) % 360.0 - 180.0
                alpha = dt / (self.rate_tau + dt)
                self.rate += alpha * (d / dt - self.rate)
        self._prev_angle = angle
        self._prev_t = f.t

        self.angle = angle
        self.flags |= F_OK
        return True

    def stats(self):
        d = dict(self._counts)
        d["flags"] = self.flags
        d["resid"] = self.resid
        d["drift"] = (None if self.cal.resid_ref is None
                      else self.resid - self.cal.resid_ref)
        return d

    def describe(self):
        names = [(F_OK, "OK"), (F_NOZERO, "NOZERO"), (F_STALE, "STALE"),
                 (F_DRIFT, "DRIFT"), (F_NOCAL, "NOCAL")]
        return "|".join(n for b, n in names if self.flags & b) or "-"

    # -- calibration -----------------------------------------------------

    @staticmethod
    def calibrate(path="pivot_calib.json", sweep_s=40.0, rate_hz=50.0,
                  min_span_deg=25.0, max_residual=0.02, min_radius=0.30,
                  port=PORT):
        """Identify the pivot axis, same method as the I2C version.

        Gravity sweeps a circle in the sensor frame as the arm turns about a
        fixed axis; fitting that plane gives the axis with no assumption on
        the mounting. The gyro cross-check of the I2C version is not possible
        here, so the plane residual and the swept angle carry the whole
        verdict: a bad fit shows up as a thick plane.
        """
        import numpy as np

        imu = PivotRVC(None, port=port, verbose=True)

        def grab(n, label):
            g, t, dt = [], [], 1.0 / rate_hz
            nxt = time.perf_counter()
            deadline = nxt + 5.0 * n * dt
            while len(g) < n:
                if time.perf_counter() > deadline:
                    raise CalibrationError(
                        "only %d of %d samples: the link is not delivering"
                        % (len(g), n))
                nxt += dt
                if imu.read() and imu.frame is not None:
                    ax, ay, az = imu.frame.accel
                    nrm = math.sqrt(ax * ax + ay * ay + az * az)
                    if nrm > 1.0:
                        g.append((ax / nrm, ay / nrm, az / nrm))
                        t.append(imu.frame.t)
                sl = nxt - time.perf_counter()
                if sl > 0:
                    time.sleep(sl)
                else:
                    nxt = time.perf_counter()
            span = t[-1] - t[0] if len(t) > 1 else 0.0
            print("  %s %d samples, %.1f Hz effective"
                  % (label, n, (n - 1) / span if span > 0 else 0.0))
            return np.array(g)

        print("Pivot calibration over RVC. Motors unpowered.\n")
        input("1/3  Sweep the full travel, slowly, for %.0f s. Enter: " % sweep_s)
        G = grab(int(sweep_s * rate_hz), "sweep")

        _, S, Vt = np.linalg.svd(G - G.mean(axis=0))
        n = Vt[-1] / np.linalg.norm(Vt[-1])
        offset = float((G @ n).mean())
        residual = float((G @ n).std())
        radius = math.sqrt(max(0.0, 1.0 - offset * offset))

        e1 = G[0] - (G[0] @ n) * n
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        span = np.degrees(np.unwrap(np.arctan2(G @ e2, G @ e1)))
        swept = float(span.max() - span.min())

        print("\n  swept        %.0f deg" % swept)
        print("  S2/S1        %.4f" % (S[1] / S[0]))
        print("  out-of-plane %.4f rms" % residual)
        print("  radius       %.4f" % radius)

        bad = []
        if swept < min_span_deg:
            bad.append("swept only %.0f deg, the fit is ill-conditioned" % swept)
        if residual > max_residual:
            bad.append("out-of-plane rms %.4f: the bracket moved, or the arm "
                       "is not on a single axis" % residual)
        if radius < min_radius:
            bad.append("radius %.3f: the pivot is near vertical" % radius)
        if bad:
            raise CalibrationError("; ".join(bad))

        input("\n2/3  Let the arm rest, hands off. Enter: ")
        G_rest = grab(100, "rest ")
        g_rest = G_rest.mean(axis=0)
        e1 = g_rest - (g_rest @ n) * n
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)

        input("\n3/3  Hold the arm raised on the MAIN ROTOR side. Enter: ")
        G_up = grab(100, "up   ")
        g_up = G_up.mean(axis=0)
        theta_up = math.degrees(math.atan2(g_up @ e2, g_up @ e1))
        if abs(theta_up) < 10.0:
            raise CalibrationError("raised reading is only %.1f deg; raise the "
                                   "arm further" % theta_up)
        sign = 1.0 if theta_up > 0 else -1.0

        cal = PivotCalibration(
            n, e1, e2, sign, resid_ref=float((G_rest @ n).mean()),
            meta={"date": time.strftime("%Y-%m-%d %H:%M:%S"),
                  "link": "uart-rvc",
                  "swept_deg": round(swept, 1),
                  "residual_rms": round(residual, 5),
                  "radius": round(radius, 5),
                  "samples": int(len(G))})
        cal.validate()
        cal.save(path)
        imu.close()
        print("\nWritten to %s" % os.path.abspath(path))
        return cal


def _main():
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        out = sys.argv[2] if len(sys.argv) > 2 else "pivot_calib.json"
        PivotRVC.calibrate(out)
        return
    cal = "pivot_calib.json" if os.path.exists("pivot_calib.json") else None
    imu = PivotRVC(cal)
    print("Monitoring. Ctrl-C to stop.\n")
    try:
        while True:
            imu.read()
            print("\rangle %+8.2f  rate %+8.2f  %-24s" %
                  (imu.angle, imu.rate, imu.describe()), end="")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n\n%s" % imu.stats())
        imu.close()


if __name__ == "__main__":
    _main()