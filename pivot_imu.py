#!/usr/bin/env python3
"""Pitch-axis angle measurement from a BNO085, for propeller-driven benches.

Nine defects were found the hard way on this platform. Each one produced a
plausible-looking angle, which is what made them expensive: the loop closed on
a wrong number rather than on no number at all. They are all handled here.

  1. Unknown SHTP report ids (0x7B, 0x88, others) are absent from the Adafruit
     lookup table. The library raises and never resynchronises, so one
     malformed batch kills the stream. Open since 2020, issues #9, #16, #41.
  2. The BCM2711 hardware I2C controller aborts when the BNO085 stretches the
     clock (OSError errno 5) at the default 100 kHz. Counter-intuitively the
     fix is to speed the bus up, not slow it down: transfers are then four
     times shorter and the sensor has far less occasion to stretch. Measured
     on this bench at 400 kHz: zero exceptions, zero corrupted frames.
  3. The library caches the last report, so a dead stream reads as a valid,
     perfectly constant angle. Nothing raises, nothing returns None.
  4. Euler pitch assumes rotation about the sensor y axis. Any other mounting
     gives a nonlinear, saturating error that no scale factor corrects.
  5. asin is bounded to +-90 deg and folds past the vertical, flipping the
     sign of the loop while the arm is still rising.
  6. Subtracting the zero offset pushes the result outside +-180, so one
     physical position can read 360 deg apart.
  7. The gyro's y component is not the rate about the pivot either, and it
     feeds the derivative term of the controller.
  8. A freshly reset BNO085 returns a null quaternion that reads as 0 deg.
  9. The mounting bracket can move, silently invalidating the calibration.

The angle comes from the gravity vector projected onto the calibrated plane of
the pivot, not from Euler angles. The arm turns about a fixed axis, so in the
sensor frame the world vertical sweeps a circle whose normal is that axis.
Fitting that circle once gives an angle that is linear over the full +-180 deg
and independent of how the sensor is mounted.

Runtime needs no numpy. Calibration does, and imports it locally.

    from pivot_imu import PivotIMU

    imu = PivotIMU('pivot_calib.json')
    while True:
        imu.read()
        if imu.healthy:
            use(imu.angle, imu.rate)

Calibration, once, with the motors unpowered:

    python3 pivot_imu.py calibrate

Link setup that this was measured on, in /boot/firmware/config.txt:

    dtparam=i2c_arm=on
    dtparam=i2c_arm_baudrate=400000

A bit-banged bus (dtoverlay=i2c-gpio) also works and tolerates clock
stretching by construction, but caps the throughput: at ~25 kHz it delivered
10 fresh samples per second against 32 on the hardware bus.

Run `python3 pivot_imu.py bench` after any wiring or config change; it reports
the fresh-sample rate and the fault counts, which is the only way to tell a
fast-but-corrupt link from a slow-but-clean one.
"""

import json
import math
import os
import threading
import time

__all__ = ["PivotIMU", "PivotCalibration", "CalibrationError",
           "batch_fault_stats", "bench"]

# Status flags, exposed as PivotIMU.flags.
F_OK = 0x01            # a fresh reading is available
F_NOZERO = 0x02        # zero not yet acquired; the angle is not absolute
F_STALE = 0x04         # stream stopped, cached values were being served
F_RESETTING = 0x08     # a reconnection is in progress
F_DRIFT = 0x10         # out-of-plane residual has moved; calibration stale
F_NOCAL = 0x20         # running on the identity calibration

# Measured on this bench, hardware I2C at 400 kHz, quaternion + gyro:
#   40000 us (25 Hz)  -> 13 fresh/s
#   20000 us (50 Hz)  -> 26 fresh/s
#   10000 us (100 Hz) -> 32 fresh/s   <- best
#    5000 us (200 Hz) -> 20 fresh/s, then 63 exceptions when polled at 100 Hz
#    2500 us (400 Hz) -> sensor lost within seconds
# Past 200 Hz the sensor cannot keep up with its own reporting and resets, so
# 100 Hz is the ceiling here. Poll at about 50 Hz: polling harder collects no
# more, it consumes the sensor's own processing budget and starves it.
DEFAULT_REPORT_US = 20000
# Judged on elapsed time, not on a count of identical reads: polling faster
# than the sensor reports gives thousands of legitimate repeats between two
# fresh samples, so any count-based threshold fires constantly at high poll
# rates and never at low ones. Time is independent of how often we ask.
#
# Judged on the quaternion AND the gyro together. The Game Rotation Vector is
# quantised to about 0.007 deg and heavily filtered, so a motionless arm
# repeats the same quaternion for seconds; the raw gyro never does, it dithers
# by a degree or two per second even at rest.
DEFAULT_STALE_S = 0.5
DEFAULT_ZERO_N = 40
DEFAULT_ZERO_SPREAD = 1.5
DEFAULT_DRIFT_WARN = 0.02
RESET_COOLDOWN_S = 3.0

# Reconnecting soft-resets the sensor, which then emits an unsolicited reset
# packet that trips the next exception: resetting on every fault builds a loop
# that never converges. Tolerate a burst first; a genuinely dead sensor still
# fails EXC_TOLERANCE times in a row within a fraction of a second.
EXC_TOLERANCE = 8


class CalibrationError(Exception):
    pass


class PivotCalibration:
    """Orthonormal frame of the pivot's plane of rotation, in sensor axes.

    n is the pivot axis, e1/e2 an orthonormal basis of the plane it is normal
    to, and sign orients the result so that raising the main-rotor side reads
    positive. resid_ref stores the out-of-plane component at calibration time,
    which later serves as a health reference.
    """

    def __init__(self, n=(0.0, 1.0, 0.0), e1=(0.0, 0.0, 1.0),
                 e2=(1.0, 0.0, 0.0), sign=1.0, resid_ref=None, meta=None):
        self.n = tuple(float(x) for x in n)
        self.e1 = tuple(float(x) for x in e1)
        self.e2 = tuple(float(x) for x in e2)
        self.sign = float(sign)
        self.resid_ref = resid_ref
        self.meta = meta or {}
        self.identity = False

    @classmethod
    def identity_frame(cls):
        c = cls()
        c.identity = True
        return c

    @classmethod
    def load(cls, path):
        with open(path) as fh:
            d = json.load(fh)
        c = cls(d["n"], d["e1"], d["e2"], d["sign"],
                d.get("resid_ref"), d.get("meta"))
        c.validate()
        return c

    def save(self, path):
        with open(path, "w") as fh:
            json.dump({"n": list(self.n), "e1": list(self.e1),
                       "e2": list(self.e2), "sign": self.sign,
                       "resid_ref": self.resid_ref, "meta": self.meta},
                      fh, indent=2)

    def validate(self, tol=1e-3):
        """Reject a frame that is not orthonormal and right-handed.

        A calibration edited by hand, or copied with one component missing,
        otherwise produces a smoothly wrong angle rather than an error.
        """
        def dot(a, b):
            return sum(x * y for x, y in zip(a, b))

        for name, v in (("n", self.n), ("e1", self.e1), ("e2", self.e2)):
            if abs(math.sqrt(dot(v, v)) - 1.0) > tol:
                raise CalibrationError("%s is not a unit vector" % name)
        for a, b, name in ((self.n, self.e1, "n.e1"), (self.n, self.e2, "n.e2"),
                           (self.e1, self.e2, "e1.e2")):
            if abs(dot(a, b)) > tol:
                raise CalibrationError("%s = %.2e, axes are not orthogonal"
                                       % (name, dot(a, b)))
        cross = (self.e1[1] * self.e2[2] - self.e1[2] * self.e2[1],
                 self.e1[2] * self.e2[0] - self.e1[0] * self.e2[2],
                 self.e1[0] * self.e2[1] - self.e1[1] * self.e2[0])
        if dot(cross, self.n) < 0.9:
            raise CalibrationError("e1 x e2 does not point along n; the frame "
                                   "is left-handed and the sign will be wrong")
        if abs(self.sign) < 0.5:
            raise CalibrationError("sign must be +1 or -1")
        return True


_BATCH_FAULT_COUNT = 0
_BATCH_FAULT_IDS = set()
_BATCH_ORIGINAL = None


def patch_bno08x_batch_fault():
    """Stop one unknown SHTP report from killing the whole stream.

    _separate_batch() looks each report id up in _AVAIL_SENSOR_REPORTS and
    raises KeyError on anything absent. _handle_packet() re-raises, the
    exception reaches the caller, and the library never resynchronises: one
    malformed batch ends the session.

    Registering 0x7B by hand is not enough. Adafruit issue #9 (Dec 2020) and
    #16 (Feb 2021) report 0x7B on a Pi 4, and the forums show KeyError 136
    (0x88) from the same line, so the set of offending ids is open-ended.
    Issue #41, "make library more robust", is still open.

    Wrapping the splitter instead abandons only the offending batch and keeps
    every report decoded before it, whatever the id. Applied at runtime, so a
    plain pip install stays reproducible and the fix survives an upgrade.
    """
    try:
        import adafruit_bno08x as bno
    except Exception as exc:
        return False, str(exc)

    global _BATCH_ORIGINAL
    if _BATCH_ORIGINAL is not None:
        return True, None
    _BATCH_ORIGINAL = bno._separate_batch
    original = _BATCH_ORIGINAL

    def guarded(packet, slices):
        try:
            original(packet, slices)
        except KeyError as exc:
            # slices is filled in place, so reports decoded before the
            # unknown id are kept; only the tail of this batch is lost.
            global _BATCH_FAULT_COUNT
            _BATCH_FAULT_COUNT += 1
            try:
                _BATCH_FAULT_IDS.add(int(exc.args[0]))
            except Exception:
                pass

    bno._separate_batch = guarded
    return True, None


def batch_fault_stats():
    """Batches abandoned and which report ids caused them."""
    return {"count": _BATCH_FAULT_COUNT,
            "ids": sorted("0x%02X" % i for i in _BATCH_FAULT_IDS)}


def gravity_in_sensor(qx, qy, qz, qw):
    """World vertical expressed in the sensor frame: the third row of R(q).

    This is the only part of the Game Rotation Vector with an absolute
    reference, since it comes from the accelerometer. Yaw has none.
    """
    return (2.0 * (qx * qz - qw * qy),
            2.0 * (qy * qz + qw * qx),
            1.0 - 2.0 * (qx * qx + qy * qy))


class PivotIMU:
    """Pitch angle and rate about a calibrated pivot axis."""

    def __init__(self, calibration=None, i2c_bus=None,
                 report_interval_us=DEFAULT_REPORT_US,
                 enable_gyro=True,
                 stale_s=DEFAULT_STALE_S,
                 zero_n=DEFAULT_ZERO_N,
                 zero_spread_deg=DEFAULT_ZERO_SPREAD,
                 drift_warn=DEFAULT_DRIFT_WARN,
                 verbose=True):
        if calibration is None:
            self.cal = PivotCalibration.identity_frame()
        elif isinstance(calibration, PivotCalibration):
            self.cal = calibration
            self.cal.validate()
        else:
            self.cal = PivotCalibration.load(calibration)

        self.i2c_bus = i2c_bus
        self.report_interval_us = report_interval_us
        self.enable_gyro = enable_gyro
        self.stale_s = stale_s
        self.zero_n = zero_n
        self.zero_spread_deg = zero_spread_deg
        self.drift_warn = drift_warn
        self.verbose = verbose

        self.angle = 0.0
        self.rate = 0.0
        self.resid = 0.0
        self.offset = 0.0
        self.flags = F_NOZERO | (F_NOCAL if self.cal.identity else 0)

        self._dev = None
        self._lock = threading.Lock()
        self._resetting = False
        self._last_reset = 0.0
        self._last_sample = None
        self._last_fresh = time.monotonic()
        self._exc_run = 0
        self._zero_buf = []
        self._zeroed = False
        self._counts = {"reads": 0, "fresh": 0, "stalls": 0,
                        "resets": 0, "exceptions": 0}
        self._connect()

    # -- lifecycle -------------------------------------------------------

    def _log(self, msg):
        if self.verbose:
            print("[pivot_imu] %s" % msg)

    def _connect(self):
        try:
            from adafruit_bno08x import (BNO_REPORT_GAME_ROTATION_VECTOR,
                                         BNO_REPORT_GYROSCOPE)
            from adafruit_bno08x.i2c import BNO08X_I2C

            ok, why = patch_bno08x_batch_fault()
            if not ok:
                self._log("0x7B patch skipped: %s" % why)

            if self.i2c_bus is None:
                import board
                import busio
                i2c = busio.I2C(board.SCL, board.SDA)
            else:
                from adafruit_extended_bus import ExtendedI2C
                i2c = ExtendedI2C(self.i2c_bus)

            dev = BNO08X_I2C(i2c)
            # The library switches its packet dump on when it meets an unknown
            # report, which floods the console and slows the loop enough to
            # cause further faults.
            try:
                dev._debug = False
            except Exception:
                pass
            dev.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR,
                               self.report_interval_us)
            if self.enable_gyro:
                dev.enable_feature(BNO_REPORT_GYROSCOPE,
                                   self.report_interval_us)
            with self._lock:
                self._dev = dev
                self._last_sample = None
                self._last_fresh = time.monotonic()
        except Exception as exc:
            self._log("connect failed: %s: %s" % (type(exc).__name__, exc))
            with self._lock:
                self._dev = None

    def _schedule_reset(self, why):
        now = time.monotonic()
        with self._lock:
            if self._resetting or (now - self._last_reset) < RESET_COOLDOWN_S:
                return
            self._resetting = True
            self._last_reset = now
            self._dev = None
            self._zeroed = False
            self._zero_buf = []
            self._last_sample = None
            self._last_fresh = time.monotonic()
            self._exc_run = 0
            self._counts["resets"] += 1
        self._log("reset (%s), attempt %d" % (why, self._counts["resets"]))

        def worker():
            time.sleep(0.2)
            self._connect()
            with self._lock:
                self._resetting = False

        threading.Thread(target=worker, daemon=True).start()

    def close(self):
        with self._lock:
            self._dev = None

    # -- runtime ---------------------------------------------------------

    def request_zero(self):
        """Take the arm's current position as the new zero, once it is still."""
        with self._lock:
            self._zeroed = False
            self._zero_buf = []

    @property
    def healthy(self):
        """True only when the angle can be trusted as an absolute measurement."""
        return (self.flags & F_OK) and not (self.flags & (F_NOZERO | F_STALE))

    def read(self):
        """Poll the sensor once. Call at the loop rate; never raises."""
        self._counts["reads"] += 1
        with self._lock:
            dev = self._dev
            resetting = self._resetting

        if dev is None:
            self.flags = (self.flags & ~F_OK) | (F_RESETTING if resetting else 0)
            return False

        try:
            quat = dev.game_quaternion
            gyro = dev.gyro if self.enable_gyro else (0.0, 0.0, 0.0)
        except Exception:
            # Unknown SHTP report IDs raise here and the library never
            # resynchronises. Rebuild rather than serve the last reading.
            self._counts["exceptions"] += 1
            self._exc_run += 1
            self.flags &= ~F_OK
            if self._exc_run >= EXC_TOLERANCE:
                self._schedule_reset("%d consecutive read faults"
                                     % self._exc_run)
            return False

        if quat is None or gyro is None:
            self.flags &= ~F_OK
            return False

        # Defect 3. The library caches the last report, so a dead stream keeps
        # returning it: nothing raises, nothing returns None, and the angle
        # looks like a perfectly steady arm. The quaternion alone cannot tell
        # the two apart, since a still arm repeats it; the raw gyro settles it
        # because it always dithers.
        sample = (quat, gyro)
        if sample == self._last_sample:
            idle = time.monotonic() - self._last_fresh
            if idle >= self.stale_s:
                self._counts["stalls"] += 1
                self.flags = (self.flags | F_STALE) & ~F_OK
                self._schedule_reset("no new report for %.1f s" % idle)
            return False
        self._last_fresh = time.monotonic()
        self._exc_run = 0
        self._last_sample = sample
        self._counts["fresh"] += 1
        self.flags &= ~F_STALE

        qx, qy, qz, qw = quat
        if (qx * qx + qy * qy + qz * qz + qw * qw) < 0.5:
            # Defect 8. A freshly reset BNO085 returns a null quaternion until
            # its first report. Left through, it reads as a valid 0 deg.
            self.flags &= ~F_OK
            return False

        v = gravity_in_sensor(qx, qy, qz, qw)
        e1, e2, n = self.cal.e1, self.cal.e2, self.cal.n
        p1 = v[0] * e1[0] + v[1] * e1[1] + v[2] * e1[2]
        p2 = v[0] * e2[0] + v[1] * e2[1] + v[2] * e2[2]

        # Defects 4 and 5. atan2 on the projection, not asin on one component
        # and not an Euler angle: linear over the full +-180 deg whatever the
        # mounting orientation.
        raw = self.cal.sign * math.degrees(math.atan2(p2, p1))

        # Defect 9. Constant by construction, so any change means the bracket
        # has moved and the calibration no longer describes the bench.
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
        # Set from the state, not inside the acquisition branch: raising the
        # flag at the end of that branch re-armed it on the very cycle the
        # zero was taken, and nothing cleared it afterwards.
        if self._zeroed:
            self.flags &= ~F_NOZERO
        else:
            self.flags |= F_NOZERO

        # Defect 6. Re-wrap after removing the offset: the subtraction pushes
        # the result outside +-180, so one physical position can read 360 deg
        # apart depending on which side of the atan2 cut the raw value falls.
        self.angle = (raw - self.offset + 180.0) % 360.0 - 180.0

        # Defect 7. Rate about the pivot axis, not about the sensor y axis.
        # The sign is opposite to the angle's: the gravity vector is fixed in
        # the world and seen from a rotating frame, so d(g_s)/dt = -w x g_s
        # and the projected angle turns at -(w . n).
        self.rate = -self.cal.sign * math.degrees(
            gyro[0] * n[0] + gyro[1] * n[1] + gyro[2] * n[2])

        self.flags |= F_OK
        return True

    # -- diagnostics -----------------------------------------------------

    def stats(self):
        reads = max(1, self._counts["reads"])
        d = {}
        d.update(self._counts)
        d["fresh_ratio"] = self._counts["fresh"] / reads
        d["resid"] = self.resid
        d["drift"] = (None if self.cal.resid_ref is None
                      else self.resid - self.cal.resid_ref)
        d["flags"] = self.flags
        d["batch_faults"] = batch_fault_stats()
        return d

    def describe(self):
        names = [(F_OK, "OK"), (F_NOZERO, "NOZERO"), (F_STALE, "STALE"),
                 (F_RESETTING, "RESETTING"), (F_DRIFT, "DRIFT"),
                 (F_NOCAL, "NOCAL")]
        return "|".join(n for b, n in names if self.flags & b) or "-"

    # -- calibration -----------------------------------------------------

    @staticmethod
    def calibrate(path="pivot_calib.json", sweep_s=40.0, rate_hz=25.0,
                  min_span_deg=25.0, max_residual=0.02, min_radius=0.30,
                  i2c_bus=None):
        """Identify the pivot axis by sweeping the arm through its travel.

        The arm turns about a fixed axis, so gravity sweeps a circle in the
        sensor frame. Fitting the plane gives the axis with no assumption on
        the mounting. Sweep as far as the travel allows, slowly: past about
        100 deg/s the arm's own acceleration contaminates the fused gravity
        vector and thickens the plane.

        rate_hz must stay inside what the bus can carry. A wide sweep at
        25 Hz conditions the fit better than a narrow one at 100 Hz, so
        sampling slowly costs nothing here.
        """
        import numpy as np

        imu = PivotIMU(None, i2c_bus=i2c_bus, verbose=True)
        if imu._dev is None:
            raise CalibrationError("no BNO085 found")

        def grab(n, label):
            # Timestamp every sample. The bus does not always sustain rate_hz,
            # and deriving with the nominal step instead of the real one scales
            # the rate by exactly the ratio of the two -- which then shows up
            # as a failed gyro cross-check on a perfectly good axis fit.
            g, w, t, dt = [], [], [], 1.0 / rate_hz
            nxt = time.perf_counter()
            deadline = nxt + 5.0 * n * dt
            while len(g) < n:
                if time.perf_counter() > deadline:
                    raise CalibrationError(
                        "only %d of %d samples in %.0f s: the bus cannot "
                        "sustain %.0f Hz. Lower rate_hz, or lower "
                        "i2c_gpio_delay_us in config.txt."
                        % (len(g), n, 5.0 * n * dt, rate_hz))
                nxt += dt
                if imu.read():
                    sample = imu._last_sample
                    if sample is not None:
                        quat, gyro = sample
                        g.append(gravity_in_sensor(*quat))
                        w.append(gyro)
                        t.append(time.perf_counter())
                sl = nxt - time.perf_counter()
                time.sleep(sl) if sl > 0 else None
                if sl <= 0:
                    nxt = time.perf_counter()
            span = t[-1] - t[0] if len(t) > 1 else 0.0
            print("  %s %d samples, %.1f Hz effective"
                  % (label, n, (n - 1) / span if span > 0 else 0.0))
            return np.array(g), np.array(w), np.array(t)

        print("Pivot calibration. Motors unpowered; move the arm by hand.\n")
        input("1/3  Sweep its full travel, slowly, for %.0f s. Enter to start: "
              % sweep_s)
        G, W, T = grab(int(sweep_s * rate_hz), "sweep")

        centre = G.mean(axis=0)
        _, S, Vt = np.linalg.svd(G - centre)
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

        problems = []
        if swept < min_span_deg:
            problems.append("sweep of %.0f deg is too short; the fit is "
                            "ill-conditioned" % swept)
        if residual > max_residual:
            problems.append("out-of-plane rms %.4f: the bracket moved, or the "
                            "arm is not on a single axis" % residual)
        if radius < min_radius:
            problems.append("radius %.3f: the pivot is near vertical and "
                            "gravity cannot resolve the angle" % radius)
        if problems:
            raise CalibrationError("; ".join(problems))

        input("\n2/3  Let the arm rest, hands off. Enter: ")
        G_rest, _, _ = grab(100, "rest ")
        g_rest = G_rest.mean(axis=0)
        e1 = g_rest - (g_rest @ n) * n
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)

        input("\n3/3  Hold the arm clearly raised on the MAIN ROTOR side. Enter: ")
        G_up, _, _ = grab(100, "up   ")
        g_up = G_up.mean(axis=0)
        theta_up = math.degrees(math.atan2(g_up @ e2, g_up @ e1))
        if abs(theta_up) < 10.0:
            raise CalibrationError("raised reading is only %.1f deg; raise the "
                                   "arm further" % theta_up)
        sign = 1.0 if theta_up > 0 else -1.0

        # Cross-check against the gyro, whose scale is factory-calibrated and
        # independent of this fit. Agreement to a few percent validates the
        # axis; disagreement means the plane is wrong.
        th = np.unwrap(np.arctan2(G @ e2, G @ e1))
        rate_geom = np.degrees(np.gradient(th, T - T[0]))
        k = np.polyfit(np.degrees(W @ n), rate_geom, 1)[0]
        print("\n  gyro / geometry slope %+.4f (magnitude 1.000 expected)" % k)
        if abs(abs(k) - 1.0) > 0.15:
            raise CalibrationError("gyro and geometry disagree by %.0f %%; the "
                                   "fitted axis is wrong" % (100 * abs(abs(k) - 1)))

        cal = PivotCalibration(
            n, e1, e2, sign, resid_ref=float((G_rest @ n).mean()),
            meta={"date": time.strftime("%Y-%m-%d %H:%M:%S"),
                  "swept_deg": round(swept, 1),
                  "residual_rms": round(residual, 5),
                  "radius": round(radius, 5),
                  "gyro_slope": round(float(k), 4),
                  "samples": int(len(G))})
        cal.validate()
        cal.save(path)
        imu.close()
        print("\nWritten to %s" % os.path.abspath(path))
        return cal


def bench(seconds=6.0, rates=(10, 25, 50, 100), report_us=None, gyro=True,
          i2c_bus=None):
    """Measure the fresh-sample rate the link sustains at a paced read rate.

    Reading flat out is not a measurement: polling far faster than the sensor
    reports consumes its own processing budget, so it emits fewer reports and
    the figure comes out worse than at a sane rate. Each target rate is paced,
    and what matters is how many reads return new data.
    """
    print("report %d us, gyro %s\n"
          % (report_us or DEFAULT_REPORT_US, "on" if gyro else "off"))
    print("  target   reads/s   fresh/s   ratio   exc  faults")
    best = (0.0, 0)
    for hz in rates:
        imu = PivotIMU(None, i2c_bus=i2c_bus, enable_gyro=gyro,
                       report_interval_us=report_us or DEFAULT_REPORT_US,
                       verbose=False)
        if imu._dev is None:
            print("no BNO085 found")
            return
        base = batch_fault_stats()["count"]
        dt = 1.0 / hz
        t0 = time.perf_counter()
        nxt = t0
        while time.perf_counter() - t0 < seconds:
            nxt += dt
            imu.read()
            sl = nxt - time.perf_counter()
            if sl > 0:
                time.sleep(sl)
            else:
                nxt = time.perf_counter()
        el = time.perf_counter() - t0
        st = imu.stats()
        faults = st["batch_faults"]["count"] - base
        fresh = st["fresh"] / el
        print("  %4d Hz  %7.1f   %7.1f   %5.2f  %4d  %6d"
              % (hz, st["reads"] / el, fresh, st["fresh_ratio"],
                 st["exceptions"], faults))
        if fresh > best[0] and faults == 0:
            best = (fresh, hz)
        imu.close()
        time.sleep(0.5)
    print("\n  best clean rate: %.1f fresh/s at a %d Hz target" % best)
    return best


def _main():
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "bench":
        rep = int(sys.argv[2]) if len(sys.argv) > 2 else None
        gy = not (len(sys.argv) > 3 and sys.argv[3] == "nogyro")
        bench(report_us=rep, gyro=gy)
        return
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        out = sys.argv[2] if len(sys.argv) > 2 else "pivot_calib.json"
        PivotIMU.calibrate(out)
        return
    cal = "pivot_calib.json" if os.path.exists("pivot_calib.json") else None
    imu = PivotIMU(cal)
    print("Monitoring. Ctrl-C to stop.\n")
    try:
        while True:
            imu.read()
            print("\rangle %+8.2f  rate %+8.2f  %-28s" %
                  (imu.angle, imu.rate, imu.describe()), end="")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\n%s" % imu.stats())
        imu.close()


if __name__ == "__main__":
    _main()