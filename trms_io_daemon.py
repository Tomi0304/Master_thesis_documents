#!/usr/bin/env python3
"""TRMS hardware-in-the-loop I/O server.

Exposes the TRMS sensors and ESCs to MATLAB/Simulink over the USB CDC-ACM
gadget link. No control law runs here: the daemon samples the plant, applies
the PWM commanded by the host, and enforces the safety interlock.
"""

import glob
import math
import struct
import threading
import time

import pigpio
import serial

PORT = "/dev/ttyGS0"
BAUD = 921600

ACQ_HZ = 50.0
ACQ_DT = 1.0 / ACQ_HZ

HOST_DIV = 1
HOST_HZ = ACQ_HZ / HOST_DIV
PACE_ON_TICK = True

WATCHDOG_S = 0.100

PITCH_A, PITCH_B = 27, 17
YAW_A, YAW_B = 22, 23
ENC_CPR = 2000.0
ENC_DEG = 360.0 / ENC_CPR
RATE_TAU = 0.010

ESC_MAIN, ESC_TAIL = 12, 13
PWM_FREQ = 200
PWM_NEUTRAL = 1500
PWM_SPAN = 390
PWM_MIN = PWM_NEUTRAL - PWM_SPAN
PWM_MAX = PWM_NEUTRAL + PWM_SPAN
PWM_DEADBAND = 25
SLEW_US_PER_S = 3000.0

# The pitch axis has no mechanical stop, so this limit is the only thing that
# stops the beam going over the top. It was previously 140 deg, which the old
# Euler projection could never reach because that projection saturated near
# 110 deg: the interlock was inert. With the pivot projection the angle is a
# true angle, so set a value that means something.
PITCH_LIMIT_DEG = 95.0
YAW_LIMIT_DEG = 65.0

# Once tripped the interlock stays tripped until the arm is well back inside.
# Without this hysteresis it releases the instant the angle dips below the
# threshold, and the bench cycles between saturated command and cutoff: the
# arm falls, re-arms on a command that is still saturated, and climbs again.
LIMIT_RELEASE_DEG = 15.0

# Back the pitch limit with the IMU as well as the encoder. Set False only if a
# pitch encoder is wired and trusted on its own.
IMU_PITCH_LIMIT = True

# Unwired encoder inputs float on their pull-ups and pick up edges from the
# motors, so their counters drift and trip the angle limit with no encoder
# present. Set each flag True only once that encoder is actually wired.
ENC_PITCH_PRESENT = False
ENC_YAW_PRESENT = False

MAGIC0, MAGIC1 = 0xA5, 0x5A
CMD_FMT = "<BBBBBBhhH"
TLM_FMT = "<BBBBI8fH"
CMD_LEN = struct.calcsize(CMD_FMT)
TLM_LEN = struct.calcsize(TLM_FMT)

ST_ARMED = 0x01
ST_WATCHDOG = 0x02
ST_LIMIT = 0x04
ST_IMU_OK = 0x08
ST_CRC_ERR = 0x10
ST_SLEW = 0x20
ST_UNDERVOLT = 0x40
ST_NOZERO = 0x80

# The BNO085 zero depends on the orientation in which it initialised, so the
# raw pitch is meaningless as an absolute angle. Take the arm's resting
# position as zero: average the first ZERO_N samples once they hold within
# ZERO_SPREAD_DEG, which requires the arm to be still at startup.
ZERO_N = 40
ZERO_SPREAD_DEG = 1.5

# Re-acquire the zero whenever the bench has been disarmed and motionless for
# AUTO_REZERO_S. The arm at rest is the reference by definition, so each run
# starts from a fresh zero without restarting the daemon. Set False to keep the
# single zero taken at startup.
AUTO_REZERO = True
AUTO_REZERO_S = 2.0
AUTO_REZERO_RATE = 3.0
# Only re-zero if the arm has come back within this of the current zero. Without
# it, any pause while handling the arm redefines the reference wherever it
# happens to be. Also fires at most once per disarm.
AUTO_REZERO_BAND = 25.0

UNDERVOLT_GLOB = "/sys/class/hwmon/hwmon*/in0_lcrit_alarm"
UNDERVOLT_PERIOD = 100

# With patch_bno08x_batch_fault() applied, a malformed batch costs one report
# instead of the whole stream, so the IMU can be polled at the full loop rate.
# At IMU_DIV = 2 the angle is refreshed at 25 Hz only, so the measurement the
# loop closes on is up to 40 ms old: 3.4 deg of phase at 1.5 rad/s. Raise it
# back to 2 only if CPU goes past ~70 %: top -bn1 | grep python3
IMU_DIV = 1

IMU_ENABLE_GYRO = True

# Interval between sensor reports, in microseconds. The Adafruit library
# defaults to 50000 us (20 Hz), which caps the freshness of the whole chain no
# matter how fast the loop polls. 10000 us gives 100 Hz. Going faster loads the
# bit-banged bus and raises the malformed-batch rate, so check CPU after a
# change: top -bn1 | grep python3
IMU_REPORT_INTERVAL_US = 10000

# The BCM2711 hardware I2C controller aborts transfers when the BNO085 stretches
# the clock, surfacing as OSError errno 5. The fix is a bit-banged bus, which
# polls the line every cycle and waits properly:
#   #dtparam=i2c_arm=on
#   dtoverlay=i2c-gpio,bus=1,i2c_gpio_sda=2,i2c_gpio_scl=3
# Giving it bus number 1 lets Blinka find it on board.SCL / board.SDA exactly as
# it found the hardware bus, so leave I2C_BUS at None. Set it to a bus number
# only if the software bus is created under a different number, which requires
# adafruit-extended-bus and is rejected by some Blinka versions.
I2C_BUS = None

# --- Pitch pivot axis, identified by calib_pivot.py ------------------------
# The arm turns about a fixed axis, so in the sensor frame the world vertical
# sweeps a circle whose normal is that axis. PIVOT_N is that normal, PIVOT_E1
# and PIVOT_E2 an orthonormal basis of the circle's plane, and PIVOT_SIGN
# orients the result so that raising the main-rotor side reads positive.
#
# This replaces the Euler pitch, which assumed the pivot ran along the IMU y
# axis. On this bench the pivot is 89.5 deg away from y and 0.4 deg from z, so
# that projection sat in permanent near-gimbal-lock: its error was nonlinear,
# it saturated around 110 deg, and no scale factor could correct it.
#
# Rerun calib_pivot.py whenever the IMU or its bracket is disturbed.
PIVOT_N = (+0.001386, +0.007480, +0.999971)
PIVOT_E1 = (-0.024372, +0.999675, -0.007444)
PIVOT_E2 = (-0.999702, -0.024361, +0.001568)
PIVOT_SIGN = -1.0

# Out-of-plane drift, in the units of the unit gravity vector, beyond which the
# calibration is reported as stale. 0.02 is roughly 1 deg of axis movement.
PIVOT_DRIFT_WARN = 0.02

QUAD_LUT = (0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0)


def _build_crc_table():
    table = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        table.append(c)
    return tuple(table)


CRC_TABLE = _build_crc_table()


def find_undervolt_sensor():
    for path in sorted(glob.glob(UNDERVOLT_GLOB)):
        return path
    return None


def crc16(data):
    c = 0xFFFF
    for b in data:
        c = ((c << 8) & 0xFFFF) ^ CRC_TABLE[((c >> 8) ^ b) & 0xFF]
    return c


class Encoder:
    def __init__(self, pi, pin_a, pin_b):
        self.pi = pi
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.count = 0
        for pin in (pin_a, pin_b):
            pi.set_mode(pin, pigpio.INPUT)
            pi.set_pull_up_down(pin, pigpio.PUD_UP)
        self._state = (pi.read(pin_a) << 1) | pi.read(pin_b)
        self._cb = [
            pi.callback(pin_a, pigpio.EITHER_EDGE, self._edge),
            pi.callback(pin_b, pigpio.EITHER_EDGE, self._edge),
        ]

    def _edge(self, gpio, level, tick):
        state = (self.pi.read(self.pin_a) << 1) | self.pi.read(self.pin_b)
        self.count += QUAD_LUT[(self._state << 2) | state]
        self._state = state

    def zero(self):
        self.count = 0

    def cancel(self):
        for cb in self._cb:
            cb.cancel()


def patch_bno08x_batch_fault():
    """Teach the Adafruit BNO08x library to step over report ID 0x7B.

    The library raises on any report ID absent from its lookup table and has
    no resynchronisation, so one malformed batch kills the stream for good.
    In every reported case the offending ID is 0x7B followed by four bytes,
    the same 5-byte shape as the 0xFB timestamp reference that precedes each
    batch. Registering it with a zero value count makes the splitter skip it
    and carry on to the real report behind it.

    Applied at runtime rather than by editing the installed package, so the
    platform stays reproducible from a plain pip install.
    """
    try:
        import adafruit_bno08x as bno
    except Exception as exc:
        print("BNO08x patch skipped: %s" % exc)
        return False

    try:
        reports = bno._AVAIL_SENSOR_REPORTS
    except AttributeError:
        print("BNO08x patch skipped: report table not found")
        return False

    for report_id in (0x7B,):
        if report_id not in reports:
            # (scalar, value count, report length in bytes)
            reports[report_id] = (1, 0, 5)
    return True


class IMU:
    def __init__(self):
        self.ok = False
        self.pitch = 0.0
        self.yaw = 0.0
        self.gyro_pitch = 0.0
        self.gyro_yaw = 0.0
        self._dev = None
        self._lock = threading.Lock()
        self._resetting = False
        self.resets = 0
        self.offset = 0.0
        self.yaw_offset = 0.0
        self._pending_yaw = 0.0
        self.zeroed = False
        self._zero_buf = []
        self._yaw_buf = []
        # Out-of-plane component of the gravity vector. Constant by
        # construction, so any change means the IMU bracket has moved and the
        # pivot calibration no longer describes the bench.
        self.resid = 0.0
        self.resid_ref = None
        self._connect()

    def _connect(self):
        try:
            from adafruit_bno08x import (
                BNO_REPORT_GAME_ROTATION_VECTOR,
                BNO_REPORT_GYROSCOPE,
            )
            from adafruit_bno08x.i2c import BNO08X_I2C

            patch_bno08x_batch_fault()

            if I2C_BUS is None:
                import board
                import busio

                i2c = busio.I2C(board.SCL, board.SDA)
            else:
                from adafruit_extended_bus import ExtendedI2C

                i2c = ExtendedI2C(I2C_BUS)

            dev = BNO08X_I2C(i2c)
            dev.enable_feature(
                BNO_REPORT_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US
            )
            if IMU_ENABLE_GYRO:
                dev.enable_feature(BNO_REPORT_GYROSCOPE, IMU_REPORT_INTERVAL_US)
            with self._lock:
                self._dev = dev
                self.ok = True
        except Exception as exc:
            print("IMU connect failed: %s: %s" % (type(exc).__name__, exc))
            with self._lock:
                self._dev = None
                self.ok = False

    def request_zero(self):
        with self._lock:
            if not self.zeroed:
                return
            self.zeroed = False
            self._zero_buf = []

    def _schedule_reset(self):
        with self._lock:
            if self._resetting:
                return
            self._resetting = True
            self._dev = None
            self.ok = False
            self.zeroed = False
            self._zero_buf = []
            self._yaw_buf = []
            self.resets += 1

        def worker():
            time.sleep(0.2)
            self._connect()
            with self._lock:
                self._resetting = False

        threading.Thread(target=worker, daemon=True).start()

    def read(self):
        with self._lock:
            dev = self._dev
        if dev is None:
            return
        try:
            quat = dev.game_quaternion
            gyro = dev.gyro if IMU_ENABLE_GYRO else (0.0, 0.0, 0.0)
        except Exception:
            # The Adafruit BNO08x library raises on unknown SHTP report IDs
            # (KeyError 123 / report 0x7B) and never resynchronises. Rebuild
            # the object rather than serve the last reading forever.
            self._schedule_reset()
            return
        if quat is None or gyro is None:
            return

        qx, qy, qz, qw = quat
        if (qx * qx + qy * qy + qz * qz + qw * qw) < 0.5:
            # A freshly reset BNO085 returns a null quaternion until its first
            # report arrives. Left through, it reads as a valid 0 deg.
            return
        gx, gy, gz = gyro

        # World vertical expressed in the sensor frame: the third row of R(q).
        # This is the only part of the Game Rotation Vector with an absolute
        # reference, since it comes from the accelerometer.
        v0 = 2.0 * (qx * qz - qw * qy)
        v1 = 2.0 * (qy * qz + qw * qx)
        v2 = 1.0 - 2.0 * (qx * qx + qy * qy)

        # Project onto the calibrated plane of the pivot. Linear over the full
        # +-180 deg and independent of how the IMU is mounted, where the Euler
        # pitch it replaces was neither.
        p1 = v0 * PIVOT_E1[0] + v1 * PIVOT_E1[1] + v2 * PIVOT_E1[2]
        p2 = v0 * PIVOT_E2[0] + v1 * PIVOT_E2[1] + v2 * PIVOT_E2[2]
        raw_pitch = PIVOT_SIGN * math.degrees(math.atan2(p2, p1))

        self.resid = v0 * PIVOT_N[0] + v1 * PIVOT_N[1] + v2 * PIVOT_N[2]

        if not self.zeroed:
            self._zero_buf.append(raw_pitch)
            if len(self._zero_buf) > ZERO_N:
                self._zero_buf.pop(0)
            if len(self._zero_buf) == ZERO_N:
                spread = max(self._zero_buf) - min(self._zero_buf)
                if spread < ZERO_SPREAD_DEG:
                    self.offset = sum(self._zero_buf) / ZERO_N
                    self.yaw_offset = self._pending_yaw
                    self.zeroed = True
                    if self.resid_ref is None:
                        self.resid_ref = self.resid
                    drift = self.resid - self.resid_ref
                    print("IMU zero: pitch %+.2f deg, yaw %+.2f deg, "
                          "out-of-plane %+.4f (drift %+.4f)"
                          % (self.offset, self.yaw_offset, self.resid, drift))
                    if abs(drift) > PIVOT_DRIFT_WARN:
                        print("  WARNING: the IMU bracket appears to have "
                              "moved. Rerun calib_pivot.py.")

        # Re-wrap after removing the offset. atan2 returns +-180, but the
        # subtraction pushes the result outside that range, so a single physical
        # position can read 360 deg apart depending on which side of the atan2
        # cut the raw value falls. That step is what breaks the loop.
        self.pitch = (raw_pitch - self.offset + 180.0) % 360.0 - 180.0

        # Yaw still goes through the Euler projection and carries the same
        # distortion as the old pitch did. It is not corrected here because the
        # Game Rotation Vector has no magnetic reference, so yaw drifts anyway
        # and must not be closed on. It is kept for monitoring only; robust yaw
        # needs the HEDS-5540 on that axis.
        raw_yaw = math.degrees(
            math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        )
        self._pending_yaw = raw_yaw
        self.yaw = raw_yaw - self.yaw_offset

        # Rate about the pivot axis, not about the sensor y axis. The sign is
        # the opposite of PIVOT_SIGN: the gravity vector is fixed in the world
        # and seen from a rotating frame, so d(g_s)/dt = -w x g_s and the
        # projected angle turns at -(w . n).
        self.gyro_pitch = -PIVOT_SIGN * math.degrees(
            gx * PIVOT_N[0] + gy * PIVOT_N[1] + gz * PIVOT_N[2]
        )
        self.gyro_yaw = math.degrees(gz)
        self.ok = True


class Daemon:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod not running")

        self.enc_pitch = Encoder(self.pi, PITCH_A, PITCH_B)
        self.enc_yaw = Encoder(self.pi, YAW_A, YAW_B)
        self.imu = IMU()

        self.lock = threading.Lock()
        self.tick_cv = threading.Condition()
        self.host_tick = 0
        self.pace_count = 0
        self.armed = False
        self.target = [PWM_NEUTRAL, PWM_NEUTRAL]
        self.applied = [float(PWM_NEUTRAL), float(PWM_NEUTRAL)]
        self.last_cmd = 0.0
        self.status = ST_IMU_OK if self.imu.ok else 0
        self.tlm = [0.0] * 8
        self.seq = 0
        self.running = True
        self.t0 = time.perf_counter()

        self.uv_path = find_undervolt_sensor()
        self.uv_latched = False
        self.uv_tick = 0
        self.fault = False
        self.acq_seq = 0
        self.still_time = 0.0
        self.rezero_done = False
        self.limit_latched = False

        for pin in (ESC_MAIN, ESC_TAIL):
            self.pi.set_mode(pin, pigpio.OUTPUT)
        self._write_pwm(PWM_NEUTRAL, PWM_NEUTRAL)

    def _write_pwm(self, us_main, us_tail):
        duty = int(us_main * PWM_FREQ * 1e-6 * 1e6)
        self.pi.hardware_PWM(ESC_MAIN, PWM_FREQ, duty)
        duty = int(us_tail * PWM_FREQ * 1e-6 * 1e6)
        self.pi.hardware_PWM(ESC_TAIL, PWM_FREQ, duty)

    @staticmethod
    def _sanitise(us):
        us = max(PWM_MIN, min(PWM_MAX, us))
        if abs(us - PWM_NEUTRAL) < PWM_DEADBAND:
            us = PWM_NEUTRAL
        return us

    def acquisition(self):
        try:
            self._acquisition_loop()
        except BaseException:
            import traceback

            traceback.print_exc()
            self.running = False
            self.fault = True

    def _acquisition_loop(self):
        prev_pitch = 0.0
        prev_yaw = 0.0
        rate_pitch = 0.0
        rate_yaw = 0.0
        imu_count = 0
        alpha = ACQ_DT / (RATE_TAU + ACQ_DT)
        next_tick = time.perf_counter()

        while self.running:
            next_tick += ACQ_DT
            now = time.perf_counter()

            pitch = self.enc_pitch.count * ENC_DEG
            yaw = self.enc_yaw.count * ENC_DEG
            rate_pitch += alpha * ((pitch - prev_pitch) / ACQ_DT - rate_pitch)
            rate_yaw += alpha * ((yaw - prev_yaw) / ACQ_DT - rate_yaw)
            prev_pitch, prev_yaw = pitch, yaw

            imu_count += 1
            if imu_count >= IMU_DIV:
                imu_count = 0
                self.imu.read()

            self.uv_tick += 1
            if self.uv_path and self.uv_tick >= UNDERVOLT_PERIOD:
                self.uv_tick = 0
                try:
                    with open(self.uv_path) as handle:
                        if handle.read().strip() != "0":
                            self.uv_latched = True
                except OSError:
                    pass

            with self.lock:
                status = ST_IMU_OK if self.imu.ok else 0
                if not self.imu.zeroed:
                    status |= ST_NOZERO
                    self.armed = False
                    self.target = [PWM_NEUTRAL, PWM_NEUTRAL]
                if self.uv_latched:
                    status |= ST_UNDERVOLT
                if now - self.last_cmd > WATCHDOG_S:
                    self.armed = False
                    self.target = [PWM_NEUTRAL, PWM_NEUTRAL]
                    status |= ST_WATCHDOG
                over_pitch = ENC_PITCH_PRESENT and abs(pitch) > PITCH_LIMIT_DEG
                over_yaw = ENC_YAW_PRESENT and abs(yaw) > YAW_LIMIT_DEG
                if self.imu.ok and IMU_PITCH_LIMIT:
                    # The encoder reads zero when none is wired, which leaves the
                    # limit inert. The IMU pitch is zeroed at rest so it is a
                    # usable absolute angle. The IMU yaw is not: the game
                    # rotation vector has no magnetic reference, so it drifts and
                    # would trip the limit continuously. Yaw protection therefore
                    # relies on its encoder alone.
                    over_pitch = over_pitch or abs(self.imu.pitch) > PITCH_LIMIT_DEG
                if over_pitch or over_yaw:
                    self.limit_latched = True
                elif self.limit_latched and self.imu.ok and \
                        abs(self.imu.pitch) < PITCH_LIMIT_DEG - LIMIT_RELEASE_DEG:
                    self.limit_latched = False
                if self.limit_latched:
                    self.armed = False
                    self.target = [PWM_NEUTRAL, PWM_NEUTRAL]
                    status |= ST_LIMIT
                if self.armed:
                    status |= ST_ARMED
                    self.still_time = 0.0
                    self.rezero_done = False
                elif AUTO_REZERO and not self.rezero_done:
                    # From the IMU, not the encoder: with ENC_PITCH_PRESENT
                    # False the counter never moves, so rate_pitch is always 0
                    # and this test always passes -- the zero could then be
                    # re-taken on a moving arm after any limit trip.
                    still = abs(self.imu.gyro_pitch) < AUTO_REZERO_RATE
                    near = abs(self.imu.pitch) < AUTO_REZERO_BAND
                    if still and near and self.imu.zeroed:
                        self.still_time += ACQ_DT
                        if self.still_time >= AUTO_REZERO_S:
                            self.still_time = 0.0
                            self.rezero_done = True
                            self.imu.request_zero()
                    elif not still:
                        self.still_time = 0.0

                max_step = SLEW_US_PER_S * ACQ_DT
                out = []
                for i in range(2):
                    goal = self._sanitise(self.target[i]) if self.armed else PWM_NEUTRAL
                    delta = goal - self.applied[i]
                    if abs(delta) > max_step:
                        delta = max_step if delta > 0 else -max_step
                        status |= ST_SLEW
                    self.applied[i] += delta
                    out.append(int(round(self.applied[i])))

                self.status = (self.status & ST_CRC_ERR) | status
                self.acq_seq += 1
                self.tlm = [
                    pitch,
                    rate_pitch,
                    yaw,
                    rate_yaw,
                    self.imu.pitch,
                    self.imu.yaw,
                    self.imu.gyro_pitch,
                    self.imu.gyro_yaw,
                ]

            self._write_pwm(out[0], out[1])

            self.pace_count += 1
            if self.pace_count >= HOST_DIV:
                self.pace_count = 0
                with self.tick_cv:
                    self.host_tick += 1
                    self.tick_cv.notify_all()

            sleep = next_tick - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.perf_counter()

    def serve(self):
        port = serial.Serial(PORT, BAUD, timeout=0.2)
        port.reset_input_buffer()
        buf = bytearray()

        while self.running:
            chunk = port.read(CMD_LEN)
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= CMD_LEN:
                if buf[0] != MAGIC0 or buf[1] != MAGIC1:
                    del buf[0]
                    continue
                frame = bytes(buf[:CMD_LEN])
                fields = struct.unpack(CMD_FMT, frame)
                if crc16(frame[:-2]) != fields[8]:
                    with self.lock:
                        self.status |= ST_CRC_ERR
                    del buf[0]
                    continue
                del buf[:CMD_LEN]

                with self.lock:
                    self.status &= ~ST_CRC_ERR
                    self.armed = bool(fields[2] & 0x01)
                    rezero = bool(fields[2] & 0x02)
                    self.target = [fields[6], fields[7]]
                    self.last_cmd = time.perf_counter()

                if rezero:
                    self.imu.request_zero()

                if PACE_ON_TICK:
                    with self.tick_cv:
                        due = self.host_tick + 1
                        self.tick_cv.wait_for(
                            lambda: self.host_tick >= due, timeout=2 * HOST_DIV * ACQ_DT
                        )

                with self.lock:
                    status = self.status
                    tlm = list(self.tlm)
                    acq = self.acq_seq & 0xFFFFFFFF

                self.seq = (self.seq + 1) & 0xFF
                t_us = acq
                body = struct.pack(
                    TLM_FMT[:-1], MAGIC0, MAGIC1, status, self.seq, t_us, *tlm
                )
                port.write(body + struct.pack("<H", crc16(body)))

    def shutdown(self):
        self.running = False
        time.sleep(2 * ACQ_DT)
        self._write_pwm(PWM_NEUTRAL, PWM_NEUTRAL)
        self.enc_pitch.cancel()
        self.enc_yaw.cancel()
        self.pi.stop()


def main():
    daemon = Daemon()
    thread = threading.Thread(target=daemon.acquisition, daemon=True)
    thread.start()
    try:
        daemon.serve()
    except KeyboardInterrupt:
        pass
    finally:
        daemon.shutdown()


if __name__ == "__main__":
    main()