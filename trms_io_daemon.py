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

ACQ_HZ = 200.0
ACQ_DT = 1.0 / ACQ_HZ

HOST_DIV = 4
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

PITCH_LIMIT_DEG = 140.0
YAW_LIMIT_DEG = 65.0

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

UNDERVOLT_GLOB = "/sys/class/hwmon/hwmon*/in0_lcrit_alarm"
UNDERVOLT_PERIOD = 100

# With patch_bno08x_batch_fault() applied, a malformed batch costs one report
# instead of the whole stream, so the IMU can be polled at the full loop rate
# again. Raise IMU_DIV if the I2C bus needs relief.
IMU_DIV = 2

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

    def _schedule_reset(self):
        with self._lock:
            if self._resetting:
                return
            self._resetting = True
            self._dev = None
            self.ok = False
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

        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))

        self.pitch = math.degrees(math.asin(sinp))
        self.yaw = math.degrees(
            math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        )
        self.gyro_pitch = math.degrees(gy)
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
                if self.uv_latched:
                    status |= ST_UNDERVOLT
                if now - self.last_cmd > WATCHDOG_S:
                    self.armed = False
                    self.target = [PWM_NEUTRAL, PWM_NEUTRAL]
                    status |= ST_WATCHDOG
                if abs(pitch) > PITCH_LIMIT_DEG or abs(yaw) > YAW_LIMIT_DEG:
                    self.armed = False
                    self.target = [PWM_NEUTRAL, PWM_NEUTRAL]
                    status |= ST_LIMIT
                if self.armed:
                    status |= ST_ARMED

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
                    self.target = [fields[6], fields[7]]
                    self.last_cmd = time.perf_counter()

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