#!/usr/bin/env python3
"""TRMS hardware-in-the-loop I/O server.

Safe USB-CDC version for MATLAB/Simulink.

Architecture:
    MATLAB/Simulink <-> USB CDC (/dev/ttyGS0) <-> Raspberry Pi
    Raspberry Pi <-> BNO085 UART-RVC
    Raspberry Pi -> ESC PWM

No control law runs on the Raspberry Pi in this mode. The Pi samples the
hardware, applies host PWM commands, and enforces local safety interlocks.

Required beside this file:
    pivot_imu.py
    pivot_imu_rvc.py
    pivot_calib.json

The wire protocol is intentionally unchanged relative to the previous version:
    command frame   = 12 bytes
    telemetry frame = 42 bytes
so the existing Simulink model remains compatible.
"""

import os
import struct
import subprocess
import threading
import time

import pigpio
import serial

from pivot_imu import F_DRIFT, F_NOZERO, F_OK, F_STALE
from pivot_imu_rvc import PivotRVC

PORT = "/dev/ttyGS0"
BAUD = 921600

# BNO085 UART-RVC acquisition.
ACQ_HZ = 100.0
ACQ_DT = 1.0 / ACQ_HZ

# Existing Simulink model runs at 0.02 s = 50 Hz.
HOST_DIV = 2
HOST_HZ = ACQ_HZ / HOST_DIV
PACE_ON_TICK = True

# If no valid host command is received for this duration, force neutral.
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

PITCH_LIMIT_DEG = 120.0
YAW_LIMIT_DEG = 65.0
LIMIT_RELEASE_DEG = 15.0

IMU_PITCH_LIMIT = True

# IMPORTANT:
# Set these True only after the corresponding encoder is physically wired
# and its electrical interface has been validated.
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

# Automatic zeroing is kept on the Raspberry Pi side.
# MATLAB no longer requests a zero automatically at every Simulink start.
AUTO_REZERO = True
AUTO_REZERO_S = 2.0
AUTO_REZERO_RATE = 3.0
AUTO_REZERO_BAND = 25.0

# Check Raspberry Pi undervoltage twice per second.
# A current undervoltage event latches a fault until this daemon is restarted.
UNDERVOLT_PERIOD = 50

CALIB_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "pivot_calib.json",
)
RVC_PORT = "/dev/serial0"

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


def crc16(data):
    c = 0xFFFF
    for b in data:
        c = ((c << 8) & 0xFFFF) ^ CRC_TABLE[((c >> 8) ^ b) & 0xFF]
    return c


def get_throttled():
    """Return Raspberry Pi get_throttled bitfield, or None if unavailable.

    Bit 0 means undervoltage is occurring now. Only the current bit is used to
    create a new latch, so a historical undervoltage from before daemon start
    does not prevent the bench from starting.
    """
    try:
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
            timeout=0.20,
            check=True,
        )
        text = result.stdout.strip()
        # Expected form: throttled=0x0
        value = text.split("=", 1)[1]
        return int(value, 16)
    except (FileNotFoundError, subprocess.SubprocessError, ValueError, IndexError):
        return None


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


class Daemon:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod not running")

        self.enc_pitch = Encoder(self.pi, PITCH_A, PITCH_B)
        self.enc_yaw = Encoder(self.pi, YAW_A, YAW_B)

        if not os.path.exists(CALIB_PATH):
            raise RuntimeError(
                "%s missing. Run: python3 pivot_imu_rvc.py calibrate" % CALIB_PATH
            )

        self.imu = PivotRVC(CALIB_PATH, port=RVC_PORT, verbose=True)
        if self.imu.cal.meta:
            print(
                "pivot calibration %s, %s deg swept, residual %s"
                % (
                    self.imu.cal.meta.get("date"),
                    self.imu.cal.meta.get("swept_deg"),
                    self.imu.cal.meta.get("residual_rms"),
                )
            )

        self.lock = threading.Lock()
        self.tick_cv = threading.Condition()
        self.host_tick = 0
        self.pace_count = 0

        self.armed = False
        self.target = [PWM_NEUTRAL, PWM_NEUTRAL]
        self.applied = [float(PWM_NEUTRAL), float(PWM_NEUTRAL)]
        self.last_cmd = 0.0

        self.status = 0
        self.tlm = [0.0] * 8
        self.seq = 0
        self.running = True

        self.uv_latched = False
        self.uv_tick = UNDERVOLT_PERIOD  # perform the first check immediately
        self.uv_monitor_warned = False

        self.fault = False
        self.acq_seq = 0
        self.still_time = 0.0
        self.rezero_done = False
        self.limit_latched = False
        self.drift_warned = False

        for pin in (ESC_MAIN, ESC_TAIL):
            self.pi.set_mode(pin, pigpio.OUTPUT)

        self._write_pwm(PWM_NEUTRAL, PWM_NEUTRAL)

    def _write_pwm(self, us_main, us_tail):
        duty = int(us_main * PWM_FREQ)
        self.pi.hardware_PWM(ESC_MAIN, PWM_FREQ, duty)

        duty = int(us_tail * PWM_FREQ)
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

            # PivotRVC.read() is expected to keep the newest RVC frame.
            self.imu.read()

            # Raspberry Pi supply check.
            self.uv_tick += 1
            if self.uv_tick >= UNDERVOLT_PERIOD:
                self.uv_tick = 0
                throttled = get_throttled()

                if throttled is None:
                    if not self.uv_monitor_warned:
                        self.uv_monitor_warned = True
                        print(
                            "WARNING: vcgencmd get_throttled unavailable; "
                            "Raspberry Pi undervoltage monitoring is disabled."
                        )
                else:
                    # Bit 0 = undervoltage currently detected.
                    if throttled & 0x1:
                        if not self.uv_latched:
                            print(
                                "FAULT: Raspberry Pi undervoltage detected. "
                                "Motors are latched disarmed until daemon restart."
                            )
                        self.uv_latched = True

            with self.lock:
                imu_flags = self.imu.flags

                # A stale IMU value must never be treated as a valid feedback
                # measurement.
                imu_ok = bool(imu_flags & F_OK) and not (imu_flags & F_STALE)
                status = ST_IMU_OK if imu_ok else 0

                if imu_flags & F_NOZERO:
                    status |= ST_NOZERO

                if not imu_ok or (imu_flags & F_NOZERO):
                    self.armed = False
                    self.target = [PWM_NEUTRAL, PWM_NEUTRAL]

                if (imu_flags & F_DRIFT) and not self.drift_warned:
                    self.drift_warned = True
                    print(
                        "WARNING: out-of-plane residual has moved; the IMU "
                        "bracket may have shifted. Rerun the calibration."
                    )

                # CHANGED: undervoltage is now an actual safety interlock, not
                # only a status bit.
                if self.uv_latched:
                    status |= ST_UNDERVOLT
                    self.armed = False
                    self.target = [PWM_NEUTRAL, PWM_NEUTRAL]

                if now - self.last_cmd > WATCHDOG_S:
                    self.armed = False
                    self.target = [PWM_NEUTRAL, PWM_NEUTRAL]
                    status |= ST_WATCHDOG

                over_pitch = (
                    ENC_PITCH_PRESENT and abs(pitch) > PITCH_LIMIT_DEG
                )
                over_yaw = ENC_YAW_PRESENT and abs(yaw) > YAW_LIMIT_DEG

                if imu_ok and IMU_PITCH_LIMIT:
                    over_pitch = (
                        over_pitch or abs(self.imu.angle) > PITCH_LIMIT_DEG
                    )

                if over_pitch or over_yaw:
                    self.limit_latched = True
                elif (
                    self.limit_latched
                    and imu_ok
                    and abs(self.imu.angle)
                    < PITCH_LIMIT_DEG - LIMIT_RELEASE_DEG
                ):
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
                    still = abs(self.imu.rate) < AUTO_REZERO_RATE
                    near = abs(self.imu.angle) < AUTO_REZERO_BAND

                    if (
                        still
                        and near
                        and imu_ok
                        and not (imu_flags & F_NOZERO)
                    ):
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
                    goal = (
                        self._sanitise(self.target[i])
                        if self.armed
                        else PWM_NEUTRAL
                    )
                    delta = goal - self.applied[i]

                    if abs(delta) > max_step:
                        delta = max_step if delta > 0 else -max_step
                        status |= ST_SLEW

                    self.applied[i] += delta
                    out.append(int(round(self.applied[i])))

                # Preserve a CRC fault set by the serial server until a new
                # valid command clears it.
                self.status = (self.status & ST_CRC_ERR) | status
                self.acq_seq += 1

                # Wire format unchanged.
                self.tlm = [
                    pitch,                                          # 1
                    rate_pitch,                                     # 2
                    yaw,                                            # 3
                    rate_yaw,                                       # 4
                    self.imu.angle,                                 # 5
                    self.imu.frame.yaw if self.imu.frame else 0.0,  # 6
                    self.imu.rate,                                  # 7
                    0.0,                                            # 8 spare
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
                    # CHANGED: corrupted host command immediately disarms the
                    # bench instead of allowing the previous PWM target to stay
                    # active until the watchdog expires.
                    with self.lock:
                        self.status |= ST_CRC_ERR
                        self.armed = False
                        self.target = [PWM_NEUTRAL, PWM_NEUTRAL]
                    del buf[0]
                    continue

                del buf[:CMD_LEN]

                requested_arm = bool(fields[2] & 0x01)
                rezero = bool(fields[2] & 0x02)

                with self.lock:
                    self.status &= ~ST_CRC_ERR

                    # An undervoltage fault is latched locally and cannot be
                    # overridden by a host arm command.
                    self.armed = requested_arm and not self.uv_latched

                    if self.armed:
                        self.target = [fields[6], fields[7]]
                    else:
                        self.target = [PWM_NEUTRAL, PWM_NEUTRAL]

                    self.last_cmd = time.perf_counter()

                if rezero:
                    self.imu.request_zero()

                if PACE_ON_TICK:
                    with self.tick_cv:
                        due = self.host_tick + 1
                        self.tick_cv.wait_for(
                            lambda: self.host_tick >= due,
                            timeout=2 * HOST_DIV * ACQ_DT,
                        )

                with self.lock:
                    status = self.status
                    tlm = list(self.tlm)
                    acq = self.acq_seq & 0xFFFFFFFF

                self.seq = (self.seq + 1) & 0xFF

                # Kept for wire compatibility. It is an acquisition sequence
                # counter, not a physical timestamp.
                t_us = acq

                body = struct.pack(
                    TLM_FMT[:-1],
                    MAGIC0,
                    MAGIC1,
                    status,
                    self.seq,
                    t_us,
                    *tlm,
                )
                port.write(body + struct.pack("<H", crc16(body)))

    def shutdown(self):
        self.running = False
        time.sleep(2 * ACQ_DT)

        self._write_pwm(PWM_NEUTRAL, PWM_NEUTRAL)

        self.enc_pitch.cancel()
        self.enc_yaw.cancel()
        self.imu.close()
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
