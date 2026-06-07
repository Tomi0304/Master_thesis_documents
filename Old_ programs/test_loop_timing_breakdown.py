#!/usr/bin/env python3
"""
test_loop_timing_breakdown.py

Measure the time taken by each component of a 200 Hz AP/TRMS control loop:
  - encoder read
  - IMU read
  - optional gyro read
  - PID computation
  - PWM conversion
  - PWM send through pigpio
  - CSV logging
  - total compute time and remaining timing margin

Default behaviour is safe: the script sends only neutral PWM = 1500 us.
Use --send-active-pwm only if you intentionally want to send the computed PWM.

Usage:
    sudo pigpiod
    python3 test_loop_timing_breakdown.py
    python3 test_loop_timing_breakdown.py --duration 30 --enable-gyro
    python3 test_loop_timing_breakdown.py --no-imu
    python3 test_loop_timing_breakdown.py --no-pwm
"""

import argparse
import csv
import math
import os
import threading
import time
from datetime import datetime

import numpy as np
import pigpio

try:
    import board
    import busio
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
    IMU_IMPORT_OK = True
except Exception:
    IMU_IMPORT_OK = False


CONTROL_FREQ_HZ = 200.0
LOOP_DT = 1.0 / CONTROL_FREQ_HZ

ENCODER_PIN_A = 27
ENCODER_PIN_B = 17
ENCODER_CPR_X4 = 2000

ESC_MAIN_PWM_PIN = 12
ESC_NEUTRAL_US = 1500
ESC_MIN_US = 1100
ESC_MAX_US = 1900
ESC_DEADBAND_US = 25

I2C_FREQ_HZ = 400_000
BNO_ADDRESS = 0x4A
IMU_INTERVAL_US = 5000

WARMUP_SAMPLES = 20


def precise_sleep_until(target_time):
    while True:
        now = time.perf_counter()
        remaining = target_time - now
        if remaining <= 0:
            return
        if remaining > 0.001:
            time.sleep(remaining - 0.0005)
        else:
            # short busy wait for sub-ms accuracy
            pass


def quat_to_euler_pitch_yaw(qi, qj, qk, qr):
    sinp = 2.0 * (qr * qj - qk * qi)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny = 2.0 * (qr * qk + qi * qj)
    cosy = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw = math.degrees(math.atan2(siny, cosy))

    return pitch, yaw


def stats_ms(values):
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return {k: np.nan for k in ("mean", "std", "min", "p50", "p95", "p99", "max")}
    return {
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
        "min": float(np.min(arr)),
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "p99": float(np.percentile(arr, 99)),
        "max": float(np.max(arr)),
    }


class EncoderHEDS5540:
    _QUAD_TABLE = [
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0,
    ]

    def __init__(self, pi):
        self.pi = pi
        self._position = 0
        self._lock = threading.Lock()
        self._psi_prev = 0.0
        self._psi_dot_filt = 0.0
        self.alpha = 0.3

        pi.set_mode(ENCODER_PIN_A, pigpio.INPUT)
        pi.set_mode(ENCODER_PIN_B, pigpio.INPUT)
        pi.set_pull_up_down(ENCODER_PIN_A, pigpio.PUD_OFF)
        pi.set_pull_up_down(ENCODER_PIN_B, pigpio.PUD_OFF)

        a = pi.read(ENCODER_PIN_A)
        b = pi.read(ENCODER_PIN_B)
        self._last_state = (a << 1) | b

        self._cb_a = pi.callback(ENCODER_PIN_A, pigpio.EITHER_EDGE, self._cb)
        self._cb_b = pi.callback(ENCODER_PIN_B, pigpio.EITHER_EDGE, self._cb)

    def _cb(self, gpio, level, tick):
        a = self.pi.read(ENCODER_PIN_A)
        b = self.pi.read(ENCODER_PIN_B)
        current = (a << 1) | b
        idx = (self._last_state << 2) | current
        with self._lock:
            self._position += self._QUAD_TABLE[idx]
        self._last_state = current

    @property
    def position(self):
        with self._lock:
            return self._position

    def reset(self):
        with self._lock:
            self._position = 0
        self._psi_prev = 0.0
        self._psi_dot_filt = 0.0

    def get_psi_and_dot(self):
        psi = -((self.position / ENCODER_CPR_X4) * 360.0)
        psi_dot_raw = (psi - self._psi_prev) / LOOP_DT
        self._psi_dot_filt = self.alpha * psi_dot_raw + (1.0 - self.alpha) * self._psi_dot_filt
        self._psi_prev = psi
        return psi, self._psi_dot_filt

    def close(self):
        self._cb_a.cancel()
        self._cb_b.cancel()


class BNO085TimingReader:
    def __init__(self, enable_gyro=False):
        if not IMU_IMPORT_OK:
            raise RuntimeError("Adafruit BNO08x imports failed")

        self.enable_gyro = enable_gyro
        self.report_count = 0
        self.report_timestamps = []
        self.value_change_count = 0
        self._last_quat = None
        self._hook_installed = False

        i2c = busio.I2C(board.SCL, board.SDA, frequency=I2C_FREQ_HZ)
        self.bno = BNO08X_I2C(i2c, address=BNO_ADDRESS)

        time.sleep(0.5)
        try:
            self.bno.soft_reset()
        except AttributeError:
            pass
        time.sleep(1.0)

        self._install_report_counter_hook()
        self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, IMU_INTERVAL_US)
        time.sleep(0.2)

        if self.enable_gyro:
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE, IMU_INTERVAL_US)
            time.sleep(0.2)

        time.sleep(0.5)
        self.report_count = 0
        self.report_timestamps.clear()
        self.value_change_count = 0
        self._last_quat = None

    def _install_report_counter_hook(self):
        if not hasattr(self.bno, "_process_report"):
            self._hook_installed = False
            return

        original_process_report = self.bno._process_report
        target_report_id = BNO_REPORT_GAME_ROTATION_VECTOR
        parent = self

        def counted_process_report(*args, **kwargs):
            if len(args) >= 1 and args[0] == target_report_id:
                parent.report_count += 1
                parent.report_timestamps.append(time.perf_counter())
            return original_process_report(*args, **kwargs)

        self.bno._process_report = counted_process_report
        self._hook_installed = True

    def read_orientation(self):
        t0 = time.perf_counter()
        reports_before = self.report_count
        try:
            quat = self.bno.game_quaternion
            imu_read_ms = (time.perf_counter() - t0) * 1000.0
            reports = self.report_count - reports_before

            if quat is not None:
                value_changed = int(quat != self._last_quat)
                if value_changed:
                    self.value_change_count += 1
                self._last_quat = quat
                pitch, yaw = quat_to_euler_pitch_yaw(*quat)
            else:
                value_changed = 0
                pitch, yaw = np.nan, np.nan

            return quat, pitch, yaw, reports, value_changed, imu_read_ms
        except Exception:
            imu_read_ms = (time.perf_counter() - t0) * 1000.0
            return None, np.nan, np.nan, 0, 0, imu_read_ms

    def read_gyro(self):
        if not self.enable_gyro:
            return np.nan, np.nan, np.nan, 0.0
        t0 = time.perf_counter()
        try:
            gyro = self.bno.gyro
            gyro_read_ms = (time.perf_counter() - t0) * 1000.0
            if gyro is None:
                return np.nan, np.nan, np.nan, gyro_read_ms
            gx, gy, gz = gyro
            return gx, gy, gz, gyro_read_ms
        except Exception:
            gyro_read_ms = (time.perf_counter() - t0) * 1000.0
            return np.nan, np.nan, np.nan, gyro_read_ms

    def report_rate_from_timestamps(self):
        if len(self.report_timestamps) < 2:
            return 0.0
        duration = self.report_timestamps[-1] - self.report_timestamps[0]
        if duration <= 0:
            return 0.0
        return (len(self.report_timestamps) - 1) / duration


class DummyPID:
    def __init__(self, kp=0.8, ki=0.3, kd=0.5, kg=0.85,
                 setpoint_deg=20.0, rest_angle_deg=85.0, throttle_limit=0.95):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kg = kg
        self.setpoint = setpoint_deg
        self.rest_angle = rest_angle_deg
        self.throttle_limit = throttle_limit
        self.integral = 0.0
        self.integral_max = 400.0

    def compute(self, psi_deg, psi_dot_dps):
        error = self.setpoint - psi_deg
        u_p = self.kp * error
        u_d = -self.kd * psi_dot_dps
        u_g = self.kg * math.sin(math.radians(self.rest_angle + self.setpoint)) * 90.0

        u_i_prev = self.ki * self.integral
        u_test_norm = (u_p + u_i_prev + u_d + u_g) / 90.0

        saturated_high = u_test_norm > self.throttle_limit and error > 0
        saturated_low = u_test_norm < -self.throttle_limit and error < 0
        if not (saturated_high or saturated_low):
            self.integral += error * LOOP_DT
            self.integral = max(-self.integral_max, min(self.integral_max, self.integral))

        u_i = self.ki * self.integral
        u_raw = u_p + u_i + u_d + u_g
        u_norm = max(-self.throttle_limit, min(self.throttle_limit, u_raw / 90.0))
        return u_norm, error, u_p / 90.0, u_i / 90.0, u_d / 90.0, u_g / 90.0

    def to_pwm(self, u_norm):
        if u_norm <= 0:
            return ESC_NEUTRAL_US
        span = ESC_NEUTRAL_US - ESC_DEADBAND_US - ESC_MIN_US
        pw = ESC_NEUTRAL_US - ESC_DEADBAND_US - u_norm * span
        return int(max(ESC_MIN_US, min(ESC_MAX_US, pw)))


def main():
    parser = argparse.ArgumentParser(description="Measure timing breakdown of a 200 Hz AP/TRMS loop")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--no-imu", action="store_true")
    parser.add_argument("--no-pwm", action="store_true")
    parser.add_argument("--enable-gyro", action="store_true")
    parser.add_argument("--send-active-pwm", action="store_true", help="Danger: sends computed PWM instead of neutral")
    parser.add_argument("--print-every", type=int, default=100)
    parser.add_argument("--outdir", default="logs")
    args = parser.parse_args()

    print("=" * 72)
    print("200 Hz CONTROL LOOP TIMING BREAKDOWN")
    print("=" * 72)
    print(f"Duration:        {args.duration:.1f} s")
    print(f"Loop target:     {CONTROL_FREQ_HZ:.1f} Hz")
    print(f"Target dt:       {LOOP_DT * 1000:.3f} ms")
    print(f"IMU enabled:     {not args.no_imu}")
    print(f"Gyro timed:      {args.enable_gyro and not args.no_imu}")
    print(f"PWM enabled:     {not args.no_pwm}")
    print(f"PWM mode:        {'ACTIVE computed PWM' if args.send_active_pwm else 'NEUTRAL 1500 us only'}")
    print("=" * 72)

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpiod non lance. Lance : sudo pigpiod")

    encoder = EncoderHEDS5540(pi)
    encoder.reset()

    imu = None
    if not args.no_imu:
        try:
            imu = BNO085TimingReader(enable_gyro=args.enable_gyro)
            print(f"[IMU] Initialised. Report hook installed: {imu._hook_installed}")
        except Exception as e:
            print(f"[IMU] ERROR: {e}")
            print("[IMU] Continuing without IMU")
            imu = None

    if not args.no_pwm:
        print(f"[PWM] Sending neutral {ESC_NEUTRAL_US} us on GPIO{ESC_MAIN_PWM_PIN}")
        pi.set_servo_pulsewidth(ESC_MAIN_PWM_PIN, ESC_NEUTRAL_US)
        time.sleep(3.0)

    pid = DummyPID()

    os.makedirs(args.outdir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.outdir, f"loop_timing_breakdown_{ts}.csv")

    fieldnames = [
        "t_s", "iteration", "dt_ms",
        "encoder_ms", "imu_ms", "gyro_ms", "pid_ms", "pwm_convert_ms", "pwm_send_ms",
        "logging_ms", "compute_total_ms", "sleep_margin_ms", "overrun",
        "imu_reports", "imu_value_changed",
        "angle_enc_deg", "psi_dot_enc_dps", "imu_pitch_deg", "imu_yaw_deg",
        "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s", "u_norm", "pwm_us",
    ]

    collected = {name: [] for name in [
        "dt_ms", "encoder_ms", "imu_ms", "gyro_ms", "pid_ms", "pwm_convert_ms",
        "pwm_send_ms", "logging_ms", "compute_total_ms", "sleep_margin_ms",
    ]}
    imu_report_counts = []
    imu_value_changes = []

    print(f"[LOG] CSV -> {csv_path}")
    print("[RUN] Starting timing test...\n")

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        t_start = time.perf_counter()
        t_prev = t_start
        next_tick = t_start + LOOP_DT
        it = 0

        try:
            while time.perf_counter() - t_start < args.duration:
                precise_sleep_until(next_tick)
                iter_start = time.perf_counter()
                it += 1
                dt_ms = (iter_start - t_prev) * 1000.0

                t0 = time.perf_counter()
                angle_enc, psi_dot_enc = encoder.get_psi_and_dot()
                encoder_ms = (time.perf_counter() - t0) * 1000.0

                imu_ms = 0.0
                imu_reports = 0
                imu_value_changed = 0
                imu_pitch = np.nan
                imu_yaw = np.nan
                if imu is not None:
                    quat, imu_pitch, imu_yaw, imu_reports, imu_value_changed, imu_ms = imu.read_orientation()
                imu_report_counts.append(imu_reports)
                imu_value_changes.append(imu_value_changed)

                if imu is not None and args.enable_gyro:
                    gx, gy, gz, gyro_ms = imu.read_gyro()
                else:
                    gx, gy, gz, gyro_ms = np.nan, np.nan, np.nan, 0.0

                t0 = time.perf_counter()
                u_norm, error, u_p, u_i, u_d, u_g = pid.compute(angle_enc, psi_dot_enc)
                pid_ms = (time.perf_counter() - t0) * 1000.0

                t0 = time.perf_counter()
                pwm_computed = pid.to_pwm(u_norm)
                pwm_to_send = pwm_computed if args.send_active_pwm else ESC_NEUTRAL_US
                pwm_convert_ms = (time.perf_counter() - t0) * 1000.0

                if not args.no_pwm:
                    t0 = time.perf_counter()
                    pi.set_servo_pulsewidth(ESC_MAIN_PWM_PIN, pwm_to_send)
                    pwm_send_ms = (time.perf_counter() - t0) * 1000.0
                else:
                    pwm_send_ms = 0.0

                before_log = time.perf_counter()
                compute_total_ms = (before_log - iter_start) * 1000.0

                row = {
                    "t_s": iter_start - t_start,
                    "iteration": it,
                    "dt_ms": dt_ms,
                    "encoder_ms": encoder_ms,
                    "imu_ms": imu_ms,
                    "gyro_ms": gyro_ms,
                    "pid_ms": pid_ms,
                    "pwm_convert_ms": pwm_convert_ms,
                    "pwm_send_ms": pwm_send_ms,
                    "logging_ms": np.nan,
                    "compute_total_ms": compute_total_ms,
                    "sleep_margin_ms": np.nan,
                    "overrun": 0,
                    "imu_reports": imu_reports,
                    "imu_value_changed": imu_value_changed,
                    "angle_enc_deg": angle_enc,
                    "psi_dot_enc_dps": psi_dot_enc,
                    "imu_pitch_deg": imu_pitch,
                    "imu_yaw_deg": imu_yaw,
                    "gyro_x_rad_s": gx,
                    "gyro_y_rad_s": gy,
                    "gyro_z_rad_s": gz,
                    "u_norm": u_norm,
                    "pwm_us": pwm_to_send,
                }

                t0 = time.perf_counter()
                writer.writerow(row)
                logging_ms = (time.perf_counter() - t0) * 1000.0

                iter_end = time.perf_counter()
                margin_ms = (next_tick + LOOP_DT - iter_end) * 1000.0
                overrun = int(margin_ms < 0.0)

                for k, v in {
                    "dt_ms": dt_ms,
                    "encoder_ms": encoder_ms,
                    "imu_ms": imu_ms,
                    "gyro_ms": gyro_ms,
                    "pid_ms": pid_ms,
                    "pwm_convert_ms": pwm_convert_ms,
                    "pwm_send_ms": pwm_send_ms,
                    "logging_ms": logging_ms,
                    "compute_total_ms": compute_total_ms,
                    "sleep_margin_ms": margin_ms,
                }.items():
                    collected[k].append(v)

                if args.print_every > 0 and it % args.print_every == 0:
                    print(
                        f"\r it={it:6d} | dt={dt_ms:6.3f} ms | "
                        f"enc={encoder_ms:5.3f} | imu={imu_ms:5.3f} | "
                        f"pid={pid_ms:5.3f} | pwm={pwm_send_ms:5.3f} | "
                        f"total={compute_total_ms:5.3f} | margin={margin_ms:6.3f} ms",
                        end="",
                        flush=True,
                    )

                t_prev = iter_start
                next_tick += LOOP_DT

        except KeyboardInterrupt:
            print("\n[STOP] Ctrl+C")

    print("\n")

    if not args.no_pwm:
        pi.set_servo_pulsewidth(ESC_MAIN_PWM_PIN, ESC_NEUTRAL_US)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(ESC_MAIN_PWM_PIN, 0)

    encoder.close()
    pi.stop()

    print("=" * 72)
    print("SUMMARY, after warmup")
    print("=" * 72)
    n_total = len(collected["dt_ms"])
    start = min(WARMUP_SAMPLES, n_total)
    analysed = n_total - start
    print(f"Iterations total:      {n_total}")
    print(f"Iterations analysed:   {analysed}")
    print(f"Target dt:             {LOOP_DT * 1000:.3f} ms")
    print(f"CSV saved:             {csv_path}\n")

    def print_stats(label, key):
        s = stats_ms(collected[key][start:])
        print(
            f"{label:<22s} mean={s['mean']:7.4f} ms | "
            f"p95={s['p95']:7.4f} | p99={s['p99']:7.4f} | max={s['max']:7.4f}"
        )

    print_stats("Loop dt", "dt_ms")
    print_stats("Encoder read", "encoder_ms")
    print_stats("IMU read", "imu_ms")
    if args.enable_gyro and imu is not None:
        print_stats("Gyro read", "gyro_ms")
    print_stats("PID compute", "pid_ms")
    print_stats("PWM conversion", "pwm_convert_ms")
    print_stats("PWM send pigpio", "pwm_send_ms")
    print_stats("CSV logging", "logging_ms")
    print_stats("Total compute", "compute_total_ms")
    print_stats("Sleep margin", "sleep_margin_ms")

    if analysed > 0:
        overrun_analysed = sum(1 for x in collected["sleep_margin_ms"][start:] if x < 0.0)
        print(f"\nOverruns after warmup: {overrun_analysed}/{analysed} ({100.0 * overrun_analysed / analysed:.3f}%)")

    if imu is not None:
        report_counts = np.asarray(imu_report_counts[start:], dtype=float)
        value_changes = np.asarray(imu_value_changes[start:], dtype=float)
        duration_s = sum(collected["dt_ms"][start:]) / 1000.0
        report_rate = report_counts.sum() / duration_s if duration_s > 0 else 0.0
        change_rate = value_changes.sum() / duration_s if duration_s > 0 else 0.0
        print("\n" + "=" * 72)
        print("IMU REPORT SUMMARY")
        print("=" * 72)
        print(f"IMU reports processed: {int(report_counts.sum())}")
        print(f"Effective report rate: {report_rate:.1f} Hz")
        print(f"Value-change rate:     {change_rate:.1f} Hz")
        print(f"Timestamp-based rate:  {imu.report_rate_from_timestamps():.1f} Hz")
        print(f"Report hook installed: {imu._hook_installed}")
        if report_counts.size:
            print(f"Polls with 0 reports:  {int((report_counts == 0).sum())}")
            print(f"Polls with 1 report:   {int((report_counts == 1).sum())}")
            print(f"Polls with >1 reports: {int((report_counts > 1).sum())}")

    print("\nInterpretation:")
    print("  - Encoder read = time to read decoded encoder state and compute filtered derivative.")
    print("  - IMU read = time spent inside bno.game_quaternion.")
    print("  - PID compute = pure controller calculation.")
    print("  - PWM send = time for pigpio set_servo_pulsewidth().")
    print("  - Total compute excludes CSV logging.")
    print("  - Sleep margin should stay positive for stable 200 Hz execution.")
    print("=" * 72)


if __name__ == "__main__":
    main()
