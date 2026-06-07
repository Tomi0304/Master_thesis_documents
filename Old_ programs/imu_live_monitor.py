#!/usr/bin/env python3
"""
imu_live_monitor.py

Affiche en direct les donnees du BNO085 IMU.

Utile pour :
  - Verifier l'orientation de l'IMU avant un test
  - Observer l'angle de repos d'un prototype (AP, SATR, TRMS)
  - Diagnostiquer un probleme de calibration
  - Voir le bruit du capteur en statique

Affiche :
  - Pitch (deg)
  - Yaw   (deg)
  - Roll  (deg)
  - Gyro pitch (deg/s)
  - Quaternion brut (qi, qj, qk, qr)
  - Taux de rafraichissement effectif (Hz)

Usage:
    python3 imu_live_monitor.py [--rate 200] [--log]

Ctrl+C pour quitter.
"""

import time
import math
import signal
import argparse
import os
from datetime import datetime

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ACCELEROMETER,
)
from adafruit_bno08x.i2c import BNO08X_I2C


IMU_I2C_ADDR = 0x4A


class IMU_BNO085:

    def __init__(self, interval_us=5000, enable_gyro=True, enable_accel=False):
        print("[IMU]  Initialisation BNO085 sur I2C @ 0x4A...")
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        self.bno = BNO08X_I2C(self.i2c, address=IMU_I2C_ADDR)
        time.sleep(0.5)
        try:
            self.bno.soft_reset()
        except AttributeError:
            pass
        time.sleep(1.0)

        self._enable_feature_safe(BNO_REPORT_GAME_ROTATION_VECTOR,
                                  interval_us=interval_us)
        time.sleep(0.2)
        if enable_gyro:
            self._enable_feature_safe(BNO_REPORT_GYROSCOPE,
                                      interval_us=interval_us)
            print(f"[IMU]  Gyroscope active @ {1e6/interval_us:.0f} Hz")
        if enable_accel:
            self._enable_feature_safe(BNO_REPORT_ACCELEROMETER,
                                      interval_us=interval_us)
            print(f"[IMU]  Accelerometer active @ {1e6/interval_us:.0f} Hz")
        time.sleep(0.5)

        self._last_quat = None
        self._quat_changed = False
        self._has_data = False

        print(f"[IMU]  BNO085 pret @ {1e6/interval_us:.0f} Hz")

    def _enable_feature_safe(self, feature, retries=5, delay=0.5,
                             interval_us=5000):
        for attempt in range(1, retries + 1):
            try:
                try:
                    self.bno.enable_feature(feature, interval_us)
                except TypeError:
                    self.bno.enable_feature(feature)
                return
            except RuntimeError as e:
                print(f"[IMU]  Tentative {attempt}/{retries} : {e}")
                if attempt < retries:
                    try:
                        self.bno.soft_reset()
                    except AttributeError:
                        pass
                    time.sleep(delay)
        raise RuntimeError(f"[IMU]  Feature {feature} impossible a activer")

    def _quat_to_euler(self, qi, qj, qk, qr):
        sinr_cosp = 2.0 * (qr * qi + qj * qk)
        cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
        roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2.0 * (qr * qj - qk * qi)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.degrees(math.asin(sinp))

        siny_cosp = 2.0 * (qr * qk + qi * qj)
        cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
        yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        return roll, pitch, yaw

    def read(self):
        try:
            quat = self.bno.game_quaternion
            if quat is None:
                self._has_data = False
                return None

            self._has_data = True
            if self._last_quat is None:
                self._quat_changed = True
            else:
                self._quat_changed = (quat != self._last_quat)
            self._last_quat = quat

            qi, qj, qk, qr = quat
            roll, pitch, yaw = self._quat_to_euler(qi, qj, qk, qr)

            try:
                gx, gy, gz = self.bno.gyro
                gx_dps = math.degrees(gx)
                gy_dps = math.degrees(gy)
                gz_dps = math.degrees(gz)
            except Exception:
                gx_dps = gy_dps = gz_dps = 0.0

            return {
                'quat': (qi, qj, qk, qr),
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'gx': gx_dps,
                'gy': gy_dps,
                'gz': gz_dps,
                'changed': self._quat_changed,
            }

        except Exception:
            self._has_data = False
            return None


def main():
    p = argparse.ArgumentParser(description="IMU BNO085 live monitor")
    p.add_argument("--rate", type=int, default=200,
                   help="Read rate in Hz (default: 200)")
    p.add_argument("--log", action="store_true",
                   help="Log to CSV file")
    p.add_argument("--no-gyro", action="store_true",
                   help="Disable gyroscope feature")
    args = p.parse_args()

    interval_us = int(1e6 / args.rate)
    dt = 1.0 / args.rate

    imu = IMU_BNO085(interval_us=interval_us, enable_gyro=not args.no_gyro)

    csv_file = None
    csv_path = None
    if args.log:
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = f"logs/imu_monitor_{ts}.csv"
        csv_file = open(csv_path, "w")
        csv_file.write("t_s,qi,qj,qk,qr,roll_deg,pitch_deg,yaw_deg,"
                       "gx_dps,gy_dps,gz_dps\n")
        print(f"[LOG]  CSV -> {csv_path}")

    stop_flag = {'stop': False}

    def sig_handler(signum, frame):
        stop_flag['stop'] = True
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    print("\n" + "=" * 75)
    print("  IMU LIVE MONITOR - Ctrl+C pour quitter")
    print("=" * 75)
    print()
    header = (
        f"{'Roll':>8} {'Pitch':>8} {'Yaw':>8}  | "
        f"{'gx':>7} {'gy':>7} {'gz':>7}  | "
        f"{'qi':>6} {'qj':>6} {'qk':>6} {'qr':>6}  | "
        f"{'rate':>5}"
    )
    print(header)
    print("-" * len(header))

    t_start = time.monotonic()
    sample_count = 0
    valid_count = 0
    last_print = time.monotonic()
    rate_window_start = time.monotonic()
    rate_window_count = 0
    effective_rate = 0.0

    try:
        while not stop_flag['stop']:
            t0 = time.monotonic()
            sample_count += 1

            data = imu.read()

            if data is not None:
                valid_count += 1
                rate_window_count += 1

                now = time.monotonic()
                if now - rate_window_start >= 1.0:
                    effective_rate = rate_window_count / (now - rate_window_start)
                    rate_window_start = now
                    rate_window_count = 0

                if now - last_print >= 0.1:
                    qi, qj, qk, qr = data['quat']
                    line = (
                        f"{data['roll']:+8.2f} "
                        f"{data['pitch']:+8.2f} "
                        f"{data['yaw']:+8.2f}  | "
                        f"{data['gx']:+7.1f} "
                        f"{data['gy']:+7.1f} "
                        f"{data['gz']:+7.1f}  | "
                        f"{qi:+6.3f} {qj:+6.3f} {qk:+6.3f} {qr:+6.3f}  | "
                        f"{effective_rate:5.1f}"
                    )
                    print(f"\r{line}", end="", flush=True)
                    last_print = now

                if csv_file is not None:
                    t_elapsed = t0 - t_start
                    qi, qj, qk, qr = data['quat']
                    csv_file.write(
                        f"{t_elapsed:.4f},{qi:.6f},{qj:.6f},{qk:.6f},{qr:.6f},"
                        f"{data['roll']:.4f},{data['pitch']:.4f},{data['yaw']:.4f},"
                        f"{data['gx']:.4f},{data['gy']:.4f},{data['gz']:.4f}\n"
                    )

            elapsed = time.monotonic() - t0
            wait = dt - elapsed
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        pass
    finally:
        t_total = time.monotonic() - t_start
        print(f"\n\n[STOP] Duree: {t_total:.2f}s, samples lus: {sample_count}, "
              f"valides: {valid_count} ({100*valid_count/max(1,sample_count):.0f}%)")
        if csv_file is not None:
            csv_file.close()
            print(f"[LOG]  CSV -> {csv_path}")


if __name__ == "__main__":
    main()