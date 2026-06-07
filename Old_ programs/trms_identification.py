#!/usr/bin/env python3
"""
TRMS — Identification en boucle ouverte
Applique une sequence de PWM et enregistre la reponse psi(t)

Protocole :
  1. Repos (neutre) pendant T_repos secondes
  2. Echelon PWM pendant T_step secondes
  3. Retour neutre (avec slew) pendant T_repos secondes

Repete N_steps echelons de valeurs differentes.

Usage :
    sudo pigpiod
    python3 trms_identification.py --pwm-steps 1600 1650 1700 --step-duration 4
"""

import time
import math
import signal
import argparse
import threading
import os
import sys

import pigpio
import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

ENCODER_PIN_A    = 27
ENCODER_PIN_B    = 17
ESC_MAIN_PWM_PIN = 12
ENCODER_CPR_X4   = 2000
ESC_NEUTRAL_US   = 1500
ESC_MIN_US       = 1100
ESC_MAX_US       = 1900
ESC_SLEW_MAX_US  = 25
CONTROL_FREQ_HZ  = 50.0
DT               = 1.0 / CONTROL_FREQ_HZ
MAX_PSI_DEG      = 85.0
CALIB_SAMPLES    = 50
CALIB_PERIOD_S   = 2.0


class IMU_BNO085:

    def __init__(self):
        print("[IMU]  Initialisation BNO085 sur I2C @ 0x4A...")
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(self.i2c, address=0x4A)
        time.sleep(0.5)
        try:
            self.bno.soft_reset()
        except AttributeError:
            pass
        time.sleep(1.0)
        self._enable_feature_safe(BNO_REPORT_ROTATION_VECTOR)
        time.sleep(0.2)
        self._enable_feature_safe(BNO_REPORT_GYROSCOPE)
        time.sleep(0.5)
        self.pitch_offset = 0.0
        print("[IMU]  BNO085 pret")

    def _enable_feature_safe(self, feature, retries=5, delay=0.5):
        for attempt in range(1, retries + 1):
            try:
                self.bno.enable_feature(feature)
                return
            except RuntimeError as e:
                print(f"[IMU]  Tentative {attempt}/{retries} : {e}")
                if attempt < retries:
                    try:
                        self.bno.soft_reset()
                    except AttributeError:
                        pass
                    time.sleep(1.0)
        raise RuntimeError(f"[IMU]  Feature {feature} impossible a activer")

    def _quat_to_pitch(self, qi, qj, qk, qr):
        sinp = 2.0 * (qr * qj - qk * qi)
        sinp = max(-1.0, min(1.0, sinp))
        return math.degrees(math.asin(sinp))

    def _get_pitch_raw(self):
        try:
            quat = self.bno.quaternion
            if quat is None:
                return None
            return self._quat_to_pitch(*quat)
        except Exception:
            return None

    def get_gyro_y(self):
        try:
            gyro = self.bno.gyro
            if gyro is None:
                return 0.0
            return gyro[1]
        except Exception:
            return 0.0

    def calibrate(self, n_samples=CALIB_SAMPLES, duration_s=CALIB_PERIOD_S):
        print(f"[IMU]  Calibration ({duration_s}s)...")
        print("[IMU]  >>> BRAS IMMOBILE AU REPOS <<<")
        samples = []
        dt = duration_s / n_samples
        for _ in range(n_samples):
            v = self._get_pitch_raw()
            if v is not None:
                samples.append(v)
            time.sleep(dt)
        if len(samples) < 5:
            return False
        self.pitch_offset = sum(samples) / len(samples)
        spread = max(samples) - min(samples)
        rest_angle = 90.0 - abs(self.pitch_offset)
        print(f"[IMU]  pitch_offset = {self.pitch_offset:.2f} deg")
        print(f"[IMU]  alpha_rest   = {rest_angle:.1f} deg depuis verticale")
        print(f"[IMU]  Dispersion   = {spread:.2f} deg")
        return True

    def get_psi(self):
        v = self._get_pitch_raw()
        if v is None:
            return None
        return -(v - self.pitch_offset)

    def get_psi_dot(self):
        return -math.degrees(self.get_gyro_y())

    def get_pitch_raw(self):
        return self._get_pitch_raw()

    def get_rest_angle(self):
        return 90.0 - abs(self.pitch_offset)


class EncoderHEDS5540:

    _QUAD_TABLE = [
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    ]

    def __init__(self, pi):
        self.pi        = pi
        self._position = 0
        self._lock     = threading.Lock()
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

    def get_psi(self):
        return -((self.position / ENCODER_CPR_X4) * 360.0)

    def reset(self):
        with self._lock:
            self._position = 0

    def cancel(self):
        self._cb_a.cancel()
        self._cb_b.cancel()


class ESC:

    def __init__(self, pi, pin):
        self.pi       = pi
        self.pin      = pin
        self._cur     = ESC_NEUTRAL_US
        print(f"[ESC]  Armement GPIO{pin}...")
        pi.set_servo_pulsewidth(pin, ESC_NEUTRAL_US)
        time.sleep(3)
        print("[ESC]  Arme")

    def set_pwm(self, target):
        target = int(max(ESC_MIN_US, min(ESC_MAX_US, target)))
        delta  = max(-ESC_SLEW_MAX_US, min(ESC_SLEW_MAX_US, target - self._cur))
        self._cur += delta
        self.pi.set_servo_pulsewidth(self.pin, self._cur)
        return self._cur

    def ramp_to(self, target):
        """Rampe vers une valeur cible en bloquant (pour transitions entre phases)."""
        target = int(max(ESC_MIN_US, min(ESC_MAX_US, target)))
        while self._cur != target:
            delta = max(-ESC_SLEW_MAX_US, min(ESC_SLEW_MAX_US, target - self._cur))
            self._cur += delta
            self.pi.set_servo_pulsewidth(self.pin, self._cur)
            time.sleep(DT)

    def shutdown(self):
        self.ramp_to(ESC_NEUTRAL_US)
        time.sleep(0.3)
        self.pi.set_servo_pulsewidth(self.pin, 0)


def run_identification(pwm_steps, t_repos, t_step, output_path):
    pi = pigpio.pi()
    if not pi.connected:
        print("ERREUR : pigpiod non lance")
        sys.exit(1)

    imu     = IMU_BNO085()
    encoder = EncoderHEDS5540(pi)
    esc     = ESC(pi, ESC_MAIN_PWM_PIN)
    time.sleep(1.0)

    if not imu.calibrate():
        esc.shutdown()
        encoder.cancel()
        pi.stop()
        sys.exit(1)

    encoder.reset()
    rest_angle = imu.get_rest_angle()
    print(f"[ID]   alpha_rest = {rest_angle:.1f} deg")
    print(f"[ID]   Protocole : {len(pwm_steps)} echelons x {t_step}s + {t_repos}s repos")
    print(f"[ID]   Duree totale estimee : {len(pwm_steps)*(t_step+t_repos):.0f}s")
    print(f"[ID]   Fichier : {output_path}")
    print()

    running = [True]

    def sig_handler(s, f):
        print("\n[ID]   Interruption...")
        running[0] = False

    signal.signal(signal.SIGINT, sig_handler)

    csv_file = open(output_path, "w")
    csv_file.write(
        "t_s,phase,step_idx,pwm_target,pwm_actual,"
        "psi_imu_deg,psi_enc_deg,psi_dot_dps,"
        "pitch_raw_deg,rest_angle_deg\n"
    )

    t_start = time.monotonic()

    try:
        for step_idx, pwm_target in enumerate(pwm_steps):
            if not running[0]:
                break

            print(f"[ID]   === Echelon {step_idx+1}/{len(pwm_steps)} : PWM cible = {pwm_target} us ===")

            # ── Phase REPOS ──
            print(f"[ID]   Phase repos ({t_repos}s)...")
            esc.ramp_to(ESC_NEUTRAL_US)
            t_phase = time.monotonic()
            while running[0] and (time.monotonic() - t_phase) < t_repos:
                t0  = time.monotonic()
                psi = imu.get_psi()
                if psi is None:
                    time.sleep(DT)
                    continue
                if abs(psi) > MAX_PSI_DEG:
                    print(f"\n[!!!] SECURITE : |psi|={abs(psi):.1f} > {MAX_PSI_DEG}")
                    running[0] = False
                    break
                psi_enc = encoder.get_psi()
                psi_dot = imu.get_psi_dot()
                pr      = imu.get_pitch_raw()
                pw_act  = esc.set_pwm(ESC_NEUTRAL_US)
                t_now   = time.monotonic() - t_start
                pr_log  = str(round(pr, 4)) if pr is not None else ""
                csv_file.write(
                    f"{t_now:.4f},repos,{step_idx},{ESC_NEUTRAL_US},{pw_act},"
                    f"{psi:.4f},{psi_enc:.4f},{psi_dot:.4f},"
                    f"{pr_log},{rest_angle:.2f}\n"
                )
                print(f"\r  repos | psi={psi:+6.1f} enc={psi_enc:+6.1f} pwm={pw_act}",
                      end="", flush=True)
                wait = DT - (time.monotonic() - t0)
                if wait > 0:
                    time.sleep(wait)
            print()

            if not running[0]:
                break

            # ── Phase ECHELON ──
            print(f"[ID]   Phase echelon ({t_step}s) → PWM={pwm_target}...")
            t_phase = time.monotonic()
            while running[0] and (time.monotonic() - t_phase) < t_step:
                t0  = time.monotonic()
                psi = imu.get_psi()
                if psi is None:
                    time.sleep(DT)
                    continue
                if abs(psi) > MAX_PSI_DEG:
                    print(f"\n[!!!] SECURITE : |psi|={abs(psi):.1f} > {MAX_PSI_DEG}")
                    running[0] = False
                    break
                psi_enc = encoder.get_psi()
                psi_dot = imu.get_psi_dot()
                pr      = imu.get_pitch_raw()
                pw_act  = esc.set_pwm(pwm_target)
                t_now   = time.monotonic() - t_start
                pr_log  = str(round(pr, 4)) if pr is not None else ""
                csv_file.write(
                    f"{t_now:.4f},step,{step_idx},{pwm_target},{pw_act},"
                    f"{psi:.4f},{psi_enc:.4f},{psi_dot:.4f},"
                    f"{pr_log},{rest_angle:.2f}\n"
                )
                print(f"\r  step  | psi={psi:+6.1f} enc={psi_enc:+6.1f} pwm={pw_act}",
                      end="", flush=True)
                wait = DT - (time.monotonic() - t0)
                if wait > 0:
                    time.sleep(wait)
            print()

        # Repos final
        print("[ID]   Repos final...")
        esc.ramp_to(ESC_NEUTRAL_US)
        time.sleep(1.0)

    finally:
        esc.shutdown()
        encoder.cancel()
        pi.stop()
        csv_file.close()
        print(f"\n[ID]   CSV sauvegarde → {output_path}")
        print("[ID]   Termine proprement")


def main():
    p = argparse.ArgumentParser(
        description="TRMS — Identification en boucle ouverte",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples :
  # 3 echelons : 1600, 1650, 1700 us pendant 4s chacun
  python3 trms_identification.py --pwm-steps 1600 1650 1700 --step-duration 4

  # Echelon unique pour mesurer la dynamique
  python3 trms_identification.py --pwm-steps 1650 --step-duration 6 --rest-duration 3

  # Sens inverse (freinage / descente)
  python3 trms_identification.py --pwm-steps 1350 1400 1450 --step-duration 4
        """)

    p.add_argument("--pwm-steps",     type=int, nargs="+", default=[1600, 1650, 1700],
                   help="Valeurs PWM des echelons [us]")
    p.add_argument("--step-duration", type=float, default=4.0,
                   help="Duree de chaque echelon [s] (defaut: 4)")
    p.add_argument("--rest-duration", type=float, default=3.0,
                   help="Duree du repos entre echelons [s] (defaut: 3)")

    args = p.parse_args()

    from datetime import datetime
    os.makedirs("logs", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    steps_str = "_".join(str(s) for s in args.pwm_steps)
    output = f"logs/ident_steps{steps_str}_dur{args.step_duration:.0f}s_{ts}.csv"

    print("=" * 60)
    print("  TRMS — Identification boucle ouverte")
    print(f"  Echelons PWM   : {args.pwm_steps}")
    print(f"  Duree echelon  : {args.step_duration}s")
    print(f"  Duree repos    : {args.rest_duration}s")
    print(f"  Slew rate max  : {ESC_SLEW_MAX_US} us/iter")
    print("=" * 60)

    run_identification(
        pwm_steps   = args.pwm_steps,
        t_repos     = args.rest_duration,
        t_step      = args.step_duration,
        output_path = output,
    )


if __name__ == "__main__":
    main()