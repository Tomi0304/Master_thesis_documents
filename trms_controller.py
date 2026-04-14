#!/usr/bin/env python3
"""
TRMS — Controleur PID + compensation gravitationnelle + slew rate limiter
Rotor principal seul (GPIO 12), IMU BNO085

Le slew rate limiter empeche les transitions PWM trop rapides
qui declenchent le re-armement BLHeli_S bidirectionnel.

GPIO 12 : Main rotor (PWM0 hardware)
GPIO 13 : Tail rotor  (PWM1 hardware) — non utilise ici

Usage :
    sudo pigpiod
    python3 trms_controller.py --kp 1.5 --ki 0.20 --kd 0.4 --kg 0.25 --setpoint -20
"""

import time
import math
import signal
import argparse
import threading
import os

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
ESC_DEADBAND_US  = 25
ESC_SLEW_MAX_US  = 20   

CONTROL_FREQ_HZ  = 200.0
DT               = 1.0 / CONTROL_FREQ_HZ
MAX_PSI_DEG      = 90.0

CALIB_SAMPLES    = 50
CALIB_PERIOD_S   = 2.0

THROTTLE_LIMIT   = 0.60


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
        self.yaw_offset   = 0.0
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

    def _quat_to_euler(self, qi, qj, qk, qr):
        sinp  = 2.0 * (qr * qj - qk * qi)
        sinp  = max(-1.0, min(1.0, sinp))
        pitch = math.degrees(math.asin(sinp))
        siny  = 2.0 * (qr * qk + qi * qj)
        cosy  = 1.0 - 2.0 * (qj * qj + qk * qk)
        yaw   = math.degrees(math.atan2(siny, cosy))
        return pitch, yaw

    def _get_raw(self):
        try:
            quat = self.bno.quaternion
            if quat is None:
                return None, None
            qi, qj, qk, qr = quat
            return self._quat_to_euler(qi, qj, qk, qr)
        except Exception:
            return None, None

    def get_pitch_raw(self):
        pitch, _ = self._get_raw()
        return pitch

    def calibrate(self, n_samples=CALIB_SAMPLES, duration_s=CALIB_PERIOD_S):
        print(f"[IMU]  Calibration ({duration_s}s, {n_samples} echantillons)...")
        print("[IMU]  >>> BRAS IMMOBILE A SA POSITION DE REPOS <<<")
        samples_pitch, samples_yaw = [], []
        dt = duration_s / n_samples
        for _ in range(n_samples):
            pitch, yaw = self._get_raw()
            if pitch is not None:
                samples_pitch.append(pitch)
            if yaw is not None:
                samples_yaw.append(yaw)
            time.sleep(dt)
        if len(samples_pitch) < 5:
            print("[IMU]  ERREUR : echantillons insuffisants !")
            return False
        self.pitch_offset = sum(samples_pitch) / len(samples_pitch)
        self.yaw_offset   = sum(samples_yaw) / len(samples_yaw) if samples_yaw else 0.0
        spread = max(samples_pitch) - min(samples_pitch)
        print(f"[IMU]  pitch_offset = {self.pitch_offset:.2f} deg")
        print(f"[IMU]  yaw_offset   = {self.yaw_offset:.2f} deg")
        print(f"[IMU]  Dispersion   = {spread:.2f} deg ({len(samples_pitch)} echantillons)")
        if spread > 3.0:
            print("[IMU]  ATTENTION : dispersion elevee — bras en mouvement ?")
        return True

    def get_rest_angle_from_vertical(self):
        return 90.0 - abs(self.pitch_offset)

    def get_psi(self):
        pitch, _ = self._get_raw()
        if pitch is None:
            return None
        return pitch - self.pitch_offset

    def get_phi(self):
        _, yaw = self._get_raw()
        if yaw is None:
            return None
        delta = yaw - self.yaw_offset
        if delta >  180.0: delta -= 360.0
        if delta < -180.0: delta += 360.0
        return delta

    def get_gyro_pitch_rate(self):
        try:
            gyro = self.bno.gyro
            if gyro is None:
                return 0.0
            _, gy, _ = gyro
            return gy
        except Exception:
            return 0.0


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
        print(f"[ENC]  HEDS-5540 (A=GPIO{ENCODER_PIN_A}, B=GPIO{ENCODER_PIN_B}) pret")

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

    def __init__(self, pi, pin, name="ESC"):
        self.pi         = pi
        self.pin        = pin
        self.name       = name
        self._pwm_cur   = ESC_NEUTRAL_US
        print(f"[{name}]  Armement ({ESC_NEUTRAL_US} us sur GPIO{pin})...")
        pi.set_servo_pulsewidth(pin, ESC_NEUTRAL_US)
        time.sleep(3)
        print(f"[{name}]  Arme et pret")

    def set_pwm(self, target_us):
        """Applique le PWM avec slew rate limiter."""
        target_us = int(max(ESC_MIN_US, min(ESC_MAX_US, target_us)))
        delta = target_us - self._pwm_cur
        delta = max(-ESC_SLEW_MAX_US, min(ESC_SLEW_MAX_US, delta))
        self._pwm_cur = int(self._pwm_cur + delta)
        self.pi.set_servo_pulsewidth(self.pin, self._pwm_cur)
        return self._pwm_cur

    def stop(self):
        """Retour au neutre avec slew."""
        while self._pwm_cur != ESC_NEUTRAL_US:
            delta = ESC_NEUTRAL_US - self._pwm_cur
            delta = max(-ESC_SLEW_MAX_US, min(ESC_SLEW_MAX_US, delta))
            self._pwm_cur = int(self._pwm_cur + delta)
            self.pi.set_servo_pulsewidth(self.pin, self._pwm_cur)
            time.sleep(DT)

    def shutdown(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)


class PIDController:

    def __init__(self, kp, ki, kd, kg, setpoint_deg, rest_angle_deg=80.0):
        self.kp         = kp
        self.ki         = ki
        self.kd         = kd
        self.kg         = kg
        self.setpoint   = setpoint_deg
        self.rest_angle = rest_angle_deg
        self._integral     = 0.0
        self._integral_max = 120.0

    def reset_integral(self):
        self._integral = 0.0

    def compute(self, psi_deg, psi_dot_dps):
        error = self.setpoint - psi_deg

        u_p = self.kp * error

        self._integral += error * DT
        self._integral = max(-self._integral_max, min(self._integral_max, self._integral))
        u_i = self.ki * self._integral

        u_d = -self.kd * psi_dot_dps

        phys_angle_rad = math.radians(self.rest_angle + self.setpoint)
        u_g = self.kg * math.sin(phys_angle_rad) * 90.0

        u_raw  = u_p + u_i + u_d + u_g
        u_norm = max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, u_raw / 90.0))

        # Anti-windup back-calculation
        if abs(u_raw / 90.0) > THROTTLE_LIMIT:
            self._integral -= error * DT

        return u_norm, error, u_p / 90.0, u_i / 90.0, u_d / 90.0, u_g / 90.0

    def to_pwm(self, u_norm):
        u_esc = -u_norm
        if u_esc > 0:
            pw = (ESC_NEUTRAL_US + ESC_DEADBAND_US +
                  u_esc * (ESC_MAX_US - ESC_NEUTRAL_US - ESC_DEADBAND_US))
        else:
            pw = (ESC_NEUTRAL_US - ESC_DEADBAND_US +
                  u_esc * (ESC_NEUTRAL_US - ESC_DEADBAND_US - ESC_MIN_US))
        return int(max(ESC_MIN_US, min(ESC_MAX_US, pw)))


class TRMSController:

    def __init__(self, kp, ki, kd, kg, setpoint_deg, rest_angle_override=None):
        self.running = False
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod non lance ! → sudo pigpiod")

        print("=" * 66)
        print("  TRMS — Controleur PID + Gravite + Slew Rate Limiter")
        print(f"  Consigne psi* = {setpoint_deg:.1f} deg")
        print(f"  Kp={kp:.2f}  Ki={ki:.3f}  Kd={kd:.2f}  Kg={kg:.2f}")
        print(f"  Throttle max  = {THROTTLE_LIMIT*100:.0f}%")
        print(f"  Slew rate max = {ESC_SLEW_MAX_US} us/iter ({ESC_SLEW_MAX_US*CONTROL_FREQ_HZ:.0f} us/s)")
        print("=" * 66)

        self.imu     = IMU_BNO085()
        self.encoder = EncoderHEDS5540(self.pi)
        self.esc     = ESC(self.pi, ESC_MAIN_PWM_PIN, name="ESC_MAIN")
        time.sleep(1.0)

        print()
        print("  *** CALIBRATION : bras immobile a sa position de repos ***")
        print()
        if not self.imu.calibrate():
            self._abort()
            raise RuntimeError("Echec calibration IMU")

        if rest_angle_override is not None:
            rest_angle = rest_angle_override
            print(f"[CAL]  Angle de repos (manuel)  = {rest_angle:.1f} deg depuis verticale")
        else:
            rest_angle = self.imu.get_rest_angle_from_vertical()
            print(f"[CAL]  Angle de repos (IMU auto) = {rest_angle:.1f} deg depuis verticale")

        phys_at_sp  = rest_angle + setpoint_deg
        u_g_preview = kg * math.sin(math.radians(phys_at_sp))
        print(f"[CAL]  Angle physique au setpoint = {phys_at_sp:.1f} deg")
        print(f"[CAL]  u_G prevu au setpoint      = {u_g_preview:.3f} (norme)")

        self.ctrl = PIDController(kp, ki, kd, kg, setpoint_deg, rest_angle_deg=rest_angle)

        psi_check = self.imu.get_psi()
        if psi_check is not None:
            print(f"[CAL]  psi post-calib = {psi_check:.2f} deg (attendu ~ 0)")
            print("[CAL]  OK" if abs(psi_check) <= 10.0 else "[CAL]  ATTENTION : ecart important")

        self.encoder.reset()
        print("[ENC]  Position remise a zero")

        from datetime import datetime
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = (
            f"logs/trms_kp{kp}_ki{ki}_kd{kd}_kg{kg}"
            f"_sp{setpoint_deg}_ra{rest_angle:.0f}"
            f"_thr{THROTTLE_LIMIT}_{ts}.csv"
        )
        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write(
            "t_s,iteration,"
            "psi_imu_deg,psi_enc_deg,phi_yaw_deg,psi_dot_dps,"
            "setpoint_deg,rest_angle_deg,phys_angle_deg,"
            "error_deg,integral_deg_s,"
            "u_norm,u_p,u_i,u_d,u_g,pwm_us,pitch_raw_deg\n"
        )
        print(f"[LOG]  CSV → {self.csv_path}")

        signal.signal(signal.SIGINT,  self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        print()
        print("=" * 66)
        print("  Systeme pret — Ctrl+C pour arreter")
        print("=" * 66)

    def _sig(self, signum, frame):
        print("\n[STOP] Arret demande...")
        self.running = False

    def _abort(self):
        self.esc.stop()
        self.encoder.cancel()
        self.pi.stop()

    def _emergency(self, reason):
        self.running = False
        self.esc.stop()
        print(f"\n[!!!] ARRET D'URGENCE : {reason}")

    def run(self):
        self.running = True
        hdr = (f"{'#':>5} | {'psi':>7} | {'enc':>7} | {'phi':>7} | "
               f"{'psi_d':>7} | {'Err':>7} | {'I':>7} | {'u':>7} | {'PWM':>5}")
        print(f"\n{hdr}")
        print("-" * len(hdr))

        it      = 0
        t_start = time.monotonic()

        try:
            while self.running:
                t0 = time.monotonic()
                it += 1

                psi = self.imu.get_psi()
                if psi is None:
                    time.sleep(DT)
                    continue

                if abs(psi) > MAX_PSI_DEG:
                    self._emergency(f"|psi| = {abs(psi):.1f} > {MAX_PSI_DEG}")
                    break

                phi         = self.imu.get_phi()
                psi_enc     = self.encoder.get_psi()
                psi_dot_dps = math.degrees(self.imu.get_gyro_pitch_rate())

                u, err, u_p, u_i, u_d, u_g = self.ctrl.compute(psi, psi_dot_dps)
                pwm_cmd    = self.ctrl.to_pwm(u)
                pwm_actual = self.esc.set_pwm(pwm_cmd)

                t_elapsed  = t0 - t_start
                pitch_raw  = self.imu.get_pitch_raw()
                phys_angle = self.ctrl.rest_angle + self.ctrl.setpoint

                phi_log = str(round(phi, 4))       if phi       is not None else ""
                pr_log  = str(round(pitch_raw, 4)) if pitch_raw is not None else ""

                self.csv_file.write(
                    f"{t_elapsed:.4f},{it},"
                    f"{psi:.4f},{psi_enc:.4f},"
                    f"{phi_log},"
                    f"{psi_dot_dps:.4f},"
                    f"{self.ctrl.setpoint:.2f},{self.ctrl.rest_angle:.2f},{phys_angle:.2f},"
                    f"{err:.4f},{self.ctrl._integral:.4f},"
                    f"{u:.6f},{u_p:.6f},{u_i:.6f},{u_d:.6f},{u_g:.6f},"
                    f"{pwm_actual},{pr_log}\n"
                )

                phi_str = str(round(phi, 1)) if phi is not None else "  N/A"
                print(
                    f"\r{it:5d} | {psi:+7.1f} | {psi_enc:+7.1f} | {phi_str:>7} | "
                    f"{psi_dot_dps:+7.1f} | {err:+7.1f} | {self.ctrl._integral:+7.2f} | "
                    f"{u:+7.3f} | {pwm_actual:5d}",
                    end="", flush=True
                )

                elapsed = time.monotonic() - t0
                wait = DT - elapsed
                if wait > 0:
                    time.sleep(wait)

        except KeyboardInterrupt:
            print("\n[STOP] Ctrl+C")

        finally:
            self._cleanup()

    def _cleanup(self):
        print("\n[STOP] Arret moteur (slew vers neutre)...")
        self.esc.stop()
        self.esc.shutdown()
        self.encoder.cancel()
        self.pi.stop()
        if hasattr(self, "csv_file") and self.csv_file:
            self.csv_file.close()
            print(f"[LOG]  CSV sauvegarde → {self.csv_path}")
        print("[STOP] Systeme arrete proprement")


def main():
    p = argparse.ArgumentParser(
        description="TRMS — Controleur PID + gravite + slew rate limiter",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples :
  python3 trms_controller.py --kp 1.5 --ki 0.20 --kd 0.4 --kg 0.25 --setpoint -20
  python3 trms_controller.py --kp 1.5 --ki 0.10 --kd 0.4 --kg 0.25 --setpoint 6
        """)

    p.add_argument("--setpoint",   type=float, default=6.0)
    p.add_argument("--kp",         type=float, default=1.5)
    p.add_argument("--ki",         type=float, default=0.10)
    p.add_argument("--kd",         type=float, default=0.4)
    p.add_argument("--kg",         type=float, default=0.25)
    p.add_argument("--throttle",   type=float, default=0.55)
    p.add_argument("--rest-angle", type=float, default=None,
                   help="Angle de repos depuis verticale [deg] — si absent, estime par IMU")

    args = p.parse_args()

    global THROTTLE_LIMIT
    THROTTLE_LIMIT = max(0.05, min(1.0, args.throttle))

    ctrl = TRMSController(
        kp                  = args.kp,
        ki                  = args.ki,
        kd                  = args.kd,
        kg                  = args.kg,
        setpoint_deg        = args.setpoint,
        rest_angle_override = args.rest_angle,
    )
    ctrl.run()


if __name__ == "__main__":
    main()