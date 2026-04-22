#!/usr/bin/env python3
"""
TRMS — Controleur PID + compensation gravitationnelle
Rotor principal seul (GPIO 12)

Architecture capteurs :
  - Encodeur HEDS-5540 : source principale pour psi et psi_dot (200 Hz, frais a chaque iter)
  - IMU BNO085 : calibration initiale uniquement (alpha_rest) + logging yaw phi
  - psi_dot derive numeriquement depuis encodeur + filtre passe-bas

IMU orientee a 180deg autour de Z :
  - pitch_raw = -85 deg au repos, monte vers 0 en montant
  - get_rest_angle = -pitch_offset = 85 deg depuis verticale

Slew rate ESC : 20 us/iter @ 200 Hz = 4000 us/s
Dwell ESC : 400 ms au neutre avant changement de sens

GPIO 12 : Main rotor (PWM0)
GPIO 13 : Tail rotor  (PWM1) — non utilise
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
ESC_DWELL_S      = 0.1

CONTROL_FREQ_HZ  = 200.0
DT               = 1.0 / CONTROL_FREQ_HZ
MAX_PSI_DEG      = 100.0

CALIB_SAMPLES    = 50
CALIB_PERIOD_S   = 2.0

# Filtre passe-bas sur psi_dot derive encodeur
# alpha = 0.3 a 200 Hz → coupure ~10 Hz
PSI_DOT_ALPHA    = 0.3
THROTTLE_LIMIT   = 0.60

class IMU_BNO085:
    """
    Utilisee uniquement pour :
    1. Calibration initiale -> pitch_offset -> alpha_rest
    2. Logging yaw phi (non controle)
    """

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
        time.sleep(0.5)
        self.pitch_offset = 0.0
        self.yaw_offset   = 0.0
        print("[IMU]  BNO085 pret (calibration + yaw logging)")

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
                    time.sleep(delay)
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
            return self._quat_to_euler(*quat)
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
        print(f"[IMU]  Dispersion   = {spread:.2f} deg")
        if spread > 3.0:
            print("[IMU]  ATTENTION : dispersion elevee — bras en mouvement ?")
        return True

    def get_rest_angle_from_vertical(self):
        """
        IMU orientee 180 deg autour de Z :
          pitch_offset ≈ -85 deg au repos
          → alpha_rest = -pitch_offset = 85 deg depuis verticale
        """
        return -self.pitch_offset

    def get_phi(self):
        """Yaw φ pour logging uniquement."""
        _, yaw = self._get_raw()
        if yaw is None:
            return None
        delta = yaw - self.yaw_offset
        if delta >  180.0: delta -= 360.0
        if delta < -180.0: delta += 360.0
        return delta


class EncoderHEDS5540:
    """
    Source principale de mesure a 200 Hz.
    psi et psi_dot derives depuis les counts encodeur.
    """

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

        # Etat pour la derivee filtree
        self._psi_prev     = 0.0
        self._psi_dot_filt = 0.0

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
        """psi en deg, 0 au repos, positif vers le haut."""
        return -((self.position / ENCODER_CPR_X4) * 360.0)

    def get_psi_and_dot(self):
        """
        Retourne (psi, psi_dot) en deg et deg/s.
        psi_dot derive numeriquement + filtre passe-bas.
        """
        psi = self.get_psi()
        psi_dot_raw       = (psi - self._psi_prev) / DT
        self._psi_dot_filt = (PSI_DOT_ALPHA * psi_dot_raw +
                              (1.0 - PSI_DOT_ALPHA) * self._psi_dot_filt)
        self._psi_prev = psi
        return psi, self._psi_dot_filt

    def reset(self):
        with self._lock:
            self._position = 0
        self._psi_prev     = 0.0
        self._psi_dot_filt = 0.0

    def cancel(self):
        self._cb_a.cancel()
        self._cb_b.cancel()


class ESC:

    SIDE_NEUTRAL = 0
    SIDE_REVERSE = -1
    SIDE_FORWARD = +1

    def __init__(self, pi, pin, name="ESC"):
        self.pi         = pi
        self.pin        = pin
        self.name       = name
        self._cur       = ESC_NEUTRAL_US
        self._side      = self.SIDE_NEUTRAL
        self._dwell_end = 0.0
        print(f"[{name}]  Armement ({ESC_NEUTRAL_US} us sur GPIO{pin})...")
        pi.set_servo_pulsewidth(pin, ESC_NEUTRAL_US)
        time.sleep(3)
        print(f"[{name}]  Arme et pret")

    def _get_side(self, pw):
        if pw > ESC_NEUTRAL_US + ESC_DEADBAND_US:
            return self.SIDE_FORWARD
        elif pw < ESC_NEUTRAL_US - ESC_DEADBAND_US:
            return self.SIDE_REVERSE
        return self.SIDE_NEUTRAL

    def set_pwm(self, target_us):
        target_us   = int(max(ESC_MIN_US, min(ESC_MAX_US, target_us)))
        target_side = self._get_side(target_us)
        now         = time.monotonic()

        # Changement de sens → dwell
        if (self._side != self.SIDE_NEUTRAL and
                target_side != self.SIDE_NEUTRAL and
                target_side != self._side):
            target_us   = ESC_NEUTRAL_US
            self._dwell_end = now + ESC_DWELL_S

        if now < self._dwell_end:
            target_us = ESC_NEUTRAL_US

        delta     = max(-ESC_SLEW_MAX_US, min(ESC_SLEW_MAX_US, target_us - self._cur))
        self._cur = int(self._cur + delta)
        self._side = self._get_side(self._cur)
        self.pi.set_servo_pulsewidth(self.pin, self._cur)
        return self._cur

    def stop(self):
        while abs(self._cur - ESC_NEUTRAL_US) > ESC_SLEW_MAX_US:
            delta     = max(-ESC_SLEW_MAX_US, min(ESC_SLEW_MAX_US, ESC_NEUTRAL_US - self._cur))
            self._cur = int(self._cur + delta)
            self.pi.set_servo_pulsewidth(self.pin, self._cur)
            time.sleep(DT)
        self._cur = ESC_NEUTRAL_US
        self.pi.set_servo_pulsewidth(self.pin, ESC_NEUTRAL_US)

    def shutdown(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)


class PIDController:

    def __init__(self, kp, ki, kd, kg, setpoint_deg, rest_angle_deg=85.0):
        self.kp         = kp
        self.ki         = ki
        self.kd         = kd
        self.kg         = kg
        self.setpoint   = setpoint_deg
        self.rest_angle = rest_angle_deg
        self._integral     = 0.0
        self._integral_max = 120   # deg*s — a 200 Hz l'integrale accumule vite

    def reset_integral(self):
        self._integral = 0.0

    def compute(self, psi_deg, psi_dot_dps):
        error = self.setpoint - psi_deg

        u_p = self.kp * error

        self._integral += error * DT
        self._integral  = max(-self._integral_max,
                              min(self._integral_max, self._integral))
        u_i = self.ki * self._integral

        u_d = -self.kd * psi_dot_dps

        # Feedforward gravitationnel sur setpoint
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

        print("=" * 70)
        print("  TRMS — PID | Encodeur 200Hz | IMU calibration+yaw | Slew+Dwell")
        print(f"  Consigne psi* = {setpoint_deg:.1f} deg")
        print(f"  Kp={kp:.2f}  Ki={ki:.3f}  Kd={kd:.2f}  Kg={kg:.2f}")
        print(f"  Throttle max  = {THROTTLE_LIMIT*100:.0f}%")
        print(f"  Loop          = {CONTROL_FREQ_HZ:.0f} Hz  |  DT = {DT*1000:.1f} ms")
        print(f"  Slew          = {ESC_SLEW_MAX_US} us/iter = {ESC_SLEW_MAX_US*CONTROL_FREQ_HZ:.0f} us/s")
        print(f"  Dwell ESC     = {ESC_DWELL_S*1000:.0f} ms")
        print(f"  psi_dot filtre alpha = {PSI_DOT_ALPHA} (coupure ~{PSI_DOT_ALPHA*CONTROL_FREQ_HZ/(2*math.pi):.1f} Hz)")
        print("=" * 70)

        self.imu     = IMU_BNO085()
        self.encoder = EncoderHEDS5540(self.pi)
        self.esc     = ESC(self.pi, ESC_MAIN_PWM_PIN, name="ESC_MAIN")
        time.sleep(1.0)

        print()
        print("  *** CALIBRATION IMU : bras immobile a sa position de repos ***")
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
        print(f"[CAL]  u_G prevu au setpoint      = {u_g_preview:.3f}")
        print(f"[CAL]  Source controle            = ENCODEUR (psi + psi_dot derive)")
        print(f"[CAL]  Source logging yaw         = IMU (phi)")

        self.ctrl = PIDController(kp, ki, kd, kg, setpoint_deg,
                                  rest_angle_deg=rest_angle)

        # Verification encodeur au repos
        self.encoder.reset()
        time.sleep(0.1)
        psi_enc_check = self.encoder.get_psi()
        print(f"[ENC]  psi encodeur post-reset = {psi_enc_check:.2f} deg (attendu 0)")
        print("[ENC]  OK")

        from datetime import datetime
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = (
            f"logs/trms_enc_kp{kp}_ki{ki}_kd{kd}_kg{kg}"
            f"_sp{setpoint_deg}_ra{rest_angle:.0f}"
            f"_thr{THROTTLE_LIMIT}_{ts}.csv"
        )
        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write(
            "t_s,iteration,"
            "psi_enc_deg,psi_dot_enc_dps,phi_yaw_deg,"
            "setpoint_deg,rest_angle_deg,phys_angle_deg,"
            "error_deg,integral_deg_s,"
            "u_norm,u_p,u_i,u_d,u_g,pwm_us\n"
        )
        print(f"[LOG]  CSV → {self.csv_path}")

        signal.signal(signal.SIGINT,  self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        print()
        print("=" * 70)
        print("  Systeme pret — Ctrl+C pour arreter")
        print("=" * 70)

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
        hdr = (f"{'#':>6} | {'psi':>7} | {'psi_d':>7} | {'phi':>7} | "
               f"{'Err':>7} | {'I':>7} | {'u':>7} | {'PWM':>5}")
        print(f"\n{hdr}")
        print("-" * len(hdr))

        it      = 0
        t_start = time.monotonic()

        # Premiere lecture pour initialiser la derivee
        self.encoder.get_psi_and_dot()

        try:
            while self.running:
                t0 = time.monotonic()
                it += 1

                # Source principale : encodeur
                psi, psi_dot_dps = self.encoder.get_psi_and_dot()

                if abs(psi) > MAX_PSI_DEG:
                    self._emergency(f"|psi| = {abs(psi):.1f} > {MAX_PSI_DEG}")
                    break

                # Yaw pour logging (IMU, non bloquant)
                phi = self.imu.get_phi()

                u, err, u_p, u_i, u_d, u_g = self.ctrl.compute(psi, psi_dot_dps)
                pwm_cmd    = self.ctrl.to_pwm(u)
                pwm_actual = self.esc.set_pwm(pwm_cmd)

                t_elapsed  = t0 - t_start
                phys_angle = self.ctrl.rest_angle + self.ctrl.setpoint
                phi_log    = str(round(phi, 4)) if phi is not None else ""

                self.csv_file.write(
                    f"{t_elapsed:.4f},{it},"
                    f"{psi:.4f},{psi_dot_dps:.4f},{phi_log},"
                    f"{self.ctrl.setpoint:.2f},{self.ctrl.rest_angle:.2f},{phys_angle:.2f},"
                    f"{err:.4f},{self.ctrl._integral:.4f},"
                    f"{u:.6f},{u_p:.6f},{u_i:.6f},{u_d:.6f},{u_g:.6f},{pwm_actual}\n"
                )

                phi_str = str(round(phi, 1)) if phi is not None else "  N/A"
                print(
                    f"\r{it:6d} | {psi:+7.1f} | {psi_dot_dps:+7.1f} | {phi_str:>7} | "
                    f"{err:+7.1f} | {self.ctrl._integral:+7.2f} | {u:+7.3f} | {pwm_actual:5d}",
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
        description="TRMS — PID encodeur 200Hz + IMU calibration/yaw",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples :
  python3 trms_controller.py --kp 1.5 --ki 0.10 --kd 0.05 --kg 0.25 --setpoint 10
  python3 trms_controller.py --kp 1.5 --ki 0.10 --kd 0.05 --kg 0.0  --setpoint -20

Note : Kd beaucoup plus faible qu'a 50 Hz car psi_dot est maintenant propre
       (derivee encodeur filtree, pas de bruit gyro)
        """)

    p.add_argument("--setpoint",   type=float, default=10.0)
    p.add_argument("--kp",         type=float, default=1.5)
    p.add_argument("--ki",         type=float, default=0.10)
    p.add_argument("--kd",         type=float, default=0.05,
                   help="Gain derive — faible car psi_dot encodeur est propre (defaut: 0.05)")
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
