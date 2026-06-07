#!/usr/bin/env python3
import time
import math
import signal
import sys
import argparse
import threading
import os

import pigpio
import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

ENCODER_PIN_A   = 27
ENCODER_PIN_B   = 17
ESC_PWM_PIN     = 12

ENCODER_CPR     = 500
ENCODER_CPR_X4  = 2000

ESC_NEUTRAL_US  = 1500
ESC_MIN_US      = 1100
ESC_MAX_US      = 1900
ESC_DEADBAND_US = 25

CONTROL_FREQ_HZ = 50.0
DT = 1.0 / CONTROL_FREQ_HZ

MAX_PSI_DEG     = 120.0
THROTTLE_LIMIT  = 0.60

CALIB_SAMPLES   = 50
CALIB_PERIOD_S  = 2.0

class IMU_BNO085:

    def __init__(self):
        print("[IMU]  Initialisation BNO085 sur I2C @ 0x4A...")
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        self.bno = BNO08X_I2C(self.i2c, address=0x4A)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        time.sleep(0.5)

        # L'offset sera déterminé par calibration
        self.pitch_offset = 0.0
        print("[IMU]  BNO085 pret")

    def _quat_to_pitch_deg(self, qi, qj, qk, qr):
        sinp = 2.0 * (qr * qj - qk * qi)
        sinp = max(-1.0, min(1.0, sinp))
        return math.degrees(math.asin(sinp))

    def get_pitch_raw(self):
        """Pitch brut de l'IMU (degrés)."""
        try:
            quat = self.bno.quaternion
            if quat is None:
                return None
            qi, qj, qk, qr = quat
            return self._quat_to_pitch_deg(qi, qj, qk, qr)
        except Exception:
            return None

    def calibrate(self, n_samples=CALIB_SAMPLES, duration_s=CALIB_PERIOD_S):
        """
        Mesure le pitch brut moyen au repos.
        IMPORTANT : le bras doit être immobile et pendant vers le bas !
        """
        print(f"[IMU]  Calibration en cours ({duration_s}s, {n_samples} echantillons)...")
        print("[IMU]  >>> NE PAS TOUCHER LE BRAS ! <<<")

        samples = []
        dt = duration_s / n_samples

        for i in range(n_samples):
            raw = self.get_pitch_raw()
            if raw is not None:
                samples.append(raw)
            time.sleep(dt)

        if len(samples) < 5:
            print("[IMU]  ERREUR : pas assez d'echantillons valides !")
            return False

        self.pitch_offset = sum(samples) / len(samples)
        spread = max(samples) - min(samples)

        print(f"[IMU]  Offset mesure = {self.pitch_offset:.2f} deg")
        print(f"[IMU]  Dispersion = {spread:.2f} deg ({len(samples)} echantillons)")

        if spread > 5.0:
            print("[IMU]  ATTENTION : dispersion elevee — le bras bougeait ?")

        return True

    def get_psi(self):
        """
        Angle ψ dans la convention unifiée.
        ψ = -(pitch_brut - pitch_offset)

        Sur ce montage, quand le bras monte le pitch brut DIMINUE.
        On inverse donc le signe pour que monter = ψ positif.

        Repos (bas) :  pitch_brut = offset → ψ = 0°     ✓
        Vertical :     pitch_brut < offset → ψ = +90°    ✓
        """
        raw = self.get_pitch_raw()
        if raw is None:
            return None
        return -(raw - self.pitch_offset)

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
        self.pi = pi
        self._position = 0
        self._lock = threading.Lock()

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

    def get_angle_raw(self):
        return (self.position / ENCODER_CPR_X4) * 360.0

    def get_psi(self):
        """ψ = -angle_brut (repos=0°, horizontal=+90°)."""
        return -self.get_angle_raw()

    def reset(self):
        with self._lock:
            self._position = 0

    def cancel(self):
        self._cb_a.cancel()
        self._cb_b.cancel()

class ESC:

    def __init__(self, pi):
        self.pi = pi
        print(f"[ESC]  Armement ({ESC_NEUTRAL_US} us sur GPIO{ESC_PWM_PIN})...")
        pi.set_servo_pulsewidth(ESC_PWM_PIN, ESC_NEUTRAL_US)
        time.sleep(3)
        print("[ESC]  ESC arme et pret")

    def set_pwm(self, pulse_us):
        pw = int(max(ESC_MIN_US, min(ESC_MAX_US, pulse_us)))
        self.pi.set_servo_pulsewidth(ESC_PWM_PIN, pw)
        return pw

    def stop(self):
        self.pi.set_servo_pulsewidth(ESC_PWM_PIN, ESC_NEUTRAL_US)

    def shutdown(self):
        self.pi.set_servo_pulsewidth(ESC_PWM_PIN, 0)

class PDController:

    def __init__(self, kp, kd, kg, setpoint_deg):
        self.kp = kp
        self.kd = kd
        self.kg = kg
        self.setpoint = setpoint_deg

    def compute(self, psi_deg, psi_dot_dps):
        # erreur positive = il faut monter
        error = self.setpoint - psi_deg

        # P
        u_p = self.kp * error

        # D
        u_d = -self.kd * psi_dot_dps

        # Gravité calculée sur la consigne
        psi_ref_rad = math.radians(self.setpoint)
        u_g = self.kg * math.sin(psi_ref_rad) * 90.0

        # psi_rad = math.radians(psi_deg)
        # u_g = self.kg * math.sin(psi_rad) * 90.0

        u_raw = u_p + u_d + u_g

        u_norm = u_raw / 90.0
        u_norm = max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, u_norm))

        return u_norm, error, u_p / 90.0, u_d / 90.0, u_g / 90.0

    def to_pwm(self, u_norm):
        """
        Mapping commande → PWM avec inversion moteur.

        Sur cet ESC + hélice CCW :
          reverse (PWM < 1500) = CCW = pousse vers le HAUT
          forward (PWM > 1500) = CW  = pousse vers le BAS

        Donc : u > 0 (veut monter) → PWM < 1500 (reverse)
               u < 0 (veut descendre) → PWM > 1500 (forward)
        """

        u_esc = -u_norm 

        if u_esc > 0:
            pw = (ESC_NEUTRAL_US + ESC_DEADBAND_US +
                  u_esc * (ESC_MAX_US - ESC_NEUTRAL_US - ESC_DEADBAND_US))
        else:
            pw = (ESC_NEUTRAL_US - ESC_DEADBAND_US +
                  u_esc * (ESC_NEUTRAL_US - ESC_DEADBAND_US - ESC_MIN_US))
        return int(max(ESC_MIN_US, min(ESC_MAX_US, pw)))

class TRMSPController:

    def __init__(self, kp, kd, kg, setpoint_deg, sensor_mode):
        self.sensor_mode = sensor_mode
        self.running = False

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod non lance ! → sudo pigpiod")

        print("=" * 62)
        print("  TRMS  —  Controleur PD + Gravite (auto-calibration)")
        print(f"  Consigne psi* = {setpoint_deg:.1f} deg")
        print(f"  K_P = {kp:.2f}  |  K_D = {kd:.2f}  |  K_G = {kg:.2f}")
        print(f"  Throttle max : {THROTTLE_LIMIT*100:.0f}%")
        print(f"  Capteur : {sensor_mode}")
        print("=" * 62)

        self.imu = IMU_BNO085()
        self.encoder = EncoderHEDS5540(self.pi)
        self.esc = ESC(self.pi)
        time.sleep(1.0)   
        self.ctrl = PDController(kp, kd, kg, setpoint_deg)

        # --- AUTO-CALIBRATION ---
        print()
        print("  *** CALIBRATION : garder le bras immobile au repos ***")
        print()
        ok = self.imu.calibrate()
        if not ok:
            self.esc.shutdown()
            self.encoder.cancel()
            self.pi.stop()
            raise RuntimeError("Echec de la calibration IMU")

        # Vérification post-calibration
        psi_check = self.imu.get_psi()
        print(f"[CAL]  Verification : psi = {psi_check:.2f} deg (doit etre ~ 0)")
        if psi_check is not None and abs(psi_check) > 10.0:
            print(f"[CAL]  ATTENTION : psi = {psi_check:.2f} != 0 — le bras a bouge ?")
        else:
            print("[CAL]  Calibration OK")

        # Reset encodeur aussi
        self.encoder.reset()
        print("[ENC]  Position remise a zero")

        # --- CSV LOG ---
        from datetime import datetime
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"logs/trms_kp{kp}_kd{kd}_kg{kg}_sp{setpoint_deg}_thr{THROTTLE_LIMIT}_{ts}.csv"
        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write(
            "t_s,iteration,psi_imu_deg,psi_enc_deg,psi_dot_dps,setpoint_deg,"
            "error_deg,u_norm,u_p,u_d,u_g,pwm_us,pitch_raw_deg\n"
        )
        print(f"[LOG]  CSV → {self.csv_path}")

        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        print()
        print("=" * 62)
        print("  Systeme pret — Ctrl+C pour arreter")
        print("=" * 62)

    def _sig(self, signum, frame):
        print("\n[STOP] Arret demande...")
        self.running = False

    def _emergency(self, reason):
        self.running = False
        self.esc.stop()
        print(f"\n[!!!] ARRET D'URGENCE : {reason}")

    def _read_psi(self):
        if self.sensor_mode == "imu":
            return self.imu.get_psi(), None
        elif self.sensor_mode == "encoder":
            return self.encoder.get_psi(), None
        else:
            return self.imu.get_psi(), self.encoder.get_psi()

    def run(self):
        self.running = True

        both = self.sensor_mode == "both"
        if both:
            hdr = (f"{'#':>5} | {'psi':>7} | {'enc':>7} | {'psi_d':>7} | "
                   f"{'Err':>7} | {'u':>7} | {'PWM':>5}")
        else:
            hdr = (f"{'#':>5} | {'psi':>7} | {'psi_d':>7} | "
                   f"{'Err':>7} | {'u':>7} | {'PWM':>5}")

        print(f"\n{hdr}")
        print("-" * len(hdr))

        it = 0
        t_start = time.monotonic()

        try:
            while self.running:
                t0 = time.monotonic()
                it += 1

                psi, psi2 = self._read_psi()
                if psi is None:
                    time.sleep(DT)
                    continue

                if abs(psi) > MAX_PSI_DEG:
                    self._emergency(f"|psi| = {abs(psi):.1f} > {MAX_PSI_DEG}")
                    break

                # Lire la vitesse angulaire du gyroscope (rad/s → deg/s)
                # Le signe est inversé comme pour get_psi : monter = positif
                gyro_raw = self.imu.get_gyro_pitch_rate()
                psi_dot_dps = -math.degrees(gyro_raw)  # Inverser pour cohérence avec ψ

                # Contrôleur PD + gravité
                u, err, u_p, u_d, u_g = self.ctrl.compute(psi, psi_dot_dps)
                pwm_cmd = self.ctrl.to_pwm(u)
                pwm_actual = self.esc.set_pwm(pwm_cmd)

                # --- CSV ---
                t_elapsed = t0 - t_start
                pitch_raw = self.imu.get_pitch_raw()
                psi_enc = psi2 if psi2 is not None else self.encoder.get_psi()
                pr = f"{pitch_raw:.4f}" if pitch_raw is not None else ""
                self.csv_file.write(
                    f"{t_elapsed:.4f},{it},{psi:.4f},{psi_enc:.4f},{psi_dot_dps:.4f},"
                    f"{self.ctrl.setpoint:.2f},{err:.4f},{u:.6f},{u_p:.6f},{u_d:.6f},{u_g:.6f},"
                    f"{pwm_actual},{pr}\n"
                )

                # --- Affichage terminal ---
                if both:
                    p2 = f"{psi2:+7.1f}" if psi2 is not None else "    N/A"
                    line = (f"\r{it:5d} | {psi:+7.1f} | {p2} | {psi_dot_dps:+7.1f} | "
                            f"{err:+7.1f} | {u:+7.3f} | {pwm_actual:5d}")
                else:
                    line = (f"\r{it:5d} | {psi:+7.1f} | {psi_dot_dps:+7.1f} | "
                            f"{err:+7.1f} | {u:+7.3f} | {pwm_actual:5d}")
                print(line, end="", flush=True)

                elapsed = time.monotonic() - t0
                wait = DT - elapsed
                if wait > 0:
                    time.sleep(wait)

        except KeyboardInterrupt:
            print("\n[STOP] Ctrl+C")

        finally:
            self._cleanup()

    def _cleanup(self):
        print("\n[STOP] Arret moteur...")
        self.esc.stop()
        time.sleep(0.3)
        self.esc.shutdown()
        self.encoder.cancel()
        self.pi.stop()
        # Fermer le CSV
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            print(f"[LOG]  CSV sauvegarde → {self.csv_path}")
        print("[STOP] Systeme arrete proprement")

def main():
    p = argparse.ArgumentParser(
        description="Controleur PD + gravite pour le TRMS (auto-calibration)",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    p.add_argument("--setpoint", type=float, default=45.0,
                   help="Angle cible en deg (defaut: 45)")
    p.add_argument("--kp", type=float, default=1.5,
                   help="Gain proportionnel (defaut: 1.0)")
    p.add_argument("--kd", type=float, default=0.3,
                   help="Gain derive (defaut: 0.3)")
    p.add_argument("--kg", type=float, default=0.45,
                   help="Gain compensation gravite (defaut: 0.45)")
    p.add_argument("--throttle", type=float, default=0.80,
                   help="Limite throttle 0.0-1.0 (defaut: 0.60 = 60%%)")
    p.add_argument("--sensor", choices=["imu", "encoder", "both"], default="imu",
                   help="Source de mesure (defaut: imu)")

    args = p.parse_args()

    global THROTTLE_LIMIT
    THROTTLE_LIMIT = max(0.05, min(1.0, args.throttle))

    ctrl = TRMSPController(
        kp=args.kp,
        kd=args.kd,
        kg=args.kg,
        setpoint_deg=args.setpoint,
        sensor_mode=args.sensor,
    )
    ctrl.run()


if __name__ == "__main__":
    main()
