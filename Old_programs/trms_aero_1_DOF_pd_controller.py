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

# ── GPIO ────────────────────────────────────────────────────────────────────
ENCODER_PIN_A       = 27
ENCODER_PIN_B       = 17
ESC_MAIN_PWM_PIN    = 18
# ESC_TAIL_PWM_PIN  = ???   # À câbler lors du passage 2-DOF

# ── Encodeur ────────────────────────────────────────────────────────────────
ENCODER_CPR         = 500
ENCODER_CPR_X4      = 2000

# ── ESC ─────────────────────────────────────────────────────────────────────
ESC_NEUTRAL_US      = 1500
ESC_MIN_US          = 1100
ESC_MAX_US          = 1900
ESC_DEADBAND_US     = 25

# ── Boucle de contrôle ──────────────────────────────────────────────────────
CONTROL_FREQ_HZ     = 50.0
DT                  = 1.0 / CONTROL_FREQ_HZ

MAX_PSI_DEG         = 120.0
THROTTLE_LIMIT      = 0.60

# ── Calibration IMU ─────────────────────────────────────────────────────────
CALIB_SAMPLES       = 50
CALIB_PERIOD_S      = 2.0


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
        sinp = 2.0 * (qr * qj - qk * qi)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.degrees(math.asin(sinp))

        siny = 2.0 * (qr * qk + qi * qj)
        cosy = 1.0 - 2.0 * (qj * qj + qk * qk)
        yaw = math.degrees(math.atan2(siny, cosy))

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
        print("[IMU]  >>> BRAS IMMOBILE AU REPOS <<<")

        samples_pitch = []
        samples_yaw   = []
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
        self.yaw_offset   = sum(samples_yaw)   / len(samples_yaw) if samples_yaw else 0.0

        spread = max(samples_pitch) - min(samples_pitch)
        print(f"[IMU]  pitch_offset = {self.pitch_offset:.2f} deg  |  yaw_offset = {self.yaw_offset:.2f} deg")
        print(f"[IMU]  Dispersion pitch = {spread:.2f} deg ({len(samples_pitch)} echantillons)")
        if spread > 5.0:
            print("[IMU]  ATTENTION : dispersion elevee — bras en mouvement ?")
        return True

    def get_psi(self):
        pitch, _ = self._get_raw()
        if pitch is None:
            return None
        return -(pitch - self.pitch_offset)

    def get_phi(self):
        """Yaw φ relatif à la position initiale (non contrôlé, logging seulement)."""
        _, yaw = self._get_raw()
        if yaw is None:
            return None
        delta = yaw - self.yaw_offset
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0
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
        self.pi   = pi
        self.pin  = pin
        self.name = name
        print(f"[{name}]  Armement ({ESC_NEUTRAL_US} us sur GPIO{pin})...")
        pi.set_servo_pulsewidth(pin, ESC_NEUTRAL_US)
        time.sleep(3)
        print(f"[{name}]  Arme et pret")

    def set_pwm(self, pulse_us):
        pw = int(max(ESC_MIN_US, min(ESC_MAX_US, pulse_us)))
        self.pi.set_servo_pulsewidth(self.pin, pw)
        return pw

    def stop(self):
        self.pi.set_servo_pulsewidth(self.pin, ESC_NEUTRAL_US)

    def shutdown(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)


class PDController:

    def __init__(self, kp, kd, kg, setpoint_deg):
        self.kp       = kp
        self.kd       = kd
        self.kg       = kg
        self.setpoint = setpoint_deg

    def compute(self, psi_deg, psi_dot_dps):
        error = self.setpoint - psi_deg
        u_p   = self.kp * error
        u_d   = -self.kd * psi_dot_dps
        u_g   = self.kg * math.sin(math.radians(self.setpoint)) * 90.0
        u_raw = u_p + u_d + u_g
        u_norm = max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, u_raw / 90.0))
        return u_norm, error, u_p / 90.0, u_d / 90.0, u_g / 90.0

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

    def __init__(self, kp, kd, kg, setpoint_deg):
        self.running = False

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod non lance ! → sudo pigpiod")

        print("=" * 62)
        print("  TRMS — Controleur PD + Gravite (rotor principal seul)")
        print(f"  Consigne psi* = {setpoint_deg:.1f} deg")
        print(f"  Kp={kp:.2f}  Kd={kd:.2f}  Kg={kg:.2f}")
        print(f"  Throttle max : {THROTTLE_LIMIT*100:.0f}%")
        print("=" * 62)

        self.imu     = IMU_BNO085()
        self.encoder = EncoderHEDS5540(self.pi)
        self.esc     = ESC(self.pi, ESC_MAIN_PWM_PIN, name="ESC_MAIN")
        time.sleep(1.0)

        self.ctrl = PDController(kp, kd, kg, setpoint_deg)

        print()
        print("  *** CALIBRATION : bras immobile au repos ***")
        print()
        if not self.imu.calibrate():
            self._abort()
            raise RuntimeError("Echec calibration IMU")

        psi_check = self.imu.get_psi()
        print(f"[CAL]  psi post-calib = {psi_check:.2f} deg (attendu ~ 0)")
        if psi_check is not None and abs(psi_check) > 10.0:
            print(f"[CAL]  ATTENTION : ecart important — bras en mouvement ?")
        else:
            print("[CAL]  OK")

        self.encoder.reset()
        print("[ENC]  Position remise a zero")

        from datetime import datetime
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = (
            f"logs/trms_main_kp{kp}_kd{kd}_kg{kg}"
            f"_sp{setpoint_deg}_thr{THROTTLE_LIMIT}_{ts}.csv"
        )
        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write(
            "t_s,iteration,"
            "psi_imu_deg,psi_enc_deg,"
            "phi_yaw_deg,"
            "psi_dot_dps,"
            "setpoint_deg,error_deg,"
            "u_norm,u_p,u_d,u_g,"
            "pwm_us,pitch_raw_deg\n"
        )
        print(f"[LOG]  CSV → {self.csv_path}")

        signal.signal(signal.SIGINT,  self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        print()
        print("=" * 62)
        print("  Systeme pret — Ctrl+C pour arreter")
        print("=" * 62)

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
               f"{'psi_d':>7} | {'Err':>7} | {'u':>7} | {'PWM':>5}")
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
                gyro_raw    = self.imu.get_gyro_pitch_rate()
                psi_dot_dps = -math.degrees(gyro_raw)

                u, err, u_p, u_d, u_g = self.ctrl.compute(psi, psi_dot_dps)
                pwm_cmd    = self.ctrl.to_pwm(u)
                pwm_actual = self.esc.set_pwm(pwm_cmd)

                t_elapsed  = t0 - t_start
                pitch_raw  = self.imu.get_pitch_raw()
                phi_log    = f"{phi:.4f}" if phi is not None else ""
                pr         = f"{pitch_raw:.4f}" if pitch_raw is not None else ""

                self.csv_file.write(
                    f"{t_elapsed:.4f},{it},"
                    f"{psi:.4f},{psi_enc:.4f},"
                    f"{phi_log},"
                    f"{psi_dot_dps:.4f},"
                    f"{self.ctrl.setpoint:.2f},{err:.4f},"
                    f"{u:.6f},{u_p:.6f},{u_d:.6f},{u_g:.6f},"
                    f"{pwm_actual},{pr}\n"
                )

                phi_str = f"{phi:+7.1f}" if phi is not None else "    N/A"
                line = (
                    f"\r{it:5d} | {psi:+7.1f} | {psi_enc:+7.1f} | {phi_str} | "
                    f"{psi_dot_dps:+7.1f} | {err:+7.1f} | {u:+7.3f} | {pwm_actual:5d}"
                )
                print(line, end="", flush=True)

                elapsed = time.monotonic() - t0
                wait    = DT - elapsed
                if wait > 0:
                    time.sleep(wait)

        except KeyboardInterrupt:
            print("\n[STOP] Ctrl+C")

        finally:
            self._cleanup()

    def _cleanup(self):
        print("\n[STOP] Arret moteur principal...")
        self.esc.stop()
        time.sleep(0.3)
        self.esc.shutdown()
        self.encoder.cancel()
        self.pi.stop()
        if hasattr(self, "csv_file") and self.csv_file:
            self.csv_file.close()
            print(f"[LOG]  CSV sauvegarde → {self.csv_path}")
        print("[STOP] Systeme arrete proprement")


def main():
    p = argparse.ArgumentParser(
        description="TRMS — Controleur PD + gravite (rotor principal seul, IMU)",
        formatter_class=argparse.RawDescriptionHelpFormatter)

    p.add_argument("--setpoint",  type=float, default=45.0)
    p.add_argument("--kp",        type=float, default=3.5)
    p.add_argument("--kd",        type=float, default=1.2)
    p.add_argument("--kg",        type=float, default=1.9)
    p.add_argument("--throttle",  type=float, default=0.60)

    args = p.parse_args()

    global THROTTLE_LIMIT
    THROTTLE_LIMIT = max(0.05, min(1.0, args.throttle))

    ctrl = TRMSController(
        kp          = args.kp,
        kd          = args.kd,
        kg          = args.kg,
        setpoint_deg= args.setpoint,
    )
    ctrl.run()


if __name__ == "__main__":
    main()