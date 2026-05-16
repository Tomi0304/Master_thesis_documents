#!/usr/bin/env python3
"""
TRMS — Controleur PID avec choix du mode capteur
Rotor principal seul (GPIO 12)

Modes capteurs (--sensor-mode) :
  enc   : Encodeur seul, psi_dot derive filtre        (200 Hz)
  imu   : IMU seul, psi depuis quaternion, psi_dot depuis gyro
          BNO085 force a 200 Hz via interval_us=5000
          Boucle 200 Hz avec detection de nouvelle donnee IMU
          (si pas de nouvelle donnee → skip calcul PID, pas d'accumulation integrale)
  fused : Filtre complementaire enc+IMU (alpha=0.98)
          psi_fused = 0.98*psi_enc + 0.02*psi_imu
          psi_dot depuis encodeur filtre

IMU orientee 180 deg autour de Z :
  pitch_raw ≈ -85 deg au repos, monte vers 0 en montant
  get_rest_angle = -pitch_offset

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
ESC_DWELL_S      = 20

CONTROL_FREQ_HZ  = 200.0
DT               = 1.0 / CONTROL_FREQ_HZ
MAX_PSI_DEG      = 100.0

CALIB_SAMPLES    = 50
CALIB_PERIOD_S   = 2.0

PSI_DOT_ALPHA    = 0.3    # filtre passe-bas derivee encodeur (~10 Hz a 200 Hz)
FUSED_ALPHA      = 0.98   # filtre complementaire : 98% enc, 2% IMU
IMU_INTERVAL_US  = 5000   # 200 Hz — coherent avec la boucle de controle

THROTTLE_LIMIT   = 0.95
SENSOR_MODES     = ("enc", "imu", "fused")


class IMU_BNO085:

    def __init__(self, need_gyro=False):
        print("[IMU]  Initialisation BNO085 sur I2C @ 0x4A...")
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(self.i2c, address=0x4A)
        time.sleep(0.5)
        try:
            self.bno.soft_reset()
        except AttributeError:
            pass
        time.sleep(1.0)

        # Force le taux de rafraichissement a IMU_INTERVAL_US
        self._enable_feature_safe(BNO_REPORT_ROTATION_VECTOR,
                                   interval_us=IMU_INTERVAL_US)
        time.sleep(0.2)
        if need_gyro:
            self._enable_feature_safe(BNO_REPORT_GYROSCOPE,
                                       interval_us=IMU_INTERVAL_US)
            print(f"[IMU]  Gyroscope active @ {1e6/IMU_INTERVAL_US:.0f} Hz")
        time.sleep(0.5)

        self.pitch_offset = 0.0
        self.yaw_offset   = 0.0

        # Etat pour la detection de nouvelle donnee
        self._last_quat   = None
        self._new_data    = False

        print(f"[IMU]  BNO085 pret @ {1e6/IMU_INTERVAL_US:.0f} Hz")

    def _enable_feature_safe(self, feature, retries=5, delay=0.5,
                              interval_us=IMU_INTERVAL_US):
        for attempt in range(1, retries + 1):
            try:
                # Tente avec interval_us (versions recentes de la lib)
                try:
                    self.bno.enable_feature(feature, interval_us)
                except TypeError:
                    # Fallback si la version ne supporte pas interval_us
                    self.bno.enable_feature(feature)
                    print(f"[IMU]  AVERTISSEMENT : interval_us non supporte — "
                          f"taux par defaut (20 Hz)")
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
                self._new_data = False
                return None, None

            # Detection de nouvelle donnee : compare avec le quaternion precedent
            if self._last_quat is not None and quat == self._last_quat:
                self._new_data = False
            else:
                self._new_data = True
                self._last_quat = quat

            return self._quat_to_euler(*quat)
        except Exception:
            self._new_data = False
            return None, None

    def has_new_data(self):
        """True si la derniere lecture a retourne une nouvelle donnee IMU."""
        return self._new_data

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
        print(f"[IMU]  Dispersion   = {spread:.2f} deg")
        if spread > 3.0:
            print("[IMU]  ATTENTION : dispersion elevee — bras en mouvement ?")
        return True

    def get_rest_angle_from_vertical(self):
        """IMU orientee 180 deg / Z : pitch_offset ≈ -85 → alpha_rest = 85 deg"""
        return -self.pitch_offset

    def get_psi(self):
        """psi depuis IMU, positif vers le haut. Met a jour has_new_data()."""
        pitch, _ = self._get_raw()
        if pitch is None:
            return None
        return pitch - self.pitch_offset

    def get_psi_and_new(self):
        """Retourne (psi, is_new_data). Appel unique par iteration."""
        pitch, _ = self._get_raw()
        if pitch is None:
            return None, False
        return pitch - self.pitch_offset, self._new_data

    def get_psi_dot_gyro(self):
        try:
            gyro = self.bno.gyro
            if gyro is None:
                return 0.0
            _, gy, _ = gyro
            return -math.degrees(gy)   # ← remettre le signe négatif
        except Exception:
            return 0.0

    def get_phi(self):
        _, yaw = self._get_raw()
        if yaw is None:
            return None
        delta = yaw - self.yaw_offset
        if delta >  180.0: delta -= 360.0
        if delta < -180.0: delta += 360.0
        return delta


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
        self._last_state   = (a << 1) | b
        self._cb_a         = pi.callback(ENCODER_PIN_A, pigpio.EITHER_EDGE, self._cb)
        self._cb_b         = pi.callback(ENCODER_PIN_B, pigpio.EITHER_EDGE, self._cb)
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
        return -((self.position / ENCODER_CPR_X4) * 360.0)

    def get_psi_and_dot(self):
        psi = self.get_psi()
        psi_dot_raw        = (psi - self._psi_prev) / DT
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
        if (self._side != self.SIDE_NEUTRAL and
                target_side != self.SIDE_NEUTRAL and
                target_side != self._side):
            target_us   = ESC_NEUTRAL_US
            self._dwell_end = now + ESC_DWELL_S
        if now < self._dwell_end:
            target_us = ESC_NEUTRAL_US
        delta      = max(-ESC_SLEW_MAX_US, min(ESC_SLEW_MAX_US, target_us - self._cur))
        self._cur  = int(self._cur + delta)
        self._side = self._get_side(self._cur)
        self.pi.set_servo_pulsewidth(self.pin, self._cur)
        return self._cur

    def stop(self):
        while abs(self._cur - ESC_NEUTRAL_US) > ESC_SLEW_MAX_US:
            delta     = max(-ESC_SLEW_MAX_US, min(ESC_SLEW_MAX_US,
                            ESC_NEUTRAL_US - self._cur))
            self._cur = int(self._cur + delta)
            self.pi.set_servo_pulsewidth(self.pin, self._cur)
            time.sleep(DT)
        self._cur = ESC_NEUTRAL_US
        self.pi.set_servo_pulsewidth(self.pin, ESC_NEUTRAL_US)

    def shutdown(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)


class PIDController:

    def __init__(self, kp, ki, kd, kg, setpoint_deg, rest_angle_deg=85.0):
        self.kp           = kp
        self.ki           = ki
        self.kd           = kd
        self.kg           = kg
        self.setpoint     = setpoint_deg
        self.rest_angle   = rest_angle_deg
        self._integral     = 0.0
        self._integral_max = 120.0

    def reset_integral(self):
        self._integral = 0.0

    def compute(self, psi_deg, psi_dot_dps, dt_actual=None):
        """
        dt_actual : DT effectif de cette iteration.
        Permet d'adapter l'integrale si la boucle saute des iterations (mode IMU).
        """
        dt = dt_actual if dt_actual is not None else DT
        error = self.setpoint - psi_deg
        u_p   = self.kp * error

        self._integral += error * dt
        self._integral  = max(-self._integral_max,
                              min(self._integral_max, self._integral))
        u_i = self.ki * self._integral
        u_d = -self.kd * psi_dot_dps

        phys_angle_rad = math.radians(self.rest_angle + self.setpoint)
        u_g = self.kg * math.sin(phys_angle_rad) * 90.0

        u_raw  = u_p + u_i + u_d + u_g
        u_norm = max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, u_raw / 90.0))

        if abs(u_raw / 90.0) > THROTTLE_LIMIT:
            self._integral -= error * dt

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

    def __init__(self, kp, ki, kd, kg, setpoint_deg,
                 rest_angle_override=None, sensor_mode="enc"):

        if sensor_mode not in SENSOR_MODES:
            raise ValueError(f"sensor_mode doit etre parmi {SENSOR_MODES}")

        self.sensor_mode = sensor_mode
        self.running     = False
        self.pi          = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod non lance ! → sudo pigpiod")

        print("=" * 70)
        print("  TRMS — PID | Slew + Dwell")
        print(f"  Consigne psi*  = {setpoint_deg:.1f} deg")
        print(f"  Kp={kp:.2f}  Ki={ki:.3f}  Kd={kd:.2f}  Kg={kg:.2f}")
        print(f"  Throttle max   = {THROTTLE_LIMIT*100:.0f}%")
        print(f"  Loop           = {CONTROL_FREQ_HZ:.0f} Hz  |  DT = {DT*1000:.1f} ms")
        print(f"  Slew           = {ESC_SLEW_MAX_US} us/iter")
        print(f"  Dwell ESC      = {ESC_DWELL_S*1000:.0f} ms")
        print(f"  Mode capteur   = {sensor_mode.upper()}", end="")
        if sensor_mode == "imu":
            print(f"  (IMU @ {1e6/IMU_INTERVAL_US:.0f} Hz, calcul PID sur nouvelle donnee uniquement)")
        elif sensor_mode == "fused":
            print(f"  (alpha={FUSED_ALPHA})")
        else:
            print()
        print("=" * 70)

        need_gyro = (sensor_mode == "imu")
        self.imu  = IMU_BNO085(need_gyro=need_gyro)

        # Encodeur toujours present (psi_dot propre en mode fused/enc,
        # et disponible comme reference en mode imu)
        self.encoder = EncoderHEDS5540(self.pi)
        self.esc     = ESC(self.pi, ESC_MAIN_PWM_PIN, name="ESC_MAIN")
        time.sleep(1.0)

        print()
        print("  *** CALIBRATION IMU : bras immobile a sa position de repos ***")
        print()
        if not self.imu.calibrate():
            self._abort()
            raise RuntimeError("Echec calibration IMU")

        rest_angle = (rest_angle_override if rest_angle_override is not None
                      else self.imu.get_rest_angle_from_vertical())
        src = "manuel" if rest_angle_override else "IMU auto"
        print(f"[CAL]  Angle de repos ({src}) = {rest_angle:.1f} deg depuis verticale")

        self.ctrl = PIDController(kp, ki, kd, kg, setpoint_deg,
                                  rest_angle_deg=rest_angle)
        self.encoder.reset()
        print("[ENC]  Position remise a zero")

        from datetime import datetime
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = (
            f"logs/trms_{sensor_mode}_kp{kp}_ki{ki}_kd{kd}_kg{kg}"
            f"_sp{setpoint_deg}_ra{rest_angle:.0f}"
            f"_thr{THROTTLE_LIMIT}_{ts}.csv"
        )
        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write(
            "t_s,iteration,sensor_mode,new_data,"
            "psi_deg,psi_dot_dps,psi_enc_deg,psi_imu_deg,phi_yaw_deg,"
            "setpoint_deg,rest_angle_deg,"
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
        self.running  = True
        imu_skip_count = 0
        imu_new_count  = 0

        hdr = (f"{'#':>6} | {'psi':>7} | {'enc':>7} | {'imu':>7} | "
               f"{'dot':>7} | {'Err':>7} | {'I':>7} | {'u':>7} | {'PWM':>5} | {'new':>4}")
        print(f"\n{hdr}")
        print("-" * len(hdr))

        it      = 0
        t_start = time.monotonic()
        t_last_pid = t_start   # pour dt_actual en mode IMU

        # Init derivee encodeur
        self.encoder.get_psi_and_dot()

        # Dernier u connu (maintenu si on skip une iteration IMU)
        last_u      = 0.0
        last_pwm    = ESC_NEUTRAL_US

        try:
            while self.running:
                t0 = time.monotonic()
                it += 1

                psi_enc, psi_dot_enc = self.encoder.get_psi_and_dot()

                # ── Lecture IMU (toujours, pour detection new_data et logging) ──
                psi_imu, is_new = self.imu.get_psi_and_new()
                phi = self.imu.get_phi()

                # ── Selection source selon mode ──
                if self.sensor_mode == "enc":
                    psi      = psi_enc
                    psi_dot  = psi_dot_enc
                    do_pid   = True
                    dt_pid   = DT

                elif self.sensor_mode == "imu":
                    if psi_imu is None or not is_new:
                        # Pas de nouvelle donnee → maintient le dernier PWM,
                        # ne recalcule pas le PID, n'accumule pas l'integrale
                        imu_skip_count += 1
                        self.esc.set_pwm(last_pwm)
                        time.sleep(max(0, DT - (time.monotonic() - t0)))
                        continue
                    imu_new_count += 1
                    psi      = psi_imu
                    psi_dot  = self.imu.get_psi_dot_gyro()
                    do_pid   = True
                    dt_pid   = t0 - t_last_pid  # dt reel depuis dernier calcul
                    t_last_pid = t0

                else:  # fused
                    if psi_imu is not None:
                        psi_fused = FUSED_ALPHA * psi_enc + (1 - FUSED_ALPHA) * psi_imu
                    else:
                        psi_fused = psi_enc
                    psi      = psi_fused
                    psi_dot  = psi_dot_enc
                    do_pid   = True
                    dt_pid   = DT

                if abs(psi) > MAX_PSI_DEG:
                    self._emergency(f"|psi|={abs(psi):.1f} > {MAX_PSI_DEG}")
                    break

                u, err, u_p, u_i, u_d, u_g = self.ctrl.compute(psi, psi_dot,
                                                                  dt_actual=dt_pid)
                pwm_actual = self.esc.set_pwm(self.ctrl.to_pwm(u))
                last_u     = u
                last_pwm   = pwm_actual

                t_elapsed  = t0 - t_start
                imu_log    = str(round(psi_imu, 4)) if psi_imu is not None else ""
                phi_log    = str(round(phi, 4))    if phi    is not None else ""

                self.csv_file.write(
                    f"{t_elapsed:.4f},{it},{self.sensor_mode},{int(is_new)},"
                    f"{psi:.4f},{psi_dot:.4f},{psi_enc:.4f},{imu_log},{phi_log},"
                    f"{self.ctrl.setpoint:.2f},{self.ctrl.rest_angle:.2f},"
                    f"{err:.4f},{self.ctrl._integral:.4f},"
                    f"{u:.6f},{u_p:.6f},{u_i:.6f},{u_d:.6f},{u_g:.6f},{pwm_actual}\n"
                )

                imu_str  = f"{psi_imu:+7.1f}" if psi_imu is not None else "   N/A"
                new_str  = "NEW" if is_new else "---"
                print(
                    f"\r{it:6d} | {psi:+7.1f} | {psi_enc:+7.1f} | {imu_str} | "
                    f"{psi_dot:+7.1f} | {err:+7.1f} | {self.ctrl._integral:+7.2f} | "
                    f"{u:+7.3f} | {pwm_actual:5d} | {new_str}",
                    end="", flush=True
                )

                elapsed = time.monotonic() - t0
                wait = DT - elapsed
                if wait > 0:
                    time.sleep(wait)

        except KeyboardInterrupt:
            print("\n[STOP] Ctrl+C")
        finally:
            if self.sensor_mode == "imu" and (imu_new_count + imu_skip_count) > 0:
                total = imu_new_count + imu_skip_count
                print(f"\n[IMU]  Nouvelles donnees : {imu_new_count}/{total} "
                      f"({100*imu_new_count/total:.0f}%) "
                      f"→ taux effectif ~{imu_new_count/max(1,t0-t_start):.0f} Hz")
            self._cleanup()

    def _cleanup(self):
        print("\n[STOP] Arret moteur...")
        self.esc.stop()
        self.esc.shutdown()
        self.encoder.cancel()
        self.pi.stop()
        if hasattr(self, "csv_file") and self.csv_file:
            self.csv_file.close()
            print(f"[LOG]  CSV → {self.csv_path}")
        print("[STOP] OK")


def main():
    p = argparse.ArgumentParser(
        description="TRMS — PID avec choix du mode capteur",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Modes capteurs :
  enc   : Encodeur seul, 200 Hz (recommande, reference)
  imu   : IMU seul @ {1e6/IMU_INTERVAL_US:.0f} Hz — gyro pour psi_dot, skip si pas de nouvelle donnee
  fused : Filtre complementaire enc+IMU (alpha={FUSED_ALPHA})

Exemples :
  python3 trms_controller.py --sensor-mode enc   --kp 0.8 --ki 0.3 --kd 0.5 --setpoint 90
  python3 trms_controller.py --sensor-mode imu   --kp 0.8 --ki 0.3 --kd 0.5 --setpoint 90
  python3 trms_controller.py --sensor-mode fused --kp 0.8 --ki 0.3 --kd 0.5 --setpoint 90
        """)

    p.add_argument("--sensor-mode", choices=SENSOR_MODES, default="enc")
    p.add_argument("--setpoint",    type=float, default=10.0)
    p.add_argument("--kp",          type=float, default=0.8)
    p.add_argument("--ki",          type=float, default=0.3)
    p.add_argument("--kd",          type=float, default=0.5)
    p.add_argument("--kg",          type=float, default=0.25)
    p.add_argument("--throttle",    type=float, default=0.55)
    p.add_argument("--rest-angle",  type=float, default=None)

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
        sensor_mode         = args.sensor_mode,
    )
    ctrl.run()


if __name__ == "__main__":
    main()