#!/usr/bin/env python3
"""
test_sinusoidal_tracking.py

Tracking d'une consigne sinusoidale autour d'un point central.
Permet d'identifier experimentalement la bande passante du systeme
controle en boucle fermee.

Setpoint(t) = sp_center + sp_amplitude * sin(2*pi*f*t)

On enregistre l'angle mesure pour chaque frequence et on compare
amplitude/phase. Plusieurs frequences sont testees a la suite.

Usage:
    sudo pigpiod
    python3 test_sinusoidal_tracking.py \\
        --kp 0.8 --ki 0.3 --kd 0.5 --kg 0.85 \\
        --center 20 --amplitude 5 \\
        --freqs 0.1 0.2 0.5 1.0 2.0
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
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

# ============================================================================
# CONSTANTES HARDWARE (identiques a trms_controller.py)
# ============================================================================

ENCODER_PIN_A    = 27
ENCODER_PIN_B    = 17
ESC_MAIN_PWM_PIN = 12

ENCODER_CPR_X4   = 2000
ESC_NEUTRAL_US   = 1500
ESC_MIN_US       = 1100
ESC_MAX_US       = 1900
ESC_DEADBAND_US  = 25
ESC_SLEW_MAX_US  = 2
ESC_DWELL_S      = 0.5

CONTROL_FREQ_HZ  = 200.0
DT               = 1.0 / CONTROL_FREQ_HZ
MAX_PSI_DEG      = 100.0

CALIB_SAMPLES    = 50
CALIB_PERIOD_S   = 2.0

PSI_DOT_ALPHA    = 0.3

THROTTLE_LIMIT   = 0.95

# Tracking
SETTLE_DURATION_S    = 3.0   # temps pour atteindre le centre avant la sinusoide
COOLDOWN_BETWEEN_S   = 5.0   # repos entre deux frequences (esc neutre)
MIN_CYCLES_PER_FREQ  = 4     # au moins 4 cycles par frequence
MIN_DURATION_PER_FREQ_S = 8.0  # ou 8 s minimum


# ============================================================================
# CLASSES (copiees telles quelles de trms_controller.py)
# ============================================================================

class IMU_BNO085:

    def __init__(self, need_gyro=False):
        print("[IMU]  Initialisation BNO085 sur I2C @ 0x4A...")
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        self.bno = BNO08X_I2C(self.i2c, address=0x4A)
        time.sleep(0.5)
        try:
            self.bno.soft_reset()
        except AttributeError:
            pass
        time.sleep(1.0)

        self._enable_feature_safe(BNO_REPORT_GAME_ROTATION_VECTOR,
                                   interval_us=5000)
        time.sleep(0.5)

        self.pitch_offset = 0.0
        self._last_quat = None
        print(f"[IMU]  BNO085 pret")

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
        sinp  = 2.0 * (qr * qj - qk * qi)
        sinp  = max(-1.0, min(1.0, sinp))
        pitch = math.degrees(math.asin(sinp))
        return pitch

    def _get_raw_pitch(self):
        try:
            quat = self.bno.game_quaternion
            if quat is None:
                return None
            return self._quat_to_euler(*quat)
        except Exception:
            return None

    def calibrate(self, n_samples=CALIB_SAMPLES, duration_s=CALIB_PERIOD_S):
        print(f"[IMU]  Calibration ({duration_s}s)...")
        print("[IMU]  >>> BRAS IMMOBILE AU REPOS <<<")
        samples = []
        dt = duration_s / n_samples
        for _ in range(n_samples):
            p = self._get_raw_pitch()
            if p is not None:
                samples.append(p)
            time.sleep(dt)
        if len(samples) < 5:
            return False
        self.pitch_offset = sum(samples) / len(samples)
        print(f"[IMU]  pitch_offset = {self.pitch_offset:.2f} deg")
        return True

    def get_rest_angle_from_vertical(self):
        return abs(self.pitch_offset)


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
        print(f"[ENC]  HEDS-5540 pret")

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
        print(f"[{name}]  Arme")

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

    def __init__(self, kp, ki, kd, kg, rest_angle_deg=85.0):
        self.kp           = kp
        self.ki           = ki
        self.kd           = kd
        self.kg           = kg
        self.rest_angle   = rest_angle_deg
        self._integral    = 0.0
        self._integral_max = 400

    def reset_integral(self):
        self._integral = 0.0

    def compute(self, psi_deg, psi_dot_dps, setpoint_deg, dt_actual=None):
        dt = dt_actual if dt_actual is not None else DT
        error = setpoint_deg - psi_deg

        u_p = self.kp * error
        u_d = -self.kd * psi_dot_dps
        phys_angle_rad = math.radians(self.rest_angle + setpoint_deg)
        u_g = self.kg * math.sin(phys_angle_rad) * 90.0

        u_i_prev = self.ki * self._integral
        u_test_norm = (u_p + u_i_prev + u_d + u_g) / 90.0

        saturated_high = u_test_norm >  THROTTLE_LIMIT and error > 0
        saturated_low  = u_test_norm < -THROTTLE_LIMIT and error < 0
        if not (saturated_high or saturated_low):
            self._integral += error * dt
            self._integral = max(-self._integral_max,
                                  min(self._integral_max, self._integral))

        u_i = self.ki * self._integral
        u_raw  = u_p + u_i + u_d + u_g
        u_norm = max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, u_raw / 90.0))

        return u_norm, error, u_p / 90.0, u_i / 90.0, u_d / 90.0, u_g / 90.0

    def to_pwm(self, u_norm):
        if u_norm <= 0:
            return ESC_NEUTRAL_US
        span = ESC_NEUTRAL_US - ESC_DEADBAND_US - ESC_MIN_US
        pw = ESC_NEUTRAL_US - ESC_DEADBAND_US - u_norm * span
        return int(max(ESC_MIN_US, pw))


# ============================================================================
# SINUSOIDAL TRACKING TEST
# ============================================================================

class SinusoidalTracker:

    def __init__(self, kp, ki, kd, kg, center_deg, amplitude_deg,
                 frequencies_hz, rest_angle_override=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kg = kg
        self.center = center_deg
        self.amplitude = amplitude_deg
        self.frequencies = frequencies_hz
        self.running = False

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod non lance ! → sudo pigpiod")

        print("=" * 70)
        print(f"  TRACKING SINUSOIDAL")
        print(f"  Centre = {center_deg:.1f} deg, Amplitude = +/- {amplitude_deg:.1f} deg")
        print(f"  Plage  = [{center_deg-amplitude_deg:.1f}, "
              f"{center_deg+amplitude_deg:.1f}] deg")
        print(f"  Frequences: {frequencies_hz} Hz")
        print(f"  Kp={kp:.2f}  Ki={ki:.3f}  Kd={kd:.2f}  Kg={kg:.2f}")
        print("=" * 70)

        # Safety check
        if center_deg - amplitude_deg < 5:
            print(f"[WARN] center - amplitude = {center_deg - amplitude_deg:.1f} "
                  f"deg, risque de descendre trop bas")
        if center_deg + amplitude_deg > 80:
            print(f"[WARN] center + amplitude = {center_deg + amplitude_deg:.1f} "
                  f"deg, risque de saturation moteur")

        self.imu = IMU_BNO085(need_gyro=False)
        self.encoder = EncoderHEDS5540(self.pi)
        self.esc = ESC(self.pi, ESC_MAIN_PWM_PIN, name="ESC_MAIN")
        time.sleep(1.0)

        print("\n  *** CALIBRATION IMU : bras au repos ***\n")
        if not self.imu.calibrate():
            self._abort()
            raise RuntimeError("Echec calibration IMU")

        rest_angle = (rest_angle_override if rest_angle_override is not None
                      else self.imu.get_rest_angle_from_vertical())
        print(f"[CAL]  rest_angle = {rest_angle:.1f} deg")

        self.ctrl = PIDController(kp, ki, kd, kg, rest_angle_deg=rest_angle)
        self.encoder.reset()

        from datetime import datetime
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = (
            f"logs/trms_sine_kp{kp}_ki{ki}_kd{kd}_kg{kg}"
            f"_c{center_deg}_a{amplitude_deg}_ra{rest_angle:.0f}_{ts}.csv"
        )
        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write(
            "t_s,iteration,freq_hz,phase,"
            "setpoint_deg,psi_deg,psi_dot_dps,"
            "error_deg,integral_deg_s,"
            "u_norm,u_p,u_i,u_d,u_g,pwm_us\n"
        )
        print(f"[LOG]  CSV → {self.csv_path}")

        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

        print("\n" + "=" * 70)
        print("  Pret. Ctrl+C pour arret.")
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

    def _run_phase(self, label, duration_s, freq_hz, target_fn):
        """
        Boucle de controle generique sur une phase.

        target_fn(t) renvoie le setpoint instantane en degres.
        """
        t_phase_start = time.monotonic()
        it = 0
        t0 = t_phase_start

        while self.running:
            t0 = time.monotonic()
            t_phase = t0 - t_phase_start
            if t_phase >= duration_s:
                break
            it += 1

            psi, psi_dot = self.encoder.get_psi_and_dot()

            if abs(psi) > MAX_PSI_DEG:
                self._emergency(f"|psi|={abs(psi):.1f} > {MAX_PSI_DEG}")
                return

            setpoint = target_fn(t_phase)
            u, err, u_p, u_i, u_d, u_g = self.ctrl.compute(
                psi, psi_dot, setpoint, dt_actual=DT)
            pwm = self.esc.set_pwm(self.ctrl.to_pwm(u))

            t_global = t0  # absolute time (monotonic), retraite plus tard
            self.csv_file.write(
                f"{t0:.4f},{it},{freq_hz:.4f},{label},"
                f"{setpoint:.4f},{psi:.4f},{psi_dot:.4f},"
                f"{err:.4f},{self.ctrl._integral:.4f},"
                f"{u:.6f},{u_p:.6f},{u_i:.6f},{u_d:.6f},{u_g:.6f},{pwm}\n"
            )

            if it % 40 == 0:  # affichage toutes les 200 ms
                print(f"\r  [{label}] f={freq_hz:.2f}Hz  t={t_phase:5.2f}s  "
                      f"sp={setpoint:+6.2f}  psi={psi:+6.2f}  "
                      f"err={err:+6.2f}  pwm={pwm}",
                      end="", flush=True)

            elapsed = time.monotonic() - t0
            wait = DT - elapsed
            if wait > 0:
                time.sleep(wait)

        print()  # newline after phase

    def run(self):
        self.running = True
        try:
            # Phase 0 : ramene au centre avant la sequence
            print(f"\n[PHASE 0] Stabilisation au centre ({self.center:.1f} deg) "
                  f"pendant {SETTLE_DURATION_S}s")
            self._run_phase("settle_init", SETTLE_DURATION_S, 0.0,
                            lambda t: self.center)
            if not self.running:
                return

            # Pour chaque frequence
            for freq in self.frequencies:
                if not self.running:
                    break

                duration = max(MIN_DURATION_PER_FREQ_S,
                               MIN_CYCLES_PER_FREQ / freq)

                print(f"\n[FREQ] f = {freq:.2f} Hz  |  duree = {duration:.1f}s  "
                      f"|  cycles = {freq*duration:.1f}")

                # Reset integral entre frequences pour eviter offset
                self.ctrl.reset_integral()

                # Phase A : stabilisation au centre
                print(f"  Phase A: stabilisation au centre {SETTLE_DURATION_S}s")
                self._run_phase("settle_A", SETTLE_DURATION_S, freq,
                                lambda t: self.center)
                if not self.running:
                    break

                # Phase B : sinusoide
                print(f"  Phase B: sinusoide @ {freq} Hz pendant {duration:.1f}s")
                self._run_phase("sine", duration, freq,
                                lambda t, f=freq: self.center +
                                self.amplitude * math.sin(2 * math.pi * f * t))
                if not self.running:
                    break

                # Phase C : retour au centre
                print(f"  Phase C: retour au centre {SETTLE_DURATION_S}s")
                self._run_phase("settle_C", SETTLE_DURATION_S, freq,
                                lambda t: self.center)
                if not self.running:
                    break

                # Cooldown ESC neutre entre frequences (sauf derniere)
                if freq != self.frequencies[-1]:
                    print(f"  Cooldown {COOLDOWN_BETWEEN_S}s (ESC neutre)")
                    t_cool = time.monotonic()
                    while time.monotonic() - t_cool < COOLDOWN_BETWEEN_S:
                        self.esc.set_pwm(ESC_NEUTRAL_US)
                        time.sleep(DT)

        except KeyboardInterrupt:
            print("\n[STOP] Ctrl+C")
        finally:
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
        description="TRMS — Tracking sinusoidal multi-frequence")

    p.add_argument("--kp",        type=float, default=0.8)
    p.add_argument("--ki",        type=float, default=0.3)
    p.add_argument("--kd",        type=float, default=0.5)
    p.add_argument("--kg",        type=float, default=0.85)
    p.add_argument("--center",    type=float, default=20.0,
                   help="Angle central de la sinusoide (deg)")
    p.add_argument("--amplitude", type=float, default=5.0,
                   help="Amplitude de la sinusoide autour du centre (deg)")
    p.add_argument("--freqs",     type=float, nargs="+",
                   default=[0.1, 0.2, 0.5, 1.0, 2.0],
                   help="Liste de frequences a tester (Hz)")
    p.add_argument("--throttle",  type=float, default=1.0)
    p.add_argument("--rest-angle", type=float, default=None)

    args = p.parse_args()

    global THROTTLE_LIMIT
    THROTTLE_LIMIT = max(0.05, min(1.0, args.throttle))

    tracker = SinusoidalTracker(
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        kg=args.kg,
        center_deg=args.center,
        amplitude_deg=args.amplitude,
        frequencies_hz=args.freqs,
        rest_angle_override=args.rest_angle,
    )
    tracker.run()


if __name__ == "__main__":
    main()