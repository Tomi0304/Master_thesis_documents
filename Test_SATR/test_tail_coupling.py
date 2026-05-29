#!/usr/bin/env python3
"""
test_tail_coupling.py

Caracterisation du couplage parasite tail rotor -> pitch sur le SATR.

Le main rotor reste a l'arret (PWM=1500 neutre).
Le tail rotor est pilote en open loop par paliers de PWM.
On enregistre l'angle pitch (encodeur) en continu.

Si le SATR est purement decouple, le pitch reste a 0.
Sinon, on quantifie le couplage en deg vs PWM tail.

GPIO 12 : Main rotor  (maintenu neutre)
GPIO 13 : Tail rotor  (pilote en open loop)

Usage:
    sudo pigpiod
    python3 test_tail_coupling.py [--pwm-start 1500] [--pwm-end 1100] [--step 50] [--dwell 3.0]
"""

import time
import signal
import argparse
import threading
import os
from datetime import datetime

import pigpio

# Hardware (identique a trms_controller.py)
ENCODER_PIN_A    = 27
ENCODER_PIN_B    = 17
ESC_MAIN_PWM_PIN = 12
ESC_TAIL_PWM_PIN = 13

ENCODER_CPR_X4   = 2000
ESC_NEUTRAL_US   = 1500
ESC_MIN_US       = 1100
ESC_MAX_US       = 1900

CONTROL_FREQ_HZ  = 200.0
DT               = 1.0 / CONTROL_FREQ_HZ
MAX_PSI_DEG      = 100.0
SAFETY_PITCH_DEG = 90.0   # arret d'urgence si pitch depasse


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

    def reset(self):
        with self._lock:
            self._position = 0

    def cancel(self):
        self._cb_a.cancel()
        self._cb_b.cancel()


class TailCouplingTest:

    def __init__(self, pwm_start, pwm_end, step, dwell_s):
        self.pwm_start = pwm_start
        self.pwm_end   = pwm_end
        self.step      = step if pwm_end > pwm_start else -abs(step)
        self.dwell_s   = dwell_s

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod non lance ! → sudo pigpiod")

        # Build sequence
        self.pwm_sequence = []
        pwm = pwm_start
        while (self.step > 0 and pwm <= pwm_end) or (self.step < 0 and pwm >= pwm_end):
            self.pwm_sequence.append(pwm)
            pwm += self.step
        # Add return to neutral
        if self.pwm_sequence[-1] != ESC_NEUTRAL_US:
            self.pwm_sequence.append(ESC_NEUTRAL_US)

        print("=" * 70)
        print(f"  TAIL ROTOR COUPLING TEST")
        print(f"  Main rotor: NEUTRAL ({ESC_NEUTRAL_US} us, GPIO {ESC_MAIN_PWM_PIN})")
        print(f"  Tail rotor: OPEN-LOOP RAMP (GPIO {ESC_TAIL_PWM_PIN})")
        print(f"  Sequence: {self.pwm_sequence}")
        print(f"  Dwell per step: {dwell_s}s")
        print(f"  Total duration: ~{len(self.pwm_sequence)*dwell_s:.0f}s")
        print(f"  Safety pitch cutoff: ±{SAFETY_PITCH_DEG}°")
        print("=" * 70)

        # Encoder
        self.encoder = EncoderHEDS5540(self.pi)

        # Arm both ESCs (neutral)
        print(f"\n[ESC]  Arming MAIN (GPIO {ESC_MAIN_PWM_PIN}) at {ESC_NEUTRAL_US} us...")
        self.pi.set_servo_pulsewidth(ESC_MAIN_PWM_PIN, ESC_NEUTRAL_US)
        print(f"[ESC]  Arming TAIL (GPIO {ESC_TAIL_PWM_PIN}) at {ESC_NEUTRAL_US} us...")
        self.pi.set_servo_pulsewidth(ESC_TAIL_PWM_PIN, ESC_NEUTRAL_US)
        time.sleep(3)
        print(f"[ESC]  Both ESCs armed and ready\n")

        self.running = False
        self.encoder.reset()
        print("[ENC]  Position remise a zero\n")

        # CSV log
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = (f"logs/tail_coupling_main{ESC_NEUTRAL_US}_"
                         f"tail{pwm_start}-{pwm_end}_step{self.step}_"
                         f"dwell{int(dwell_s*1000)}ms_{ts}.csv")
        self.csv_file = open(self.csv_path, "w")
        self.csv_file.write("t_s,iteration,tail_pwm_us,main_pwm_us,psi_pitch_deg\n")
        print(f"[LOG]  CSV → {self.csv_path}\n")

        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

    def _sig(self, signum, frame):
        print("\n[STOP] Arret demande...")
        self.running = False

    def _abort(self):
        self.pi.set_servo_pulsewidth(ESC_MAIN_PWM_PIN, ESC_NEUTRAL_US)
        self.pi.set_servo_pulsewidth(ESC_TAIL_PWM_PIN, ESC_NEUTRAL_US)

    def run(self):
        self.running = True
        it = 0
        t_global_start = time.monotonic()

        try:
            for step_idx, tail_pwm in enumerate(self.pwm_sequence):
                if not self.running:
                    break

                print(f"\n[STEP {step_idx+1}/{len(self.pwm_sequence)}] "
                      f"Tail PWM = {tail_pwm} us  ({self.dwell_s}s)")

                # Set tail PWM (instant, no slew - just for characterization)
                self.pi.set_servo_pulsewidth(ESC_TAIL_PWM_PIN, tail_pwm)

                # Log for dwell_s seconds at 200 Hz
                t_step_start = time.monotonic()
                psi_max_step = 0.0
                psi_mean_accum = 0.0
                psi_mean_count = 0

                while time.monotonic() - t_step_start < self.dwell_s:
                    if not self.running:
                        break
                    t0 = time.monotonic()
                    it += 1
                    t_elapsed = t0 - t_global_start

                    psi = self.encoder.get_psi()

                    # Safety
                    if abs(psi) > SAFETY_PITCH_DEG:
                        print(f"\n[!!!] PITCH EMERGENCY: |psi|={abs(psi):.1f}° > {SAFETY_PITCH_DEG}°")
                        self._abort()
                        self.running = False
                        break

                    self.csv_file.write(
                        f"{t_elapsed:.4f},{it},{tail_pwm},{ESC_NEUTRAL_US},{psi:.4f}\n"
                    )

                    if abs(psi) > abs(psi_max_step):
                        psi_max_step = psi
                    psi_mean_accum += psi
                    psi_mean_count += 1

                    # Live display every 200 ms
                    if it % 40 == 0:
                        print(f"\r  t={t_elapsed:5.1f}s  tail_pwm={tail_pwm}  "
                              f"psi={psi:+6.2f}°  max_so_far={psi_max_step:+6.2f}°",
                              end="", flush=True)

                    elapsed = time.monotonic() - t0
                    wait = DT - elapsed
                    if wait > 0:
                        time.sleep(wait)

                psi_mean_step = psi_mean_accum / psi_mean_count if psi_mean_count else 0
                print(f"\n  → tail_pwm={tail_pwm}: psi_mean={psi_mean_step:+.2f}°  "
                      f"psi_max={psi_max_step:+.2f}°")

        except KeyboardInterrupt:
            print("\n[STOP] Ctrl+C")
        finally:
            self._cleanup()

    def _cleanup(self):
        print("\n[STOP] Arret moteurs (les deux a neutre)...")
        self.pi.set_servo_pulsewidth(ESC_MAIN_PWM_PIN, ESC_NEUTRAL_US)
        self.pi.set_servo_pulsewidth(ESC_TAIL_PWM_PIN, ESC_NEUTRAL_US)
        time.sleep(1)
        self.pi.set_servo_pulsewidth(ESC_MAIN_PWM_PIN, 0)
        self.pi.set_servo_pulsewidth(ESC_TAIL_PWM_PIN, 0)
        self.encoder.cancel()
        self.pi.stop()
        if hasattr(self, "csv_file") and self.csv_file:
            self.csv_file.close()
            print(f"[LOG]  CSV → {self.csv_path}")
        print("[STOP] OK")


def main():
    p = argparse.ArgumentParser(
        description="SATR — Tail rotor coupling characterization")
    p.add_argument("--pwm-start", type=int, default=1500,
                   help="Initial tail PWM (default: 1500 = neutral)")
    p.add_argument("--pwm-end", type=int, default=1100,
                   help="Final tail PWM (default: 1100 = max reverse). "
                        "Use 1900 for forward direction.")
    p.add_argument("--step", type=int, default=50,
                   help="PWM step size (default: 50)")
    p.add_argument("--dwell", type=float, default=3.0,
                   help="Dwell time per PWM step in seconds (default: 3.0)")

    args = p.parse_args()

    test = TailCouplingTest(
        pwm_start=args.pwm_start,
        pwm_end=args.pwm_end,
        step=args.step,
        dwell_s=args.dwell,
    )
    test.run()


if __name__ == "__main__":
    main()