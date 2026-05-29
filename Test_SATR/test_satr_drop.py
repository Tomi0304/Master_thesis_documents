#!/usr/bin/env python3
"""
test_satr_drop.py

Drop test passif pour identification J et b sur le SATR.

ATTENTION : couper l'alimentation des moteurs/ESC avant de lancer.
Le script ne pilote AUCUN moteur, il ne fait qu'enregistrer l'angle pitch.

Procedure:
  1. Couper alim ESC/moteurs
  2. Lancer le script
  3. Attendre "READY" 
  4. Tenir le beam a l'angle souhaite (~30-50°)
  5. Lacher proprement
  6. Laisser osciller librement
  7. Ctrl+C apres ~8 secondes

Plusieurs runs successifs sont recommandes.

Usage:
    sudo pigpiod
    python3 test_satr_drop.py --run 1 --duration 8
"""

import time
import signal
import argparse
import threading
import os
from datetime import datetime

import pigpio

ENCODER_PIN_A    = 27
ENCODER_PIN_B    = 17
ENCODER_CPR_X4   = 2000

SAMPLE_FREQ_HZ   = 200.0
DT               = 1.0 / SAMPLE_FREQ_HZ


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


def main():
    parser = argparse.ArgumentParser(description="SATR drop test")
    parser.add_argument("--run", type=int, default=1, help="Run number for filename")
    parser.add_argument("--duration", type=float, default=8.0,
                        help="Recording duration in seconds")
    args = parser.parse_args()

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpiod non lance ! → sudo pigpiod")

    print("=" * 60)
    print(f"  SATR DROP TEST — Run {args.run}")
    print(f"  Duration: {args.duration}s @ {SAMPLE_FREQ_HZ:.0f} Hz")
    print("=" * 60)
    print()
    print("  ⚠️  ALIMENTATION ESC/MOTEURS COUPEE ?")
    print()

    encoder = EncoderHEDS5540(pi)
    encoder.reset()

    # CSV log
    os.makedirs("logs", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"logs/satr_drop_run{args.run:02d}_{ts}.csv"
    csv_file = open(csv_path, "w")
    csv_file.write("t_s,angle_deg\n")
    print(f"  CSV → {csv_path}\n")

    # Wait for user to position
    print("  → Placez le beam a la position INITIALE souhaitee.")
    print("  → Maintenez-le immobile.")
    print("  → Appuyez sur ENTRER quand pret a relacher.\n")
    input("  Pret ? [ENTREE pour confirmer] ")

    # Reset encoder at "zero" reference (current position)
    # NOTE: on garde la position courante comme reference (0 = position au lacher)
    print()
    print("  3...", end="", flush=True)
    time.sleep(1)
    print(" 2...", end="", flush=True)
    time.sleep(1)
    print(" 1...", end="", flush=True)
    time.sleep(1)
    print(" GO !\n")

    # Note: NE PAS reset encodeur. La position au lacher devient implicite
    # via le premier echantillon.

    stop_flag = {'stop': False}
    def sig_handler(signum, frame):
        stop_flag['stop'] = True
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    # Acquisition loop
    t_start = time.monotonic()
    it = 0
    try:
        while not stop_flag['stop']:
            t0 = time.monotonic()
            t_elapsed = t0 - t_start

            if t_elapsed >= args.duration:
                break

            psi = encoder.get_psi()
            csv_file.write(f"{t_elapsed:.4f},{psi:.4f}\n")
            it += 1

            if it % 40 == 0:
                print(f"\r  t={t_elapsed:5.2f}s  psi={psi:+7.2f}°", end="", flush=True)

            elapsed = time.monotonic() - t0
            wait = DT - elapsed
            if wait > 0:
                time.sleep(wait)

        print(f"\n\n  ✅ Acquisition terminee ({it} samples, {t_elapsed:.2f}s)")

    except KeyboardInterrupt:
        print(f"\n\n  [STOP] Ctrl+C ({it} samples, {t_elapsed:.2f}s)")
    finally:
        encoder.cancel()
        pi.stop()
        csv_file.close()
        print(f"  CSV → {csv_path}\n")


if __name__ == "__main__":
    main()