import time
import math
import signal
import sys
import argparse
import os

import pigpio
import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

ESC_PWM_PIN = 18
ESC_NEUTRAL_US = 1500
SAMPLE_FREQ_HZ = 50.0
DT = 1.0 / SAMPLE_FREQ_HZ

running = True

def sig_handler(signum, frame):
    global running
    running = False

signal.signal(signal.SIGINT, sig_handler)
signal.signal(signal.SIGTERM, sig_handler)

def main():
    p = argparse.ArgumentParser(description="Boucle ouverte — reponse indicielle")
    p.add_argument("--pwm", type=int, required=True, help="PWM fixe a envoyer (ex: 1300)")
    p.add_argument("--duration", type=float, default=10.0, help="Duree de l'essai en secondes")
    p.add_argument("--delay", type=float, default=2.0, help="Delai avant echelon (pour calibration IMU)")
    args = p.parse_args()

    pwm_cmd = args.pwm
    duration = args.duration
    delay = args.delay

    print("=" * 60)
    print(f"  BOUCLE OUVERTE — PWM = {pwm_cmd} us")
    print(f"  Duree = {duration}s (+ {delay}s de delai)")
    print("=" * 60)

    pi = pigpio.pi()
    if not pi.connected:
        print("pigpiod non lance ! -> sudo pigpiod")
        sys.exit(1)

    print("[IMU]  Init BNO085...")
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    bno = BNO08X_I2C(i2c, address=0x4A)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    time.sleep(0.5)

    print("[ESC]  Armement...")
    pi.set_servo_pulsewidth(ESC_PWM_PIN, ESC_NEUTRAL_US)
    time.sleep(3)
    print("[ESC]  Pret")

    print(f"\n[CAL]  Calibration IMU ({delay}s)...")
    print("[CAL]  >>> NE PAS TOUCHER LE BRAS <<<")
    samples = []
    t0_cal = time.monotonic()
    while time.monotonic() - t0_cal < delay:
        try:
            qi, qj, qk, qr = bno.quaternion
            sinp = 2.0 * (qr * qj - qk * qi)
            sinp = max(-1.0, min(1.0, sinp))
            pitch = math.degrees(math.asin(sinp))
            samples.append(pitch)
        except:
            pass
        time.sleep(DT)

    if len(samples) < 10:
        print("[CAL]  ERREUR pas assez d'echantillons")
        pi.set_servo_pulsewidth(ESC_PWM_PIN, 0)
        pi.stop()
        sys.exit(1)

    pitch_offset = sum(samples) / len(samples)
    print(f"[CAL]  Offset = {pitch_offset:.2f} deg")

    from datetime import datetime
    os.makedirs("logs", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"logs/open_loop_pwm{pwm_cmd}_{ts}.csv"
    csv_file = open(csv_path, "w")
    csv_file.write("t_s,psi_deg,psi_dot_dps,pwm_us\n")
    print(f"[LOG]  CSV -> {csv_path}")

    print(f"\n[RUN]  Phase 1: repos (1s)...")
    t_start = time.monotonic()
    phase = "repos"

    global running
    it = 0

    hdr = f"{'t':>7} | {'psi':>7} | {'dot':>7} | {'PWM':>5} | {'phase':>8}"
    print(f"\n{hdr}")
    print("-" * len(hdr))

    try:
        while running:
            t_now = time.monotonic()
            t_elapsed = t_now - t_start
            it += 1

            if t_elapsed < 1.0:
                phase = "repos"
                pw = ESC_NEUTRAL_US
            elif t_elapsed < 1.0 + duration:
                if phase == "repos":
                    print(f"\n[RUN]  Phase 2: echelon PWM = {pwm_cmd} us !")
                    phase = "echelon"
                pw = pwm_cmd
            else:
                if phase == "echelon":
                    print(f"\n[RUN]  Phase 3: arret moteur")
                    phase = "arret"
                pw = ESC_NEUTRAL_US
                if t_elapsed > 1.0 + duration + 3.0:
                    break

            pi.set_servo_pulsewidth(ESC_PWM_PIN, pw)

            try:
                qi, qj, qk, qr = bno.quaternion
                sinp = 2.0 * (qr * qj - qk * qi)
                sinp = max(-1.0, min(1.0, sinp))
                pitch_raw = math.degrees(math.asin(sinp))
                psi = -(pitch_raw - pitch_offset)
            except:
                psi = None

            try:
                _, gy, _ = bno.gyro
                psi_dot = -math.degrees(gy)
            except:
                psi_dot = 0.0

            if psi is not None:
                csv_file.write(f"{t_elapsed:.4f},{psi:.4f},{psi_dot:.4f},{pw}\n")

                if it % 5 == 0:
                    line = f"\r{t_elapsed:7.2f} | {psi:+7.1f} | {psi_dot:+7.1f} | {pw:5d} | {phase:>8}"
                    print(line, end="", flush=True)

            elapsed = time.monotonic() - t_now
            wait = DT - elapsed
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C")

    finally:
        pi.set_servo_pulsewidth(ESC_PWM_PIN, ESC_NEUTRAL_US)
        time.sleep(0.3)
        pi.set_servo_pulsewidth(ESC_PWM_PIN, 0)
        pi.stop()
        csv_file.close()
        print(f"\n[LOG]  CSV sauvegarde -> {csv_path}")
        print("[STOP] Termine")

if __name__ == "__main__":
    main()