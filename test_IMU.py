#!/usr/bin/env python3
"""
test_imu_axis_alignment.py

But :
  Tester l'IMU BNO085 seule, sans moteur, pour voir si son axe "pitch"
  est bien aligne avec l'axe de rotation du bras.

Principe :
  1) Calibration au repos : on met roll/pitch/yaw relatifs a zero.
  2) Tu bouges le bras lentement a la main, par exemple 0 -> 20/30 deg -> 0.
  3) Le script logge les variations relatives :
       d_roll, d_pitch, d_yaw
  4) Il indique quel axe bouge le plus et si les axes parasites bougent trop.

Interpretation :
  - Si l'IMU est bien alignee et que tu utilises le bon angle,
    un axe doit dominer clairement.
  - Exemple ideal si pitch est bon :
       range(d_pitch) grand
       range(d_roll), range(d_yaw) faibles
  - Si roll/yaw bougent beaucoup, l'angle Euler choisi n'est pas une bonne
    representation directe de l'angle du bras.

Utilisation :
  sudo pigpiod  # seulement si tu utilises --with-encoder
  python3 test_imu_axis_alignment.py --duration 20 --vector game

Option avec encodeur :
  python3 test_imu_axis_alignment.py --duration 20 --vector game --with-encoder

Notes :
  - Par defaut, le script utilise GAME_ROTATION_VECTOR.
  - I2C est force a 400 kHz.
  - Aucun PWM moteur n'est active.
"""

import argparse
import csv
import math
import os
import time
import threading
from datetime import datetime

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_ROTATION_VECTOR

try:
    import pigpio
except ImportError:
    pigpio = None


# ==============================
# Config par defaut
# ==============================

IMU_ADDR = 0x4A
I2C_FREQ_HZ = 400_000
IMU_INTERVAL_US = 5000  # 200 Hz demande

ENCODER_PIN_A = 27
ENCODER_PIN_B = 17
ENCODER_CPR_X4 = 2000


# ==============================
# Utils
# ==============================

def wrap180(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def quat_to_euler(qi, qj, qk, qr):
    """
    Quaternion BNO08x Adafruit : (i, j, k, real)
    Retour : roll, pitch, yaw en degres.
    """
    # Roll X
    sinr_cosp = 2.0 * (qr * qi + qj * qk)
    cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    # Pitch Y
    sinp = 2.0 * (qr * qj - qk * qi)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    # Yaw Z
    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


def mean(values):
    return sum(values) / len(values) if values else 0.0


def rms(values):
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def data_range(values):
    if not values:
        return 0.0
    return max(values) - min(values)


# ==============================
# Encodeur optionnel
# ==============================

class OptionalEncoder:
    _QUAD_TABLE = [
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    ]

    def __init__(self):
        if pigpio is None:
            raise RuntimeError("pigpio n'est pas installe/importable.")
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod non lance. Lance : sudo pigpiod")

        self._position = 0
        self._lock = threading.Lock()

        self.pi.set_mode(ENCODER_PIN_A, pigpio.INPUT)
        self.pi.set_mode(ENCODER_PIN_B, pigpio.INPUT)
        self.pi.set_pull_up_down(ENCODER_PIN_A, pigpio.PUD_OFF)
        self.pi.set_pull_up_down(ENCODER_PIN_B, pigpio.PUD_OFF)

        a = self.pi.read(ENCODER_PIN_A)
        b = self.pi.read(ENCODER_PIN_B)
        self._last_state = (a << 1) | b

        self._cb_a = self.pi.callback(ENCODER_PIN_A, pigpio.EITHER_EDGE, self._cb)
        self._cb_b = self.pi.callback(ENCODER_PIN_B, pigpio.EITHER_EDGE, self._cb)

        print(f"[ENC] Encodeur actif A=GPIO{ENCODER_PIN_A}, B=GPIO{ENCODER_PIN_B}")

    def _cb(self, gpio, level, tick):
        a = self.pi.read(ENCODER_PIN_A)
        b = self.pi.read(ENCODER_PIN_B)
        current = (a << 1) | b
        idx = (self._last_state << 2) | current
        with self._lock:
            self._position += self._QUAD_TABLE[idx]
        self._last_state = current

    def reset(self):
        with self._lock:
            self._position = 0

    def get_angle_deg(self):
        with self._lock:
            pos = self._position
        return -((pos / ENCODER_CPR_X4) * 360.0)

    def close(self):
        self._cb_a.cancel()
        self._cb_b.cancel()
        self.pi.stop()


# ==============================
# Main
# ==============================

def main():
    parser = argparse.ArgumentParser(
        description="Test alignement axe IMU BNO085 sur bras AP/TRMS"
    )
    parser.add_argument("--duration", type=float, default=20.0,
                        help="Duree de mesure apres calibration [s]")
    parser.add_argument("--calib-duration", type=float, default=2.0,
                        help="Duree de calibration au repos [s]")
    parser.add_argument("--rate", type=float, default=200.0,
                        help="Frequence cible de la boucle de lecture [Hz]")
    parser.add_argument("--vector", choices=["game", "rotation"], default="game",
                        help="Type de quaternion BNO085")
    parser.add_argument("--with-encoder", action="store_true",
                        help="Logge aussi l'encodeur HEDS si disponible")
    parser.add_argument("--outdir", default="logs",
                        help="Dossier de sortie CSV")
    args = parser.parse_args()

    dt = 1.0 / args.rate

    print("=" * 72)
    print("TEST ALIGNEMENT AXE IMU BNO085")
    print("Moteur/PWM non utilise. Bouge le bras lentement a la main.")
    print(f"Vector mode       : {args.vector}")
    print(f"IMU interval      : {IMU_INTERVAL_US} us -> {1e6/IMU_INTERVAL_US:.0f} Hz demande")
    print(f"Loop target       : {args.rate:.0f} Hz")
    print(f"Calibration       : {args.calib_duration:.1f} s")
    print(f"Measurement       : {args.duration:.1f} s")
    print("=" * 72)

    # IMU init
    print("[IMU] Initialisation I2C...")
    i2c = busio.I2C(board.SCL, board.SDA, frequency=I2C_FREQ_HZ)
    bno = BNO08X_I2C(i2c, address=IMU_ADDR)

    time.sleep(0.5)
    try:
        bno.soft_reset()
    except AttributeError:
        pass
    time.sleep(1.0)

    if args.vector == "game":
        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, IMU_INTERVAL_US)
        quat_getter = lambda: bno.game_quaternion
        print("[IMU] GAME_ROTATION_VECTOR active")
    else:
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, IMU_INTERVAL_US)
        quat_getter = lambda: bno.quaternion
        print("[IMU] ROTATION_VECTOR active")

    time.sleep(0.5)

    # Encoder optional
    encoder = None
    if args.with_encoder:
        encoder = OptionalEncoder()
        encoder.reset()

    # Calibration
    print()
    print("[CAL] Garde le bras immobile a sa position de repos...")
    calib_roll, calib_pitch, calib_yaw = [], [], []
    t_calib_start = time.monotonic()

    while time.monotonic() - t_calib_start < args.calib_duration:
        quat = quat_getter()
        if quat is not None:
            r, p, y = quat_to_euler(*quat)
            calib_roll.append(r)
            calib_pitch.append(p)
            calib_yaw.append(y)
        time.sleep(dt)

    if len(calib_pitch) < 5:
        raise RuntimeError("[CAL] Pas assez d'echantillons IMU pour calibrer.")

    roll0 = mean(calib_roll)
    pitch0 = mean(calib_pitch)
    yaw0 = mean(calib_yaw)

    print(f"[CAL] roll0  = {roll0:+.2f} deg")
    print(f"[CAL] pitch0 = {pitch0:+.2f} deg")
    print(f"[CAL] yaw0   = {yaw0:+.2f} deg")
    print(f"[CAL] spread pitch = {data_range(calib_pitch):.3f} deg")

    input("\nAppuie sur ENTER, puis bouge le bras lentement 0 -> 20/30 deg -> 0...")

    # Output CSV
    os.makedirs(args.outdir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.outdir, f"imu_axis_alignment_{args.vector}_{ts}.csv")

    rows = []
    last_quat = None
    valid_count = 0
    changed_count = 0

    print()
    print("[RUN] Mesure en cours...")
    print(f"[LOG] CSV -> {csv_path}")
    print()

    t_start = time.monotonic()
    t_next_print = t_start

    try:
        while True:
            t0 = time.monotonic()
            t = t0 - t_start
            if t >= args.duration:
                break

            quat = quat_getter()

            valid = quat is not None
            quat_changed = False

            if valid:
                valid_count += 1
                quat_changed = (last_quat is None or quat != last_quat)
                if quat_changed:
                    changed_count += 1
                last_quat = quat

                roll, pitch, yaw = quat_to_euler(*quat)
                d_roll = wrap180(roll - roll0)
                d_pitch = wrap180(pitch - pitch0)
                d_yaw = wrap180(yaw - yaw0)
            else:
                roll = pitch = yaw = None
                d_roll = d_pitch = d_yaw = None

            enc = encoder.get_angle_deg() if encoder is not None else None

            rows.append({
                "t_s": t,
                "valid": int(valid),
                "quat_changed": int(quat_changed),
                "roll_raw_deg": roll,
                "pitch_raw_deg": pitch,
                "yaw_raw_deg": yaw,
                "d_roll_deg": d_roll,
                "d_pitch_deg": d_pitch,
                "d_yaw_deg": d_yaw,
                "enc_deg": enc,
            })

            if t0 >= t_next_print and valid:
                axes = {
                    "roll": abs(d_roll),
                    "pitch": abs(d_pitch),
                    "yaw": abs(d_yaw),
                }
                dominant = max(axes, key=axes.get)
                enc_txt = f" | enc={enc:+6.2f}" if enc is not None else ""
                print(
                    f"\r t={t:5.1f}s | "
                    f"d_roll={d_roll:+7.2f} | d_pitch={d_pitch:+7.2f} | d_yaw={d_yaw:+7.2f} | "
                    f"dom={dominant:>5s}{enc_txt}",
                    end="",
                    flush=True
                )
                t_next_print = t0 + 0.1

            elapsed = time.monotonic() - t0
            wait = dt - elapsed
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C")

    finally:
        if encoder is not None:
            encoder.close()

    print("\n")

    # Save CSV
    fieldnames = [
        "t_s", "valid", "quat_changed",
        "roll_raw_deg", "pitch_raw_deg", "yaw_raw_deg",
        "d_roll_deg", "d_pitch_deg", "d_yaw_deg",
        "enc_deg",
    ]

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    # Analyse
    valid_rows = [r for r in rows if r["valid"]]
    duration = rows[-1]["t_s"] if rows else 0.0

    d_rolls = [r["d_roll_deg"] for r in valid_rows]
    d_pitchs = [r["d_pitch_deg"] for r in valid_rows]
    d_yaws = [r["d_yaw_deg"] for r in valid_rows]

    ranges = {
        "roll": data_range(d_rolls),
        "pitch": data_range(d_pitchs),
        "yaw": data_range(d_yaws),
    }

    rms_vals = {
        "roll": rms(d_rolls),
        "pitch": rms(d_pitchs),
        "yaw": rms(d_yaws),
    }

    dominant = max(ranges, key=ranges.get)
    dominant_range = ranges[dominant]

    print("=" * 72)
    print("RESULTATS")
    print(f"Duree effective               : {duration:.2f} s")
    print(f"Lectures valides              : {valid_count}/{len(rows)} "
          f"-> {valid_count/max(duration,1e-9):.1f} Hz")
    print(f"Quaternions numeriquement diff: {changed_count}/{len(rows)} "
          f"-> {changed_count/max(duration,1e-9):.1f} Hz")
    print()
    print("Amplitude des variations relatives :")
    print(f"  range d_roll  = {ranges['roll']:.2f} deg | RMS = {rms_vals['roll']:.2f}")
    print(f"  range d_pitch = {ranges['pitch']:.2f} deg | RMS = {rms_vals['pitch']:.2f}")
    print(f"  range d_yaw   = {ranges['yaw']:.2f} deg | RMS = {rms_vals['yaw']:.2f}")
    print()
    print(f"Axe dominant selon l'IMU : {dominant.upper()}")

    if dominant_range < 3.0:
        print("[WARN] Mouvement trop faible pour conclure. Refais avec au moins 15-20 deg de mouvement.")
    else:
        other_axes = [a for a in ("roll", "pitch", "yaw") if a != dominant]
        for a in other_axes:
            ratio = ranges[a] / dominant_range if dominant_range > 1e-9 else 0.0
            print(f"  Couplage {a}/dominant = {100*ratio:.1f}%")
            if ratio > 0.35 and ranges[a] > 5.0:
                print(f"  [WARN] {a} bouge beaucoup : l'axe IMU n'est pas proprement aligne "
                      f"ou les angles Euler sont mal conditionnes.")

    if encoder is not None:
        enc_values = [r["enc_deg"] for r in valid_rows if r["enc_deg"] is not None]
        if enc_values:
            enc_range = data_range(enc_values)
            print()
            print(f"Encodeur range = {enc_range:.2f} deg")
            if dominant_range > 1e-9:
                print(f"Gain approx encodeur/{dominant} = {enc_range/dominant_range:.3f}")

    print()
    print("Interpretation rapide :")
    print("  - Si PITCH est dominant et roll/yaw restent faibles : utiliser pitch est coherent.")
    print("  - Si ROLL est dominant : ton mouvement est probablement sur roll, pas pitch.")
    print("  - Si YAW ou plusieurs axes bougent fort : l'extraction Euler directe n'est pas fiable.")
    print("  - Dans ce dernier cas, il faut utiliser l'encodeur pour l'angle, ou une methode quaternion relative.")
    print("=" * 72)


if __name__ == "__main__":
    main()
