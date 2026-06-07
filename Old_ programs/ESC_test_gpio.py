import time
import math
import pigpio

# =========================
# Configuration ESC
# =========================
ESC_GPIO = 13

STOP_US = 1500
MIN_US = 1100
MAX_US = 1900

# =========================
# Paramètres sinus
# =========================
FREQ_HZ = 0.05               # 20 s par période
AMP_TARGET_US = 300         # amplitude max autour du neutre
RAMP_TIME_S = 8.0            # montée progressive amplitude
TEST_DURATION_S = 40.0
DT_S = 0.02                  # 50 Hz

# =========================
# Lissage / sécurité
# =========================
DEADBAND_US = 25             # datasheet ESC
MIN_EFFECTIVE_OFFSET_US = 40 # un peu au-dessus de la deadband
SLEW_RATE_US_PER_S = 250.0   # vitesse max de variation de PWM
NEUTRAL_HOLD_S = 0.20        # pause au neutre lors d'un changement de sens


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def connect_pi() -> pigpio.pi:
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("Impossible de se connecter à pigpiod")
    return pi


def set_pulse(pi: pigpio.pi, pulse_us: float) -> None:
    pulse_us = clamp(pulse_us, MIN_US, MAX_US)
    pi.set_servo_pulsewidth(ESC_GPIO, int(round(pulse_us)))


def arm_esc(pi: pigpio.pi, arming_time_s: float = 5.0) -> None:
    print(f"[INFO] Armement ESC à {STOP_US} µs...")
    set_pulse(pi, STOP_US)
    time.sleep(arming_time_s)
    print("[INFO] ESC armé.")


def stop_esc(pi: pigpio.pi, stop_time_s: float = 2.0, cut_signal: bool = True) -> None:
    print("[INFO] Retour neutre...")
    set_pulse(pi, STOP_US)
    time.sleep(stop_time_s)
    if cut_signal:
        print("[INFO] Coupure du signal...")
        pi.set_servo_pulsewidth(ESC_GPIO, 0)


def half_cosine_ramp(t: float, ramp_time: float) -> float:
    """Rampe douce 0 -> 1."""
    if t <= 0:
        return 0.0
    if t >= ramp_time:
        return 1.0
    return 0.5 * (1.0 - math.cos(math.pi * t / ramp_time))


def apply_deadband_compensation(offset: float) -> float:
    """
    Compensation douce autour du neutre.
    - si très proche de zéro => 0
    - sinon on saute légèrement au-dessus de la deadband
    """
    if abs(offset) < 1e-9:
        return 0.0

    sign = 1.0 if offset > 0 else -1.0
    mag = abs(offset)

    # Si on est trop près du neutre, on considère neutre pur
    if mag < DEADBAND_US:
        return 0.0

    # Compression douce pour éviter un gros saut
    # On garde la structure sinus mais on dépasse la zone morte
    effective_mag = MIN_EFFECTIVE_OFFSET_US + 0.96 * (mag - DEADBAND_US)
    return sign * effective_mag


def slew_limit(target: float, current: float, dt: float, rate_us_per_s: float) -> float:
    max_step = rate_us_per_s * dt
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


def main() -> None:
    pi = connect_pi()

    try:
        print("=" * 60)
        print(" Test ESC bidirectionnel - sinusoïde améliorée")
        print("=" * 60)
        print("- sans hélice")
        print("- moteur sain")
        print("- alim stable")
        print("- masse commune Pi / ESC")
        print()

        input("[ACTION] Coupe l'alim ESC, puis appuie sur ENTREE...")

        set_pulse(pi, STOP_US)
        print(f"[INFO] Neutre appliqué à {STOP_US} µs.")

        input("[ACTION] Rallume l'alim ESC, attends les bips + les 2 tons, puis appuie sur ENTREE...")

        arm_esc(pi, arming_time_s=5.0)

        input("[ACTION] Appuie sur ENTREE pour lancer la sinusoïde...")

        current_pulse = STOP_US
        previous_sign = 0

        t0 = time.perf_counter()
        last_loop = t0

        while True:
            now = time.perf_counter()
            t = now - t0
            dt = now - last_loop
            last_loop = now

            if t >= TEST_DURATION_S:
                break

            # Rampe douce d'amplitude
            ramp = half_cosine_ramp(t, RAMP_TIME_S)
            amplitude = AMP_TARGET_US * ramp

            # Sinusoïde brute
            sinus = math.sin(2.0 * math.pi * FREQ_HZ * t)
            raw_offset = amplitude * sinus

            # Compensation deadband
            compensated_offset = apply_deadband_compensation(raw_offset)
            target_pulse = STOP_US + compensated_offset

            # Détection changement de sens
            current_sign = 0
            if compensated_offset > 1e-6:
                current_sign = 1
            elif compensated_offset < -1e-6:
                current_sign = -1

            # Si inversion de sens: petit passage neutre
            if previous_sign != 0 and current_sign != 0 and current_sign != previous_sign:
                set_pulse(pi, STOP_US)
                current_pulse = STOP_US
                time.sleep(NEUTRAL_HOLD_S)

            previous_sign = current_sign

            # Limiteur de pente
            current_pulse = slew_limit(target_pulse, current_pulse, dt, SLEW_RATE_US_PER_S)
            current_pulse = clamp(current_pulse, MIN_US, MAX_US)

            set_pulse(pi, current_pulse)

            print(
                f"t={t:6.2f}s | amp={amplitude:6.1f}us | "
                f"sin={sinus: .3f} | target={target_pulse:7.1f}us | "
                f"cmd={current_pulse:7.1f}us",
                end="\r"
            )

            time.sleep(DT_S)

        print()
        print("[INFO] Fin du test.")

    except KeyboardInterrupt:
        print("\n[WARN] Arrêt clavier détecté.")

    finally:
        stop_esc(pi, stop_time_s=2.0, cut_signal=True)
        pi.stop()
        print("[INFO] Programme terminé.")


if __name__ == "__main__":
    main()