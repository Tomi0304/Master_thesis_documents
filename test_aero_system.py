import pigpio
import time
import sys
import tty
import termios

# ── Pins ────────────────────────────────────────────────────────────────────
PIN_MAIN = 12
PIN_TAIL = 13

# ── ESC params ──────────────────────────────────────────────────────────────
NEUTRAL  = 1500
MIN_PW   = 1100
MAX_PW   = 1900
DEADBAND = 25
STEP     = 25   

def clamp(v):
    return int(max(MIN_PW, min(MAX_PW, v)))

def arm(pi):
    print("[ESC]  Armement (neutre 3s) — NE PAS APPROCHER LES HÉLICES")
    pi.set_servo_pulsewidth(PIN_MAIN, NEUTRAL)
    pi.set_servo_pulsewidth(PIN_TAIL, NEUTRAL)
    for i in range(3, 0, -1):
        print(f"       {i}...")
        time.sleep(1)
    print("[ESC]  Armé\n")

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def print_state(pw_m, pw_t):
    pct_m = (pw_m - NEUTRAL) / (MAX_PW - NEUTRAL) * 100 if pw_m > NEUTRAL else \
            (pw_m - NEUTRAL) / (NEUTRAL - MIN_PW) * 100
    pct_t = (pw_t - NEUTRAL) / (MAX_PW - NEUTRAL) * 100 if pw_t > NEUTRAL else \
            (pw_t - NEUTRAL) / (NEUTRAL - MIN_PW) * 100
    print(f"\r  MAIN  PWM={pw_m:4d}µs ({pct_m:+6.1f}%)  |  "
          f"TAIL  PWM={pw_t:4d}µs ({pct_t:+6.1f}%)    ", end="", flush=True)

HELP = """
╔══════════════════════════════════════════════════╗
║         TRMS — Test boucle ouverte               ║
╠══════════════════════════════════════════════════╣
║  MAIN rotor (GPIO 12)    TAIL rotor (GPIO 13)    ║
║  w : +{step}µs               i : +{step}µs              ║
║  s : -{step}µs               k : -{step}µs              ║
║  a : NEUTRE main         j : NEUTRE tail         ║
║  ESPACE : NEUTRE les deux                        ║
║  z : +{big}µs MAIN          u : +{big}µs TAIL          ║
║  x : -{big}µs MAIN          m : -{big}µs TAIL          ║
║  q : QUITTER (neutre + shutdown)                 ║
╚══════════════════════════════════════════════════╝
""".format(step=STEP, big=STEP*4)

def main():
    pi = pigpio.pi()
    if not pi.connected:
        print("ERREUR : pigpiod non lancé → sudo pigpiod")
        sys.exit(1)

    arm(pi)

    pw_main = NEUTRAL
    pw_tail = NEUTRAL

    print(HELP)
    print_state(pw_main, pw_tail)

    try:
        while True:
            ch = getch()

            if ch == 'q':
                break

            # ── MAIN ──────────────────────────────────────
            elif ch == 'w':
                pw_main = clamp(pw_main + STEP)
            elif ch == 's':
                pw_main = clamp(pw_main - STEP)
            elif ch == 'z':
                pw_main = clamp(pw_main + STEP * 4)
            elif ch == 'x':
                pw_main = clamp(pw_main - STEP * 4)
            elif ch == 'a':
                pw_main = NEUTRAL

            # ── TAIL ──────────────────────────────────────
            elif ch == 'i':
                pw_tail = clamp(pw_tail + STEP)
            elif ch == 'k':
                pw_tail = clamp(pw_tail - STEP)
            elif ch == 'u':
                pw_tail = clamp(pw_tail + STEP * 4)
            elif ch == 'm':
                pw_tail = clamp(pw_tail - STEP * 4)
            elif ch == 'j':
                pw_tail = NEUTRAL

            # ── LES DEUX ──────────────────────────────────
            elif ch == ' ':
                pw_main = NEUTRAL
                pw_tail = NEUTRAL

            else:
                continue

            pi.set_servo_pulsewidth(PIN_MAIN, pw_main)
            pi.set_servo_pulsewidth(PIN_TAIL, pw_tail)
            print_state(pw_main, pw_tail)

    except KeyboardInterrupt:
        pass

    finally:
        print("\n\n[STOP] Neutre + shutdown...")
        pi.set_servo_pulsewidth(PIN_MAIN, NEUTRAL)
        pi.set_servo_pulsewidth(PIN_TAIL, NEUTRAL)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(PIN_MAIN, 0)
        pi.set_servo_pulsewidth(PIN_TAIL, 0)
        pi.stop()
        print("[STOP] OK")

if __name__ == "__main__":
    main()