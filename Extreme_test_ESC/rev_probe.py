import csv
import os
import sys
import time

import pigpio

GPIO = {"main": 12, "tail": 13}
FREQ = 50
LOG_HZ = 200
TS = 1.0 / LOG_HZ

NEUTRAL = 1500
T_ARM = 2.0
T_SPIN = 1.0
T_HOLD = 4.0
T_POST = 1.5

PAIRS = [(1400, 1600), (1300, 1700), (1200, 1800), (1100, 1900)]
SESSION = "rev_session.csv"
FIELDS = ("wall", "v_set", "i_lim", "motor", "lo", "hi", "dwell",
          "t_rev", "cc", "ovp", "esc", "i_max", "note", "file")


def ask(prompt, default=""):
    r = input(f"{prompt} ").strip()
    return r if r else default


def yes(prompt):
    return "o" if input(f"{prompt} [o/N] ").strip().lower().startswith("o") else "n"


def build(lo, hi, dwell):
    plan = [(T_ARM, NEUTRAL), (T_SPIN, lo)]
    if dwell > 0.0:
        plan.append((dwell, NEUTRAL))
    plan += [(T_HOLD, hi), (T_POST, NEUTRAL)]
    return plan


def pwm_at(t, plan):
    acc = 0.0
    for dur, p in plan:
        acc += dur
        if t < acc:
            return p
    return NEUTRAL


def burst(pi, gpio, plan, path):
    total = sum(d for d, _ in plan)
    rows, last, t_rev = [], None, None
    wall0 = time.time()
    t0 = time.perf_counter()
    k = 0
    while True:
        now = time.perf_counter() - t0
        if now >= total:
            break
        p = pwm_at(now, plan)
        if p != last:
            pi.hardware_PWM(gpio, FREQ, int(p * FREQ))
            if last is not None and last != NEUTRAL and p != NEUTRAL:
                t_rev = wall0 + now
            elif last is not None and last != NEUTRAL and p == NEUTRAL and t_rev is None:
                t_rev = wall0 + now
            last = p
        rows.append((now, wall0 + now, p))
        k += 1
        deadline = t0 + k * TS
        rem = deadline - time.perf_counter()
        if rem > 0.0005:
            time.sleep(rem - 0.0005)
        while time.perf_counter() < deadline:
            pass

    pi.hardware_PWM(gpio, FREQ, int(NEUTRAL * FREQ))
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(("t", "wall", "pwm"))
        w.writerows(rows)
    return t_rev, len(rows)


def log_session(rec):
    new = not os.path.exists(SESSION)
    with open(SESSION, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow(rec)


def observe():
    print("  --- regarde la DPPS AVANT de repondre ---")
    cc = yes("  passage en limitation de courant (CC) ?")
    ovp = yes("  protection declenchee (OVP / coupure) ?")
    esc = yes("  ESC redemarre, bip, ou moteur cale ?")
    imax = ask("  courant max lu sur l'afficheur [A, Enter si non vu] :")
    note = ask("  remarque libre [Enter pour rien] :")
    return cc, ovp, esc, imax, note


def one(pi, gpio, motor, lo, hi, dwell, v_set, i_lim):
    print(f"\n>>> {motor}  {lo} -> {hi}   dwell {dwell:g} s   bus {v_set} V")
    if input("    Enter pour lancer, 's' pour sauter, 'q' pour finir : ").strip().lower() == "q":
        return False
    path = f"rev_{motor}_{lo}_{hi}_d{dwell:g}_{time.strftime('%H%M%S')}.csv"
    t_rev, n = burst(pi, gpio, build(lo, hi, dwell), path)
    print(f"    fait | {n} samples | renversement a t_unix = {t_rev:.3f} -> {path}")
    cc, ovp, esc, imax, note = observe()
    log_session(dict(wall=f"{time.time():.0f}", v_set=v_set, i_lim=i_lim, motor=motor,
                     lo=lo, hi=hi, dwell=f"{dwell:g}", t_rev=f"{t_rev:.3f}",
                     cc=cc, ovp=ovp, esc=esc, i_max=imax, note=note, file=path))
    if cc == "o" or ovp == "o" or esc == "o":
        print("    !! anomalie enregistree -- ne monte pas d'un cran de plus")
        return input("    continuer quand meme ? [o/N] ").strip().lower().startswith("o")
    return True


def main():
    motor = sys.argv[1] if len(sys.argv) > 1 else "main"
    if motor not in GPIO:
        return print("usage: rev_probe.py main|tail [pwm_lo pwm_hi [dwell_s]]")

    print("=== sonde de renversement ESC ===")
    print("bras BRIDE ? ecrou d'helice verifie ? personne dans le plan du disque ?")
    if not input("confirmer [o/N] ").strip().lower().startswith("o"):
        return print("annule")

    v_set = ask("tension reglee sur la DPPS [V] :", "8")
    i_lim = ask("limitation de courant reglee [A] :", "20")
    dwell = float(ask("temps mort au neutre [s, 0 = renversement direct] :", "0"))

    pi = pigpio.pi()
    if not pi.connected:
        return print("pigpiod unreachable")
    gpio = GPIO[motor]
    try:
        pi.hardware_PWM(gpio, FREQ, int(NEUTRAL * FREQ))
        time.sleep(T_ARM)
        if len(sys.argv) >= 4:
            pairs = [(int(sys.argv[2]), int(sys.argv[3]))]
        else:
            pairs = PAIRS
        for lo, hi in pairs:
            if not (1100 <= lo <= 1900 and 1100 <= hi <= 1900):
                print(f"pwm hors plage: {lo}/{hi}")
                continue
            if not one(pi, gpio, motor, lo, hi, dwell, v_set, i_lim):
                break
        print(f"\nsession -> {SESSION}")
    finally:
        pi.hardware_PWM(gpio, FREQ, int(NEUTRAL * FREQ))
        time.sleep(0.5)
        pi.stop()


if __name__ == "__main__":
    main()