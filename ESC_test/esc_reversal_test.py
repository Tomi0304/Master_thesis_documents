import csv
import sys
import time

import board
import busio
import pigpio
from adafruit_bno08x import BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

GPIO_MAIN = 12
GPIO_TAIL = 13
ROTOR = "main"
FREQ = 50
TS = 1.0 / FREQ

NEUTRAL = 1500
PWM_MIN = 1100
PWM_MAX = 1900
DEADBAND = 25

CATCH_OFFSET = 100
CATCH_TIME = 0.5
HOLD = 2.0
LAUNCH_TIME = 0.4
HYST_TOP = 170.0
HYST_STEP = 5.0
HYST_DWELL = 0.2

KILL_PWM_ON_EXIT = False
MOVE_THRESH = 0.02
SWEEP_ABORT = 5.0
RATE_ABORT = 6.0


def duty(us):
    return int(max(PWM_MIN, min(PWM_MAX, us)) * FREQ)


i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c, address=0x4A)
bno.enable_feature(BNO_REPORT_GYROSCOPE)


def read_gyro():
    try:
        g = bno.gyro
        return tuple(g) if g else (float("nan"),) * 3
    except Exception:
        return (float("nan"),) * 3


def gmax(g):
    v = [abs(x) for x in g if x == x]
    return max(v) if v else 0.0


class Esc:
    def __init__(self, pi, gpio):
        self.pi = pi
        self.gpio = gpio

    def set(self, us):
        self.pi.hardware_PWM(self.gpio, FREQ, duty(us))

    def arm(self):
        self.set(NEUTRAL)
        time.sleep(3.0)


class Guard:
    def __init__(self):
        self.sweep = [0.0] * 3
        self.sign = [0] * 3

    def check(self, g):
        for j in range(3):
            v = g[j] if g[j] == g[j] else 0.0
            s = 1 if v > 0 else (-1 if v < 0 else self.sign[j])
            if s != self.sign[j]:
                self.sign[j], self.sweep[j] = s, 0.0
            self.sweep[j] += abs(v) * TS
        worst = max(self.sweep)
        if worst > SWEEP_ABORT:
            return f"sweep {worst * 57.3:.0f} deg"
        if gmax(g) > RATE_ABORT:
            return f"rate {gmax(g):.2f} rad/s"
        return None


def profile(p_start, p_end, slew, dwell, hold, catch_offset, catch_time):
    seq = []
    c = [NEUTRAL]

    def catch(target):
        if catch_time <= 0.0:
            return
        s = 1 if target > NEUTRAL else -1
        c[0] = NEUTRAL + s * catch_offset
        seq.extend([(c[0], "catch")] * int(catch_time * FREQ))

    def ramp(target, tag):
        while abs(target - c[0]) > 1e-9:
            c[0] += max(-slew, min(slew, target - c[0]))
            seq.append((c[0], tag))

    def stay(value, seconds, tag):
        seq.extend([(value, tag)] * int(seconds * FREQ))

    catch(p_start)
    ramp(p_start, "ramp_a")
    stay(p_start, hold, "hold_a")
    ramp(NEUTRAL, "down")
    stay(NEUTRAL, dwell, "dwell")
    catch(p_end)
    ramp(p_end, "ramp_b")
    stay(p_end, hold, "hold_b")
    ramp(NEUTRAL, "end")
    stay(NEUTRAL, hold, "rest")
    return seq


def hysteresis_seq(direction, top, step, dwell_each):
    n = int(dwell_each * FREQ)
    seq = [(NEUTRAL, "rest")] * int(0.5 * FREQ)
    seq += [(NEUTRAL + direction * top, "launch")] * int(LAUNCH_TIME * FREQ)
    us = NEUTRAL + direction * top
    while abs(us - NEUTRAL) > DEADBAND:
        us -= direction * step
        seq.extend([(us, "down")] * n)
    seq += [(NEUTRAL, "rest")] * int(0.5 * FREQ)
    return seq


def report_hysteresis(rows, direction):
    up = None
    for r in rows:
        if r[2] == "launch" and gmax(r[3:]) > MOVE_THRESH:
            up = r[1]
            break
    last = None
    for r in rows:
        if r[2] == "down" and gmax(r[3:]) > MOVE_THRESH:
            last = r[1]
    side = "fwd (>1500)" if direction > 0 else "rev (<1500)"
    print(f"  {side}")
    print(f"    lancement OK a {up:.0f} us" if up else "    pas de lancement")
    if last is not None:
        print(f"    mouvement maintenu jusqu'a {last:.0f} us "
              f"({abs(last - NEUTRAL):.0f} us du neutre)")
    else:
        print("    aucun maintien detecte")


def threshold_seq(direction, step, dwell_each):
    seq = [(NEUTRAL, "rest")] * int(1.0 * FREQ)
    us = NEUTRAL + direction * DEADBAND
    limit = PWM_MAX if direction > 0 else PWM_MIN
    while (us - limit) * direction < 0:
        us += direction * step
        seq.extend([(us, "step")] * int(dwell_each * FREQ))
    seq.extend([(NEUTRAL, "rest")] * int(1.0 * FREQ))
    return seq


def run(esc, seq, path):
    rows, guard, status = [], Guard(), "ok"
    t0 = time.perf_counter()
    for k, (u, tag) in enumerate(seq):
        esc.set(u)
        g = read_gyro()
        now = time.perf_counter() - t0
        rows.append((now, u, tag) + g)
        why = guard.check(g)
        if why:
            status = f"ABORT ({why}) at t={now:.2f}s"
            break
        deadline = t0 + (k + 1) * TS
        rem = deadline - time.perf_counter()
        if rem > 0.001:
            time.sleep(rem - 0.001)
        while time.perf_counter() < deadline:
            pass
    esc.set(NEUTRAL)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(("t", "pwm", "phase", "gx", "gy", "gz"))
        w.writerows(rows)
    dt = [b[0] - a[0] for a, b in zip(rows, rows[1:])] or [TS]
    print(f"{status} | dt max {max(dt) * 1e3:.1f} ms | {len(rows)} samples -> {path}")
    return rows


def report_threshold(rows, direction):
    best = None
    for r in rows:
        if r[2] != "step":
            continue
        if gmax(r[3:]) > MOVE_THRESH:
            best = r[1]
            break
    side = "forward (>1500)" if direction > 0 else "reverse (<1500)"
    if best is None:
        print(f"  {side}: aucun demarrage detecte")
    else:
        print(f"  {side}: demarrage a {best:.0f} us "
              f"({abs(best - NEUTRAL):.0f} us du neutre)")


def usage():
    print("usage:")
    print("  esc_reversal_test.py rev P_START P_END SLEW DWELL [CATCH_US CATCH_S]")
    print("  esc_reversal_test.py thr fwd|rev [STEP_US]")
    print("  esc_reversal_test.py hyst fwd|rev [TOP_US] [STEP_US]")
    print("  esc_reversal_test.py off")
    print()
    print("  rev  profil de renversement classique")
    print("  thr  rampe lente par paliers pour trouver le seuil de demarrage")
    print("  hyst rampe DESCENDANTE : seuil de maintien apres lancement")
    print("  off  coupe le PWM (l'ESC va biper, coupe le 12 V ensuite)")


def main():
    if len(sys.argv) < 2:
        return usage()
    mode = sys.argv[1]

    if mode == "off":
        pi = pigpio.pi()
        if pi.connected:
            for g in (GPIO_MAIN, GPIO_TAIL):
                pi.hardware_PWM(g, 0, 0)
            pi.stop()
            print("PWM coupe sur GPIO 12 et 13 -- coupe aussi le 12 V")
        return

    pi = pigpio.pi()
    if not pi.connected:
        return print("pigpiod unreachable")
    active = GPIO_TAIL if ROTOR == "tail" else GPIO_MAIN
    idle = GPIO_MAIN if ROTOR == "tail" else GPIO_TAIL
    esc, other = Esc(pi, active), Esc(pi, idle)

    try:
        other.set(NEUTRAL)
        esc.arm()

        if mode == "thr":
            if len(sys.argv) < 3:
                return usage()
            direction = 1 if sys.argv[2] == "fwd" else -1
            step = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
            tag = f"thr_{ROTOR}_{sys.argv[2]}_s{step:g}.csv"
            input(f"armed [{ROTOR}]. {tag} -- Enter to run ")
            rows = run(esc, threshold_seq(direction, step, 0.4), tag)
            report_threshold(rows, direction)

        elif mode == "hyst":
            if len(sys.argv) < 3:
                return usage()
            direction = 1 if sys.argv[2] == "fwd" else -1
            top = float(sys.argv[3]) if len(sys.argv) > 3 else HYST_TOP
            step = float(sys.argv[4]) if len(sys.argv) > 4 else HYST_STEP
            tag = f"hyst_{ROTOR}_{sys.argv[2]}_t{int(top)}_s{step:g}.csv"
            input(f"armed [{ROTOR}]. {tag} -- Enter to run ")
            rows = run(esc, hysteresis_seq(direction, top, step, HYST_DWELL), tag)
            report_hysteresis(rows, direction)

        elif mode == "rev":
            if len(sys.argv) < 6:
                return usage()
            p_start, p_end = float(sys.argv[2]), float(sys.argv[3])
            slew, dwell = float(sys.argv[4]), float(sys.argv[5])
            c_us = float(sys.argv[6]) if len(sys.argv) > 6 else CATCH_OFFSET
            c_s = float(sys.argv[7]) if len(sys.argv) > 7 else CATCH_TIME
            if c_s > 0 and c_us <= DEADBAND:
                return print(f"catch {c_us:.0f} us dans la bande morte "
                             f"(+-{DEADBAND}) -- augmente-le")
            tag = (f"rev_{ROTOR}_{int(p_start)}_{int(p_end)}"
                   f"_s{slew:g}_d{dwell:g}_c{int(c_us)}-{c_s:g}.csv")
            input(f"armed [{ROTOR}]. {tag} -- Enter to run ")
            run(esc, profile(p_start, p_end, slew, dwell, HOLD, c_us, c_s), tag)

        else:
            usage()

    finally:
        esc.set(NEUTRAL)
        other.set(NEUTRAL)
        time.sleep(0.5)
        if KILL_PWM_ON_EXIT:
            pi.hardware_PWM(active, 0, 0)
            pi.hardware_PWM(idle, 0, 0)
        pi.stop()


if __name__ == "__main__":
    main()