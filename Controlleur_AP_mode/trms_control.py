import csv
import math
import sys
import time

import board
import busio
import pigpio
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

GPIO_MAIN = 12
GPIO_TAIL = 13
FREQ = 50
TS = 1.0 / FREQ

PWM_NEUTRAL = 1500
PWM_KICK_STRONG = 110
PWM_KICK_WEAK = 145
PWM_MIN_STRONG = 70
PWM_MIN_WEAK = 35
PWM_MAX_STRONG = 400
PWM_MAX_WEAK = 400
SLEW = 1250.0

T_KICK_STRONG = 0.20
T_KICK_WEAK = 0.20
T_DWELL = 0.15
U_MIN = 0.04
U_START = 0.25
T_COAST_FREE = 0.60
U_REV = 0.35
MAX_REV = 4

KP_STRONG = 1.4
KI_STRONG = 0.6
KD_STRONG = 1.5

KP_WEAK = 4.0
KI_WEAK = 0.30
KD_WEAK = 1.2
ALPHA_D = 0.6
I_MAX = 0.5
I_BAND_DEG = 40.0

ANGLE_AXIS = 2
ANGLE_SIGN = 1.0
REF_SEQUENCE = [(2.0, 0.0), (10.0, 90.0)]

ERR_DEAD = 4.0
RATE_DEAD = 0.4
KILL_PWM_ON_EXIT = False
ANGLE_MAX = 140.0
RATE_MAX = 8.0
STALL_TIME = 0.60

ZERO_WARMUP = 4.0
ZERO_SAMPLES = 40
ZERO_TIMEOUT = 25.0
ZERO_STILL = 0.03
ZERO_SPREAD = 1.5


def circ_mean(v):
    s = sum(math.sin(math.radians(x)) for x in v)
    c = sum(math.cos(math.radians(x)) for x in v)
    return math.degrees(math.atan2(s, c))


def circ_spread(v):
    m = circ_mean(v)
    d = [(x - m + 180.0) % 360.0 - 180.0 for x in v]
    return max(d) - min(d)


class Imu:
    def __init__(self):
        self.bno = BNO08X_I2C(busio.I2C(board.SCL, board.SDA), address=0x4A)
        self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.offset = 0.0
        self.ok = False
        self.last = (0.0, [0.0] * 3, [0.0] * 3)
        for _ in range(100):
            self.read()
            if self.ok:
                return
            time.sleep(0.05)
        raise RuntimeError("BNO085 muet -- verifier i2cdetect -y 1 (0x4a)")

    def read(self):
        try:
            quat = self.bno.game_quaternion
            gyro = self.bno.gyro
            if quat is None or gyro is None:
                self.ok = False
                return self.last
            x, y, z, w = quat
            g = [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
            tilt = [
                math.degrees(math.atan2(g[1], g[2])),
                math.degrees(math.atan2(g[0], g[2])),
                math.degrees(math.atan2(g[0], g[1])),
            ]
            a = ANGLE_SIGN * tilt[ANGLE_AXIS] - self.offset
            a = (a + 180.0) % 360.0 - 180.0
            self.last = (a, tilt, list(gyro))
            self.ok = True
        except Exception:
            self.ok = False
        return self.last

    def rate(self):
        return ANGLE_SIGN * self.last[2][ANGLE_AXIS]

    def zero(self):
        self.offset = 0.0
        end = time.perf_counter() + ZERO_WARMUP
        while time.perf_counter() < end:
            self.read()
            time.sleep(0.02)
        acc, best = [], 0
        deadline = time.perf_counter() + ZERO_TIMEOUT
        while time.perf_counter() < deadline and len(acc) < ZERO_SAMPLES:
            a, _, g = self.read()
            if not self.ok or max(abs(v) for v in g) >= ZERO_STILL:
                acc.clear()
            else:
                acc.append(a)
                if circ_spread(acc) > ZERO_SPREAD:
                    acc = [a]
                best = max(best, len(acc))
            time.sleep(0.02)
        if len(acc) < ZERO_SAMPLES:
            raise RuntimeError(
                f"calibration impossible: au mieux {best}/{ZERO_SAMPLES} "
                f"echantillons stables -- bras immobile ? vibrations ?"
            )
        self.offset = circ_mean(acc)
        return self.offset, circ_spread(acc)


class Pid:
    def __init__(self):
        self.p = self.i = self.d = 0.0

    def reset(self):
        self.p = self.i = self.d = 0.0

    def step(self, err, rate, dt, hold):
        strong = err > 0.0
        kp = KP_STRONG if strong else KP_WEAK
        ki = KI_STRONG if strong else KI_WEAK
        kd = KD_STRONG if strong else KD_WEAK
        self.p = kp * err
        if abs(err) > math.radians(I_BAND_DEG):
            self.i = 0.0
        elif not hold:
            self.i = max(-I_MAX, min(I_MAX, self.i + err * dt))
        self.d = ALPHA_D * (-rate) + (1.0 - ALPHA_D) * self.d
        return self.p + ki * self.i + kd * self.d


class Esc:
    def __init__(self, pi, gpio):
        self.pi, self.gpio = pi, gpio
        self.pwm = PWM_NEUTRAL
        self.state = "idle"
        self.dir = 0
        self.last_dir = 0
        self.timer = 0.0
        self.revs = 0

    def _write(self, us):
        self.pwm = us
        self.pi.hardware_PWM(self.gpio, FREQ, int(us * FREQ))
        return us

    def release(self):
        self.state, self.dir, self.timer = "idle", 0, 0.0
        return self._write(PWM_NEUTRAL)

    def new_setpoint(self):
        self.revs = 0

    def _engage(self, d):
        self.dir = self.last_dir = d
        self.state, self.timer = "kick", 0.0

    def update(self, u, dt):
        self.timer += dt
        mag = abs(u)
        want = 1 if u > 0.0 else -1

        if self.state == "dwell":
            if self.timer < T_DWELL:
                return self._write(PWM_NEUTRAL)
            self.state, self.timer = "kick", 0.0

        elif mag < U_MIN:
            if self.state != "idle":
                self.state, self.dir, self.timer = "idle", 0, 0.0
            elif self.timer > T_COAST_FREE:
                self.last_dir, self.revs = 0, 0
            return self._write(PWM_NEUTRAL)

        elif self.last_dir != 0 and want != self.last_dir:
            if mag > U_REV and self.revs < MAX_REV:
                self.revs += 1
                self.dir = self.last_dir = want
                self.state, self.timer = "dwell", 0.0
            elif self.state != "idle":
                self.state, self.dir, self.timer = "idle", 0, 0.0
            elif self.timer > T_COAST_FREE:
                self.last_dir, self.revs = 0, 0
            return self._write(PWM_NEUTRAL)

        elif self.state == "idle":
            if mag < U_START:
                return self._write(PWM_NEUTRAL)
            self._engage(want)

        strong = self.dir > 0
        if self.state == "kick":
            hold = T_KICK_STRONG if strong else T_KICK_WEAK
            if self.timer < hold:
                kick = PWM_KICK_STRONG if strong else PWM_KICK_WEAK
                return self._write(PWM_NEUTRAL - self.dir * kick)
            self.state = "run"

        floor = PWM_MIN_STRONG if strong else PWM_MIN_WEAK
        top = PWM_MAX_STRONG if strong else PWM_MAX_WEAK
        off = floor + min(1.0, mag) * (top - floor)
        target = PWM_NEUTRAL - self.dir * off
        step = SLEW * dt
        return self._write(self.pwm + max(-step, min(step, target - self.pwm)))


def ref_at(t):
    acc = 0.0
    for dur, r in REF_SEQUENCE:
        acc += dur
        if t < acc:
            return r
    return REF_SEQUENCE[-1][1]


def check(imu):
    print("calibration -- ne touche pas au bras")
    off, spread = imu.zero()
    print(f"zero = {off:+.2f} deg (dispersion {spread:.2f})")
    print("bouge le bras a la main. Ctrl-C pour sortir.")
    print("  angle  |  tilt_x   tilt_y   tilt_z  |    gx      gy      gz")
    try:
        while True:
            a, tilt, g = imu.read()
            print(
                f"\r{a:+8.2f} |{tilt[0]:+9.1f}{tilt[1]:+9.1f}{tilt[2]:+9.1f} "
                f"|{g[0]:+8.3f}{g[1]:+8.3f}{g[2]:+8.3f}   ",
                end="", flush=True,
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def control(pi, imu, path):
    pid, esc = Pid(), Esc(pi, GPIO_MAIN)
    total = sum(d for d, _ in REF_SEQUENCE)
    rows, stall, prev_ref, status = [], 0.0, None, "ok"

    print("calibration -- ne touche pas au bras")
    off, spread = imu.zero()
    print(f"zero = {off:+.2f} deg (dispersion {spread:.2f})")

    t0 = time.perf_counter()
    k = 0
    while True:
        now = time.perf_counter() - t0
        if now >= total:
            break

        angle, _, gyro = imu.read()
        rate = imu.rate()
        if abs(angle) > ANGLE_MAX or max(abs(v) for v in gyro) > RATE_MAX:
            status = f"ABORT angle {angle:+.1f} deg, rate {rate:+.2f} rad/s"
            break

        ref = ref_at(now)
        if ref != prev_ref:
            prev_ref = ref
            esc.new_setpoint()
            pid.reset()

        err = ref - angle
        hold = abs(err) < ERR_DEAD and abs(rate) < RATE_DEAD
        u = 0.0 if hold else pid.step(math.radians(err), rate, TS, hold)
        pwm = esc.update(u, TS)

        moving = abs(rate) > 0.05
        stall = 0.0 if moving or esc.state != "run" or abs(u) < 0.4 else stall + TS
        if stall > STALL_TIME:
            status = f"ABORT stall: |u|={abs(u):.2f} sans mouvement"
            break

        rows.append((now, ref, angle, err, u, pwm, esc.state,
                     pid.p, pid.i, pid.d, rate))
        k += 1
        deadline = t0 + k * TS
        rem = deadline - time.perf_counter()
        if rem > 0.001:
            time.sleep(rem - 0.001)
        while time.perf_counter() < deadline:
            pass

    esc.release()
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(("t", "ref", "angle", "err", "u", "pwm", "state",
                    "P", "I", "D", "gz"))
        w.writerows(rows)
    late = [abs(r[3]) for r in rows[int(2 * FREQ):]] or [0.0]
    print(f"{status} | {len(rows)} samples | |err| moyen {sum(late)/len(late):.2f} deg "
          f"| {esc.revs} renversement(s) | -> {path}")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    imu = Imu()
    if mode == "check":
        return check(imu)

    pi = pigpio.pi()
    if not pi.connected:
        return print("pigpiod unreachable")
    main_esc, tail_esc = Esc(pi, GPIO_MAIN), Esc(pi, GPIO_TAIL)
    try:
        main_esc.release()
        tail_esc.release()
        time.sleep(3.0)
        input("armed. Enter pour lancer la boucle, Ctrl-C pour annuler ")
        control(pi, imu, f"ctrl_ap_{time.strftime('%H%M%S')}.csv")
    finally:
        main_esc.release()
        tail_esc.release()
        time.sleep(0.5)
        if KILL_PWM_ON_EXIT:
            pi.hardware_PWM(GPIO_MAIN, 0, 0)
            pi.hardware_PWM(GPIO_TAIL, 0, 0)
        pi.stop()


if __name__ == "__main__":
    main()