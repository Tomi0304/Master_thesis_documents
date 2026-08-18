import csv
import math
import sys
import time

import board
import busio
import pigpio
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

GPIO = 12
FREQ = 50
TS = 1.0 / FREQ

NEUTRAL = 1500
OFF_MIN_S = 70
OFF_MIN_W = 35
OFF_MAX = 390
OFF_KICK = 130
T_KICK = 0.35
SLEW = 1250.0

U_REV = 2.0

KP_S = 1.8
KI_S = 0.6
KD_S = 0.5

ALPHA_W = 2.86
KP_W = KP_S * ALPHA_W
KI_W = KI_S * ALPHA_W
KD_W = KD_S * ALPHA_W

ALPHA_D = 0.6
I_MAX = 0.5

REF_SEQUENCE = [(3.0, 0.0), (15.0, 90.0), (10.0, 0.0)]
REF_RATE = 20.0

MAP_STEPS = [(12.0, 70), (12.0, 100), (12.0, 110), (12.0, 120),
             (12.0, 128), (12.0, 136), (12.0, 144)]
MAP_SETTLE = 4.0

ANGLE_AXIS = 2
ANGLE_SIGN = 1.0
ANGLE_MAX = 140.0
ANGLE_MIN = -12.0
ANGLE_STOP = 78.0
RATE_MAX = 8.0
REV_MAX_RATE = 6.0

ZERO_WARMUP = 4.0
ZERO_SAMPLES = 40
ZERO_TIMEOUT = 25.0
ZERO_STILL = 0.03
ZERO_SPREAD = 1.5


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
            quat, gyro = self.bno.game_quaternion, self.bno.gyro
            if quat is None or gyro is None:
                self.ok = False
                return self.last
            x, y, z, w = quat
            g = [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
            tilt = [math.degrees(math.atan2(g[1], g[2])),
                    math.degrees(math.atan2(g[0], g[2])),
                    math.degrees(math.atan2(g[0], g[1]))]
            a = ANGLE_SIGN * tilt[ANGLE_AXIS] - self.offset
            self.last = ((a + 180.0) % 360.0 - 180.0, tilt, list(gyro))
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
                if max(acc) - min(acc) > ZERO_SPREAD:
                    acc = [a]
                best = max(best, len(acc))
            time.sleep(0.02)
        if len(acc) < ZERO_SAMPLES:
            raise RuntimeError(f"calibration impossible: {best}/{ZERO_SAMPLES} "
                               f"echantillons stables -- bras immobile ?")
        self.offset = sum(acc) / len(acc)
        return self.offset, max(acc) - min(acc)


class Motor:
    def __init__(self, pi):
        self.pi = pi
        self.pwm = NEUTRAL
        self.dir = 1
        self.held = False
        self.revs = 0

    def write(self, us):
        self.pwm = us
        self.pi.hardware_PWM(GPIO, FREQ, int(us * FREQ))
        return us

    def stop(self):
        return self.write(NEUTRAL)

    def kick(self):
        self.write(NEUTRAL - OFF_KICK)
        time.sleep(T_KICK)
        return self.write(NEUTRAL - OFF_MIN_S)

    def command(self, u):
        want = 1 if u >= 0.0 else -1
        if want != self.dir and abs(u) >= U_REV:
            self.dir = want
            self.revs += 1
        self.held = want != self.dir
        mag = 0.0 if self.held else min(1.0, abs(u))
        floor = OFF_MIN_S if self.dir > 0 else OFF_MIN_W
        off = floor + mag * (OFF_MAX - floor)
        target = NEUTRAL - self.dir * off
        step = SLEW * TS
        return self.write(self.pwm + max(-step, min(step, target - self.pwm)))

    def offset(self, off):
        self.dir, self.held = 1, False
        target = NEUTRAL - max(OFF_MIN_S, min(OFF_MAX, off))
        step = SLEW * TS
        return self.write(self.pwm + max(-step, min(step, target - self.pwm)))


def sched(t, table):
    acc = 0.0
    for dur, v in table:
        acc += dur
        if t < acc:
            return v
    return table[-1][1]


def calibrate(imu):
    print("calibration -- ne touche pas au bras")
    off, spread = imu.zero()
    print(f"zero = {off:+.2f} deg (dispersion {spread:.2f})")


def loop(imu, motor, total, body, header, path, amax=ANGLE_MAX):
    rows, status = [], "ok"
    calibrate(imu)
    motor.kick()
    t0 = time.perf_counter()
    k = 0
    while True:
        now = time.perf_counter() - t0
        if now >= total:
            break
        angle, _, gyro = imu.read()
        if angle > amax or angle < ANGLE_MIN:
            status = f"ABORT angle {angle:+.1f} deg"
            break
        if max(abs(v) for v in gyro) > RATE_MAX:
            status = f"ABORT rate {imu.rate():+.2f} rad/s"
            break
        if now > 2.0 and motor.revs / now > REV_MAX_RATE:
            status = f"ABORT {motor.revs} renversements en {now:.1f} s"
            break
        rows.append(body(now, angle, imu.rate()))
        k += 1
        deadline = t0 + k * TS
        rem = deadline - time.perf_counter()
        if rem > 0.001:
            time.sleep(rem - 0.001)
        while time.perf_counter() < deadline:
            pass
    motor.stop()
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print(f"{status} | {len(rows)} samples -> {path}")
    return rows


def control(imu, motor, path):
    st = {"i": 0.0, "d": 0.0, "tgt": None, "ref": 0.0}

    def body(now, angle, rate):
        tgt = sched(now, REF_SEQUENCE)
        if tgt != st["tgt"]:
            st["tgt"], st["i"], st["d"] = tgt, 0.0, 0.0
        step = REF_RATE * TS
        st["ref"] += max(-step, min(step, tgt - st["ref"]))
        ref = st["ref"]
        err = math.radians(ref - angle)

        strong = motor.dir > 0
        kp = KP_S if strong else KP_W
        ki = KI_S if strong else KI_W
        kd = KD_S if strong else KD_W

        st["d"] = ALPHA_D * (-rate) + (1.0 - ALPHA_D) * st["d"]
        u = kp * err + ki * st["i"] + kd * st["d"]
        grow = (u >= 1.0 and err > 0.0) or (u <= -1.0 and err < 0.0)
        if not motor.held and not grow:
            st["i"] = max(-I_MAX, min(I_MAX, st["i"] + err * TS))

        pwm = motor.command(u)
        return (time.time(), now, tgt, ref, angle, math.degrees(err), u, pwm,
                motor.dir, int(motor.held), kp * err, ki * st["i"], kd * st["d"], rate)

    total = sum(d for d, _ in REF_SEQUENCE)
    rows = loop(imu, motor, total, body,
                ("wall", "t", "tgt", "ref", "angle", "err", "u", "pwm",
                 "dir", "held", "P", "I", "D", "gz"), path)
    if not rows:
        return
    late = [abs(r[5]) for r in rows[5 * FREQ:]] or [0.0]
    held = sum(r[9] for r in rows) * TS
    print(f"|err| moyen apres 5 s : {sum(late)/len(late):.2f} deg")
    print(f"{motor.revs} renversement(s) | plancher tenu {held:.2f} s "
          f"({100*held/(len(rows)*TS):.0f} %)")


def sweep(imu, motor, path):
    def body(now, angle, rate):
        off = sched(now, MAP_STEPS)
        return (time.time(), now, off, motor.offset(off), angle, rate)

    total = sum(d for d, _ in MAP_STEPS)
    print(f"balayage {total:.0f} s, {len(MAP_STEPS)} paliers, garde a {ANGLE_STOP:.0f} deg")
    rows = loop(imu, motor, total, body,
                ("wall", "t", "offset", "pwm", "angle", "gz"), path, amax=ANGLE_STOP)
    if not rows:
        return
    print(" offset |   psi_eq |  spread |   sin  | fenetre unix")
    acc = 0.0
    for dur, o in MAP_STEPS:
        seg = [r for r in rows if acc + dur - MAP_SETTLE <= r[1] < acc + dur]
        acc += dur
        if not seg:
            continue
        ang = [r[4] for r in seg]
        m = sum(ang) / len(ang)
        sp = max(ang) - min(ang)
        flag = "" if sp < 1.5 else "  <-- non stabilise"
        print(f" {o:6d} | {m:+8.2f} | {sp:7.2f} | {math.sin(math.radians(m)):+6.3f} "
              f"| {seg[0][0]:.1f} - {seg[-1][0]:.1f}{flag}")


def check(imu):
    calibrate(imu)
    print("bouge le bras a la main. Ctrl-C pour sortir.")
    try:
        while True:
            a, tilt, g = imu.read()
            print(f"\r{a:+8.2f} |{tilt[0]:+9.1f}{tilt[1]:+9.1f}{tilt[2]:+9.1f} "
                  f"|{g[0]:+8.3f}{g[1]:+8.3f}{g[2]:+8.3f}   ", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    if mode not in ("check", "map", "ctrl"):
        return print("usage: check | map | ctrl")
    imu = Imu()
    if mode == "check":
        return check(imu)
    pi = pigpio.pi()
    if not pi.connected:
        return print("pigpiod unreachable")
    motor = Motor(pi)
    try:
        motor.stop()
        time.sleep(3.0)
        print(f"fort  KP={KP_S} KI={KI_S} KD={KD_S}   "
              f"faible KP={KP_W} KI={KI_W} KD={KD_W}   U_REV={U_REV}")
        input("armed. Enter pour lancer, Ctrl-C pour annuler ")
        run = sweep if mode == "map" else control
        run(imu, motor, f"{mode}bi_ap_{time.strftime('%H%M%S')}.csv")
    finally:
        motor.stop()
        time.sleep(0.5)
        pi.stop()


if __name__ == "__main__":
    main()