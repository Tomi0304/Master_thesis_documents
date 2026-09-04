#!/usr/bin/env python3
"""Check a BNO085 wired in UART-RVC mode, one fault at a time.

Run it before touching the rest of the stack:

    python3 check_rvc.py            # /dev/serial0
    python3 check_rvc.py /dev/ttyS0

Each stage isolates one thing that can be wrong, in the order in which the
signal travels, so the first failure names the fault instead of leaving a
silent port. Nothing here writes to the sensor: in RVC mode it only talks.
"""

import sys
import time

BAUD = 115200
FRAME = 19          # header 0xAA 0xAA + index + 16 payload bytes
LISTEN_S = 2.0


def stage(n, title):
    print("\n%d. %s" % (n, title))


def main(port="/dev/serial0"):
    print("BNO085 UART-RVC wiring check on %s" % port)

    # ---- 1. the port exists and opens -----------------------------------
    stage(1, "serial port")
    try:
        import serial
    except ImportError:
        print("   pyserial missing:  sudo pip3 install pyserial --break-system-packages")
        return 1
    try:
        uart = serial.Serial(port, BAUD, timeout=0.5)
    except Exception as exc:
        print("   cannot open: %s" % exc)
        print("   /dev/serial0 missing means the console still owns the UART:")
        print("     sudo raspi-config -> Interface -> Serial Port")
        print("     login shell = No, hardware serial = Yes, then reboot")
        return 1
    print("   open at %d baud" % BAUD)

    # ---- 2. anything at all on the wire ---------------------------------
    # Bytes arriving proves power, ground, TX->RX and P0 high. Silence means
    # one of those four, and no amount of parsing will tell them apart.
    stage(2, "raw bytes")
    uart.reset_input_buffer()
    time.sleep(LISTEN_S)
    n = uart.in_waiting
    raw = uart.read(n) if n else b""
    print("   %d bytes in %.0f s  (~%.0f expected at 100 Hz)"
          % (len(raw), LISTEN_S, LISTEN_S * 100 * FRAME))
    if not raw:
        print("   nothing arriving. In order of likelihood:")
        print("     - P0 not pulled to 3.3 V, so the sensor is still in I2C mode")
        print("     - sensor TX on the wrong pin: try SCL instead of SDA")
        print("     - no power, or Vin on 5 V instead of 3.3 V")
        print("     - RX not on GPIO 15 (physical pin 10)")
        return 1
    print("   first bytes: %s" % " ".join("%02X" % b for b in raw[:12]))

    # ---- 3. the RVC framing ---------------------------------------------
    # Every frame starts 0xAA 0xAA. Their absence with bytes flowing means
    # the sensor talks another protocol or the baud rate is wrong.
    stage(3, "RVC framing")
    heads = sum(1 for i in range(len(raw) - 1)
                if raw[i] == 0xAA and raw[i + 1] == 0xAA)
    print("   %d headers 0xAA 0xAA in %d bytes  (~%d expected)"
          % (heads, len(raw), len(raw) // FRAME))
    if heads == 0:
        print("   bytes but no RVC frames:")
        print("     - P1 high instead of P0: that is UART-SHTP, not RVC")
        print("     - wrong baud rate (RVC is fixed at 115200)")
        return 1
    if heads < len(raw) // FRAME // 2:
        print("   fewer headers than frames: noise or a marginal connection")

    # ---- 4. decoded values ----------------------------------------------
    stage(4, "decoded output")
    try:
        from adafruit_bno08x_rvc import BNO08x_RVC
    except ImportError:
        print("   library missing:")
        print("     sudo pip3 install adafruit-circuitpython-bno08x-rvc"
              " --break-system-packages")
        return 1

    rvc = BNO08x_RVC(uart)
    print("      yaw    pitch    roll  |     ax      ay      az   |  |a|")
    got, t0, norms = 0, time.perf_counter(), []
    while got < 10 and time.perf_counter() - t0 < 5.0:
        try:
            yaw, pitch, roll, ax, ay, az = rvc.heading
        except Exception:
            continue
        got += 1
        norm = (ax * ax + ay * ay + az * az) ** 0.5
        norms.append(norm)
        print("   %+7.2f %+7.2f %+7.2f  | %+6.2f %+6.2f %+6.2f  | %.2f"
              % (yaw, pitch, roll, ax, ay, az, norm))
    if got == 0:
        print("   frames present but none decoded: checksum failures,"
              " suspect a marginal wire")
        return 1

    # ---- 5. is the acceleration usable as a gravity vector? -------------
    # This is what the pivot projection consumes. At rest its magnitude must
    # sit at 1 g; a different constant means the units are not g, and a value
    # that wanders means the sensor is not still.
    stage(5, "gravity vector")
    avg = sum(norms) / len(norms)
    spread = max(norms) - min(norms)
    print("   |a| mean %.3f, spread %.3f" % (avg, spread))
    if abs(avg - 1.0) < 0.1:
        print("   magnitude is 1 g: usable directly as the gravity direction")
    elif abs(avg - 9.81) < 1.0:
        print("   magnitude is 9.81: units are m/s^2, normalise before use")
    else:
        print("   unexpected magnitude; check the sensor is at rest")
    if spread > 0.05 * max(1e-9, avg):
        print("   moving or noisy: redo with the arm untouched")

    print("\nWiring is good. Move the arm by hand and watch ax/ay/az change;")
    print("those three are what the pivot projection replaces the quaternion")
    print("with, so they are the only channel that matters here.")
    uart.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "/dev/serial0"))