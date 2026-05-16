"""
trms_utils.py
Shared utilities for AP characterisation tests.

Adapt the imports and hardware constants to match your existing
trms_controller.py setup.
"""

import time
import csv
import os
from datetime import datetime

import numpy as np

# =============================================================================
# HARDWARE CONSTANTS - adjust to match your setup
# =============================================================================

# GPIO pins (adjust to your wiring)
PIN_PWM_ESC = 12           # hardware PWM channel
PIN_ENCODER_A = 27         # encoder channel A
PIN_ENCODER_B = 17         # encoder channel B

# Encoder
ENCODER_CPR = 500          # counts per revolution (HEDS-5540)
ENCODER_QUAD = 4           # quadrature x4
TICKS_PER_REV = ENCODER_CPR * ENCODER_QUAD  # = 2000
DEG_PER_TICK = 360.0 / TICKS_PER_REV         # = 0.18

# ESC PWM
PWM_IDLE = 1500            # microseconds, neutral/stop
PWM_MIN = 1100             # full reverse
PWM_MAX = 1900             # full forward
PWM_DEADBAND = 25          # +/- around IDLE

# Control loop
LOOP_HZ = 200
LOOP_DT = 1.0 / LOOP_HZ    # = 5 ms

# Physical constants
G = 9.81                   # m/s^2


# =============================================================================
# DATA LOGGING
# =============================================================================

class CSVLogger:
    """Simple CSV logger with timestamped filename."""

    def __init__(self, test_name, columns, output_dir="data"):
        os.makedirs(output_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(output_dir, f"{test_name}_{stamp}.csv")
        self.file = open(self.filename, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(columns)
        self.t0 = time.perf_counter()

    def log(self, *values):
        t = time.perf_counter() - self.t0
        self.writer.writerow([f"{t:.6f}"] + [f"{v:.6f}" if isinstance(v, float)
                                              else str(v) for v in values])

    def close(self):
        self.file.close()
        print(f"[CSV] Saved to {self.filename}")


# =============================================================================
# HARDWARE INTERFACE - adapt to your existing pigpio setup
# =============================================================================

class APHardware:
    """Wrapper around your existing pigpio setup.
    REPLACE THE IMPLEMENTATION BELOW with calls into your existing
    trms_controller.py infrastructure if you already have one.
    """

    def __init__(self):
        import pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod not running. Start with: sudo pigpiod")

        # Setup encoder
        self.pi.set_mode(PIN_ENCODER_A, pigpio.INPUT)
        self.pi.set_mode(PIN_ENCODER_B, pigpio.INPUT)
        self.pi.set_pull_up_down(PIN_ENCODER_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(PIN_ENCODER_B, pigpio.PUD_UP)

        # Encoder state
        self._encoder_count = 0
        self._cb_a = self.pi.callback(PIN_ENCODER_A, pigpio.EITHER_EDGE,
                                       self._on_encoder)
        self._cb_b = self.pi.callback(PIN_ENCODER_B, pigpio.EITHER_EDGE,
                                       self._on_encoder)
        self._last_state = (self.pi.read(PIN_ENCODER_A),
                            self.pi.read(PIN_ENCODER_B))

        # Setup PWM
        self.pi.set_servo_pulsewidth(PIN_PWM_ESC, 0)  # off

    def _on_encoder(self, gpio, level, tick):
        """Quadrature decoder x4."""
        a = self.pi.read(PIN_ENCODER_A)
        b = self.pi.read(PIN_ENCODER_B)
        # State transition table for x4 decoding
        last_a, last_b = self._last_state
        if (last_a, last_b) == (0, 0):
            if (a, b) == (1, 0):
                self._encoder_count += 1
            elif (a, b) == (0, 1):
                self._encoder_count -= 1
        elif (last_a, last_b) == (1, 0):
            if (a, b) == (1, 1):
                self._encoder_count += 1
            elif (a, b) == (0, 0):
                self._encoder_count -= 1
        elif (last_a, last_b) == (1, 1):
            if (a, b) == (0, 1):
                self._encoder_count += 1
            elif (a, b) == (1, 0):
                self._encoder_count -= 1
        elif (last_a, last_b) == (0, 1):
            if (a, b) == (0, 0):
                self._encoder_count += 1
            elif (a, b) == (1, 1):
                self._encoder_count -= 1
        self._last_state = (a, b)

    def reset_encoder(self):
        self._encoder_count = 0

    def get_angle_deg(self):
        """Return arm angle in degrees from rest position."""
        return self._encoder_count * DEG_PER_TICK

    def set_pwm(self, pulse_us):
        """Send PWM in microseconds. Use PWM_IDLE to stop."""
        self.pi.set_servo_pulsewidth(PIN_PWM_ESC, pulse_us)

    def arm_esc(self):
        """ESC arming procedure."""
        print("[ESC] Arming...")
        self.set_pwm(PWM_IDLE)
        time.sleep(2.0)
        print("[ESC] Armed")

    def stop(self):
        self.set_pwm(0)
        time.sleep(0.1)

    def close(self):
        self.stop()
        self._cb_a.cancel()
        self._cb_b.cancel()
        self.pi.stop()


# =============================================================================
# REAL-TIME LOOP HELPER
# =============================================================================

def precise_sleep_until(target_time):
    """Busy-wait + sleep hybrid for accurate timing."""
    while True:
        remaining = target_time - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > 0.001:
            time.sleep(remaining - 0.0005)
        # spin for the last <1 ms

def slew_limit(target, current, max_step):
    """Limit rate of change. max_step in same unit as target."""
    diff = target - current
    if abs(diff) <= max_step:
        return target
    return current + max_step if diff > 0 else current - max_step