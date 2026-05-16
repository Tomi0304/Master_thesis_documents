#!/usr/bin/env python3
"""
test_03_step_response.py

Motorised step response for thrust gain K_T identification
and equilibrium angle characterisation.

Procedure:
  1. ESC powered and armed.
  2. Arm in rest position, vertical down.
  3. Encoder is reset at rest, so theta = 0 deg at vertical down.
  4. The script applies a sequence of PWM levels.
  5. For each level: ramp-up, hold, ramp-down to idle, cooldown.

Output:
  data/test03_step_response_*.csv

Columns:
  t_s, pwm_current_us, angle_deg, pwm_target_us

Post-process with:
  python3 fit_step_response.py
"""

import time

from trms_utils import (
    APHardware,
    CSVLogger,
    LOOP_DT,
    PWM_IDLE,
    PWM_MIN,
    PWM_MAX,
    precise_sleep_until,
    slew_limit,
)


# =============================================================================
# TEST PARAMETERS
# =============================================================================

# PWM levels to test.
# 1500 us = idle/stop.
# For your setup, useful thrust seems to be below 1500 us.
# Start conservative, then increase thrust progressively.
PWM_LEVELS = [1350, 1300, 1250, 1200, 1150, 1100]

RAMP_DURATION_S = 1.0
HOLD_DURATION_S = 4.0
COOLDOWN_S = 5.0

# Emergency stop if absolute angle exceeds this value.
# Use abs(angle), because your useful motion can be negative.
SAFETY_MAX_ANGLE_DEG = 95.0

# Max PWM change per control-loop iteration.
# At 200 Hz, 5 us/iteration = 1000 us/s max slew rate.
SLEW_RATE_US_PER_ITER = 5.0


def clamp_pwm(pwm_us):
    """Clamp PWM command to ESC allowed range."""
    return max(PWM_MIN, min(PWM_MAX, pwm_us))


def run_step(hw, logger, pwm_target):
    """Run a single ramp-hold-ramp test from idle to pwm_target and back."""

    print(f"\n[STEP] PWM target = {pwm_target} us")
    print(
        f"       Ramp-up {RAMP_DURATION_S:.1f}s, "
        f"hold {HOLD_DURATION_S:.1f}s, "
        f"ramp-down {RAMP_DURATION_S:.1f}s, "
        f"cooldown {COOLDOWN_S:.1f}s"
    )

    t_start = time.perf_counter()
    next_tick = t_start + LOOP_DT
    pwm_current = PWM_IDLE

    total_duration = 2 * RAMP_DURATION_S + HOLD_DURATION_S + COOLDOWN_S

    min_angle = hw.get_angle_deg()
    max_angle = min_angle

    while True:
        precise_sleep_until(next_tick)
        next_tick += LOOP_DT

        t = time.perf_counter() - t_start

        # ------------------------------------------------------------
        # Desired PWM profile
        # ------------------------------------------------------------
        if t < RAMP_DURATION_S:
            # Ramp from idle to target
            alpha = t / RAMP_DURATION_S
            pwm_cmd = PWM_IDLE + (pwm_target - PWM_IDLE) * alpha

        elif t < RAMP_DURATION_S + HOLD_DURATION_S:
            # Hold target
            pwm_cmd = pwm_target

        elif t < 2 * RAMP_DURATION_S + HOLD_DURATION_S:
            # Ramp back to idle
            t_ramp = t - (RAMP_DURATION_S + HOLD_DURATION_S)
            alpha = t_ramp / RAMP_DURATION_S
            pwm_cmd = pwm_target + (PWM_IDLE - pwm_target) * alpha

        else:
            # Cooldown at idle
            pwm_cmd = PWM_IDLE

        pwm_cmd = clamp_pwm(pwm_cmd)

        # Apply software slew-rate limit
        pwm_current = slew_limit(
            pwm_cmd,
            pwm_current,
            SLEW_RATE_US_PER_ITER,
        )
        pwm_current = clamp_pwm(pwm_current)

        hw.set_pwm(pwm_current)

        # ------------------------------------------------------------
        # Measurement and logging
        # ------------------------------------------------------------
        angle = hw.get_angle_deg()

        min_angle = min(min_angle, angle)
        max_angle = max(max_angle, angle)

        logger.log(pwm_current, angle, pwm_target)

        # ------------------------------------------------------------
        # Safety check
        # ------------------------------------------------------------
        if abs(angle) > SAFETY_MAX_ANGLE_DEG:
            print(
                f"[SAFETY] |angle| = {abs(angle):.1f} deg "
                f"> {SAFETY_MAX_ANGLE_DEG:.1f} deg, ABORTING."
            )
            hw.set_pwm(PWM_IDLE)
            time.sleep(0.5)
            return False

        if t >= total_duration:
            break

    hw.set_pwm(PWM_IDLE)

    print(
        f"[STEP] Done. Angle range: "
        f"{min_angle:+.2f} deg to {max_angle:+.2f} deg"
    )

    return True


def main():
    print("[TEST 03] Motorised step response")
    print(f"PWM levels: {PWM_LEVELS}")
    print(f"Loop frequency: {1.0 / LOOP_DT:.0f} Hz")
    print()
    print("IMPORTANT:")
    print("- Cage installed.")
    print("- Arm initially vertical down.")
    print("- Keep clear of the propeller.")
    print("- Keep the bench power supply kill switch accessible.")
    print("- The encoder will be reset at the vertical-down rest position.")

    hw = None
    logger = None

    try:
        hw = APHardware()

        print("\nPut the arm in rest position, vertical down.")
        input("Press ENTER when ready to reset encoder and arm ESC: ")

        hw.reset_encoder()
        time.sleep(0.2)

        print(f"[ENCODER] Reset done. Current angle = {hw.get_angle_deg():+.2f} deg")

        hw.arm_esc()

        logger = CSVLogger(
            "test03_step_response",
            ["t_s", "pwm_current_us", "angle_deg", "pwm_target_us"],
        )

        for pwm in PWM_LEVELS:
            ok = run_step(hw, logger, pwm)
            if not ok:
                break

    except KeyboardInterrupt:
        print("\n[INTERRUPT] User stopped.")

    finally:
        if hw is not None:
            hw.set_pwm(PWM_IDLE)
            time.sleep(1.0)
            hw.stop()

        if logger is not None:
            logger.close()

        if hw is not None:
            hw.close()

    print("\n[TEST 03] Done.")
    print("Post-process with:")
    print("  python3 fit_step_response.py")


if __name__ == "__main__":
    main()