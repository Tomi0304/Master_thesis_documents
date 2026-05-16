#!/usr/bin/env python3
"""
test_05_closed_loop_step.py

Closed-loop step response for the aero-pendulum.

Controller:
    PWM = PWM_IDLE + feedforward_gravity + PID_feedback

The controller output is directly expressed in microseconds.

Sign convention:
    - theta = 0 deg at rest position, vertical down.
    - Useful motor thrust in this setup is obtained for PWM < 1500 us.
    - Therefore negative setpoints are used: -5, -10, -15 deg.

Output CSV columns:
    t_s, setpoint_deg, angle_deg, pwm_us, error_deg,
    integral, angle_rate_dps, u_p, u_i, u_d, u_ff, u_total
"""

import time
import numpy as np

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
# CONTROLLER PARAMETERS
# =============================================================================

# Feedback gains directly in PWM units
KP_US_PER_DEG = 8.0          # proportional gain [us/deg]
KI_US_PER_DEG_S = 1.5        # integral gain [us/(deg.s)]
KD_US_PER_DPS = 1.2          # derivative gain [us/(deg/s)]

# Gravity feedforward:
# u_ff = KG_US * sin(theta_ref)
# For negative theta_ref, this gives negative PWM offset, i.e. PWM < 1500 us.
KG_US = 1500.0               # [us]

# Integral protection
INTEGRAL_LIMIT_DEG_S = 80.0

# Derivative low-pass filter
RATE_FILTER_ALPHA = 0.20     # 0=no update, 1=no filtering


# =============================================================================
# TEST PARAMETERS
# =============================================================================

# Start small. Do NOT start with -20 or more before validating.
SETPOINTS_DEG = [-5.0, -10.0, -15.0]
RUNS_PER_SETPOINT = 2

DURATION_PER_RUN_S = 10.0
STEP_TIME_S = 1.0

SAFETY_MAX_ANGLE_DEG = 60.0
SLEW_RATE_US_PER_ITER = 5.0

COOLDOWN_BETWEEN_RUNS_S = 4.0
COOLDOWN_BETWEEN_SETPOINTS_S = 6.0


def clamp_pwm(pwm_us):
    """Clamp PWM command to ESC limits."""
    return max(PWM_MIN, min(PWM_MAX, pwm_us))


def run_one(hw, setpoint_deg, run_idx):
    """Run one closed-loop step response."""

    name = f"sp{int(abs(setpoint_deg)):03d}_{'neg' if setpoint_deg < 0 else 'pos'}_run{run_idx:02d}"

    logger = CSVLogger(
        f"test05_closed_loop_{name}",
        [
            "t_s",
            "setpoint_deg",
            "angle_deg",
            "pwm_us",
            "error_deg",
            "integral",
            "angle_rate_dps",
            "u_p",
            "u_i",
            "u_d",
            "u_ff",
            "u_total",
        ],
    )

    print(f"\n  [Run {run_idx}] Setpoint = {setpoint_deg:+.1f} deg")

    integral = 0.0
    pwm_current = PWM_IDLE

    prev_angle = hw.get_angle_deg()
    angle_rate_filt = 0.0

    t_start = time.perf_counter()
    next_tick = t_start + LOOP_DT

    min_angle = prev_angle
    max_angle = prev_angle
    max_abs_error = 0.0

    try:
        while True:
            precise_sleep_until(next_tick)
            next_tick += LOOP_DT

            t = time.perf_counter() - t_start

            if t >= DURATION_PER_RUN_S:
                break

            angle = hw.get_angle_deg()

            # Step reference
            if t < STEP_TIME_S:
                setpoint_active = 0.0
            else:
                setpoint_active = setpoint_deg

            # Angle rate from encoder
            raw_rate = (angle - prev_angle) / LOOP_DT
            prev_angle = angle

            angle_rate_filt = (
                RATE_FILTER_ALPHA * raw_rate
                + (1.0 - RATE_FILTER_ALPHA) * angle_rate_filt
            )

            # Error
            error = setpoint_active - angle

            # Integral term
            integral += error * LOOP_DT
            integral = float(np.clip(
                integral,
                -INTEGRAL_LIMIT_DEG_S,
                INTEGRAL_LIMIT_DEG_S,
            ))

            # PID terms in microseconds
            u_p = KP_US_PER_DEG * error
            u_i = KI_US_PER_DEG_S * integral

            # Derivative on measurement:
            # u_d = -Kd * theta_dot
            # If the arm moves too fast downward, this reduces the thrust.
            u_d = -KD_US_PER_DPS * angle_rate_filt

            # Gravity feedforward on reference angle
            u_ff = KG_US * np.sin(np.deg2rad(setpoint_active))

            u_total = u_p + u_i + u_d + u_ff

            pwm_target = PWM_IDLE + u_total
            pwm_target = clamp_pwm(pwm_target)

            # Slew-rate limit
            pwm_current = slew_limit(
                pwm_target,
                pwm_current,
                SLEW_RATE_US_PER_ITER,
            )
            pwm_current = clamp_pwm(pwm_current)

            hw.set_pwm(pwm_current)

            logger.log(
                setpoint_active,
                angle,
                pwm_current,
                error,
                integral,
                angle_rate_filt,
                u_p,
                u_i,
                u_d,
                u_ff,
                u_total,
            )

            min_angle = min(min_angle, angle)
            max_angle = max(max_angle, angle)
            max_abs_error = max(max_abs_error, abs(error))

            # Safety
            if abs(angle) > SAFETY_MAX_ANGLE_DEG:
                print(
                    f"  [SAFETY] |angle| = {abs(angle):.1f} deg "
                    f"> {SAFETY_MAX_ANGLE_DEG:.1f} deg"
                )
                break

    finally:
        hw.set_pwm(PWM_IDLE)
        logger.close()

    print(
        f"  [Run {run_idx}] Done. "
        f"Angle range: {min_angle:+.2f} to {max_angle:+.2f} deg, "
        f"max |error| = {max_abs_error:.2f} deg"
    )


def main():
    print("[TEST 05] Closed-loop AP step response")
    print(f"Setpoints: {SETPOINTS_DEG} deg")
    print(f"Runs per setpoint: {RUNS_PER_SETPOINT}")
    print()
    print("Controller gains:")
    print(f"  KP = {KP_US_PER_DEG:.2f} us/deg")
    print(f"  KI = {KI_US_PER_DEG_S:.2f} us/(deg.s)")
    print(f"  KD = {KD_US_PER_DPS:.2f} us/(deg/s)")
    print(f"  KG = {KG_US:.2f} us")
    print()
    print("IMPORTANT:")
    print("- Cage installed.")
    print("- Arm starts vertical down.")
    print("- Encoder will be reset at rest.")
    print("- Useful thrust is PWM < 1500 us, so setpoints are negative.")
    print("- Keep PSU kill switch accessible.")

    hw = None

    try:
        hw = APHardware()

        print("\nPut the arm in rest position, vertical down.")
        input("Press ENTER to reset encoder and arm ESC: ")

        hw.reset_encoder()
        time.sleep(0.2)

        print(f"[ENCODER] Reset done. Current angle = {hw.get_angle_deg():+.2f} deg")

        hw.arm_esc()

        input("\nPress ENTER to start closed-loop tests: ")

        for sp in SETPOINTS_DEG:
            print(f"\n[SETPOINT {sp:+.1f} deg]")

            for r in range(1, RUNS_PER_SETPOINT + 1):
                run_one(hw, sp, r)

                if r < RUNS_PER_SETPOINT:
                    print(f"  Cooldown {COOLDOWN_BETWEEN_RUNS_S:.1f}s...")
                    hw.set_pwm(PWM_IDLE)
                    time.sleep(COOLDOWN_BETWEEN_RUNS_S)

            if sp != SETPOINTS_DEG[-1]:
                print(f"  Cooldown {COOLDOWN_BETWEEN_SETPOINTS_S:.1f}s before next setpoint...")
                hw.set_pwm(PWM_IDLE)
                time.sleep(COOLDOWN_BETWEEN_SETPOINTS_S)

    except KeyboardInterrupt:
        print("\n[INTERRUPT] User stopped.")

    finally:
        if hw is not None:
            hw.set_pwm(PWM_IDLE)
            time.sleep(1.0)
            hw.stop()
            hw.close()

    print("\n[TEST 05] Done.")
    print("Post-process with:")
    print("  python3 analyse_closed_loop.py")


if __name__ == "__main__":
    main()