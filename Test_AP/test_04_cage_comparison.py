#!/usr/bin/env python3
"""
test_04_openloop_max_angle.py

Open-loop maximum angle test for one mechanical configuration.

This test is intended to quantify the maximum angle reached by the AP
for a strong open-loop PWM command.

Important:
  - This does NOT identify K_prop.
  - This only characterises the achievable angle for a given configuration.
  - Run the script once per configuration, e.g. with V1 cage installed.
  - If testing without cage, be extremely careful: the propeller is exposed.

Example:
  python3 test_04_openloop_max_angle.py --config V1_cage
  python3 test_04_openloop_max_angle.py --config No_cage

Output:
  data/test04_max_angle_<config>_*.csv
"""

import time
import argparse

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

# In your setup, useful thrust is obtained below 1500 us.
PWM_TARGET = 1100

RAMP_DURATION_S = 2.0
HOLD_DURATION_S = 5.0
COOLDOWN_S = 3.0

SAFETY_MAX_ANGLE_DEG = 95.0
SLEW_RATE_US_PER_ITER = 5.0


def clamp_pwm(pwm_us):
    return max(PWM_MIN, min(PWM_MAX, pwm_us))


def run_test(config_name):
    print("\n[TEST 04] Open-loop max angle test")
    print(f"Configuration: {config_name}")
    print(f"PWM target:    {PWM_TARGET} us")
    print()
    print("IMPORTANT:")
    print("- Arm must start vertical down.")
    print("- Encoder will be reset at rest.")
    print("- Keep clear of the propeller.")
    print("- Keep PSU kill switch accessible.")

    hw = None
    logger = None

    try:
        hw = APHardware()

        input("\nPut arm at rest, vertical down, then press ENTER: ")

        hw.reset_encoder()
        time.sleep(0.2)

        print(f"[ENCODER] Reset done. Current angle = {hw.get_angle_deg():+.2f} deg")

        hw.arm_esc()

        logger = CSVLogger(
            f"test04_max_angle_{config_name}",
            ["t_s", "pwm_us", "angle_deg", "phase"],
        )

        input("\nPress ENTER to start the open-loop test: ")

        t_start = time.perf_counter()
        next_tick = t_start + LOOP_DT

        pwm_current = PWM_IDLE

        min_angle = hw.get_angle_deg()
        max_angle = min_angle

        total_duration = RAMP_DURATION_S + HOLD_DURATION_S + COOLDOWN_S

        while True:
            precise_sleep_until(next_tick)
            next_tick += LOOP_DT

            t = time.perf_counter() - t_start

            if t < RAMP_DURATION_S:
                phase = "ramp"
                alpha = t / RAMP_DURATION_S
                pwm_cmd = PWM_IDLE + (PWM_TARGET - PWM_IDLE) * alpha

            elif t < RAMP_DURATION_S + HOLD_DURATION_S:
                phase = "hold"
                pwm_cmd = PWM_TARGET

            elif t < total_duration:
                phase = "cooldown"
                pwm_cmd = PWM_IDLE

            else:
                break

            pwm_cmd = clamp_pwm(pwm_cmd)

            pwm_current = slew_limit(
                pwm_cmd,
                pwm_current,
                SLEW_RATE_US_PER_ITER,
            )
            pwm_current = clamp_pwm(pwm_current)

            hw.set_pwm(pwm_current)

            angle = hw.get_angle_deg()

            min_angle = min(min_angle, angle)
            max_angle = max(max_angle, angle)

            logger.log(pwm_current, angle, phase)

            if abs(angle) > SAFETY_MAX_ANGLE_DEG:
                print(
                    f"\n[SAFETY] |angle| = {abs(angle):.1f} deg "
                    f"> {SAFETY_MAX_ANGLE_DEG:.1f} deg."
                )
                print("[SAFETY] Cutting PWM.")
                break

        hw.set_pwm(PWM_IDLE)
        time.sleep(1.0)

        print("\n[RESULT]")
        print(f"Configuration: {config_name}")
        print(f"Minimum angle: {min_angle:+.2f} deg")
        print(f"Maximum angle: {max_angle:+.2f} deg")
        print(f"Max |angle|:   {max(abs(min_angle), abs(max_angle)):.2f} deg")

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="Configuration name, e.g. V1_cage or No_cage",
    )

    args = parser.parse_args()
    run_test(args.config)


if __name__ == "__main__":
    main()