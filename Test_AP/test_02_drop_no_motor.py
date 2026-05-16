#!/usr/bin/env python3
"""
test_02_drop_no_motor.py

Free-decay passive drop test for moment of inertia and damping identification.

Goal:
  Identify or validate the passive mechanical dynamics of the aero-pendulum:
      J * theta_ddot + b * theta_dot + m*g*Lcm*sin(theta) = 0

Procedure for each run:
  1. Motor/ESC power OFF.
  2. Put the arm in the rest position, vertical down.
  3. Press ENTER to zero the encoder at rest.
  4. Move the arm manually to a safe initial angle, typically 15-30 deg.
  5. Press ENTER to start logging.
  6. Release the arm cleanly, without pushing.
  7. The script logs for a fixed duration.

Output:
  data/test02_drop_runXX_YYYYMMDD_HHMMSS.csv

Columns:
  t_s, angle_deg

Post-process with:
  python3 fit_drop_test.py
"""

import time
from trms_utils import APHardware, CSVLogger, LOOP_DT, precise_sleep_until


# =============================================================================
# TEST PARAMETERS
# =============================================================================

INITIAL_ANGLE_TARGET_DEG = 20.0   # recommended safe starting angle
MAX_DURATION_S = 8.0              # fixed logging duration
NUM_RUNS = 5

# Safety only. This should normally never be reached during a passive test.
SAFETY_MAX_ANGLE_DEG = 95.0


def run_one(run_idx):
    print(f"\n{'=' * 60}")
    print(f"[RUN {run_idx}] Passive drop test")
    print(f"{'=' * 60}")

    hw = APHardware()
    logger = None

    try:
        print("\n1) Make sure motor/ESC power is OFF.")
        print("2) Put the arm in the rest position, vertical down.")
        input(">>> Press ENTER when the arm is at rest: ")

        # Define theta = 0 deg at rest position
        hw.reset_encoder()
        time.sleep(0.2)

        print("\n[ENCODER] Reset done.")
        print(f"Current angle: {hw.get_angle_deg():+.2f} deg")

        print(f"\n3) Move the arm manually to about "
              f"{INITIAL_ANGLE_TARGET_DEG:.0f} deg.")
        print("   Use the largest safe angle that does NOT hit the table.")
        print("   15-30 deg is totally acceptable.")
        input(">>> Press ENTER when ready to start logging: ")

        angle_initial = hw.get_angle_deg()

        logger = CSVLogger(
            f"test02_drop_run{run_idx:02d}",
            ["t_s", "angle_deg"]
        )

        print("\n[RUN] Logging started.")
        print("Release the arm cleanly now, without pushing.")
        print(f"Logging duration: {MAX_DURATION_S:.1f} s")

        t_start = time.perf_counter()
        next_tick = t_start + LOOP_DT

        min_angle = angle_initial
        max_angle = angle_initial

        while True:
            precise_sleep_until(next_tick)
            next_tick += LOOP_DT

            elapsed = time.perf_counter() - t_start
            angle = hw.get_angle_deg()

            logger.log(angle)

            min_angle = min(min_angle, angle)
            max_angle = max(max_angle, angle)

            # Passive safety check
            if abs(angle) > SAFETY_MAX_ANGLE_DEG:
                print(f"\n[SAFETY] |angle| = {abs(angle):.1f} deg "
                      f"> {SAFETY_MAX_ANGLE_DEG:.1f} deg.")
                print("[SAFETY] Stopping this run.")
                break

            # Fixed-duration logging
            if elapsed >= MAX_DURATION_S:
                print(f"\n[RUN] Finished after {elapsed:.2f} s.")
                break

        print(f"[RUN {run_idx}] Initial angle: {angle_initial:+.2f} deg")
        print(f"[RUN {run_idx}] Min angle:     {min_angle:+.2f} deg")
        print(f"[RUN {run_idx}] Max angle:     {max_angle:+.2f} deg")

    finally:
        if logger is not None:
            logger.close()

        hw.close()


def main():
    print("[TEST 02] Passive drop test without motor")
    print(f"Number of runs: {NUM_RUNS}")
    print(f"Loop frequency: {1.0 / LOOP_DT:.0f} Hz")
    print(f"Logging duration per run: {MAX_DURATION_S:.1f} s")

    print("\nIMPORTANT:")
    print("- Motor/ESC power must be OFF.")
    print("- Do not use 90 deg if the arm can hit the table.")
    print("- Use 15-30 deg if that allows clean oscillation.")
    print("- The script no longer stops at 5 deg; it logs the full decay.")

    input("\nPress ENTER to start the test sequence: ")

    try:
        for i in range(1, NUM_RUNS + 1):
            run_one(i)

            if i < NUM_RUNS:
                print("\nPrepare the next run.")
                print("Put the arm back to rest before continuing.")
                input(">>> Press ENTER for next run, or Ctrl+C to stop: ")

    except KeyboardInterrupt:
        print("\n[INTERRUPT] User stopped the test.")

    print("\n[TEST 02] Done.")
    print("Post-process with:")
    print("  python3 fit_drop_test.py")


if __name__ == "__main__":
    main()