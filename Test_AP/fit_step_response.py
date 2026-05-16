#!/usr/bin/env python3
"""
fit_step_response.py

Identify the static thrust/couple gain K_T from motorised step-response data.

Static equilibrium model:
    K_T * (PWM - PWM_IDLE) = M_EQ * g * L_CM * sin(theta_eq)

Therefore:
    K_T = M_EQ * g * L_CM * sin(theta_eq) / (PWM - PWM_IDLE)

Important:
    In this AP setup, useful thrust can correspond to PWM < 1500 us.
    Therefore delta_pwm and theta_eq can both be negative, giving a positive K_T.

Usage:
    python3 fit_step_response.py
    python3 fit_step_response.py data/test03_step_response_*.csv

Input CSV columns:
    t_s, pwm_current_us, angle_deg, pwm_target_us

Output:
    data/test03_step_fit.png
"""

import sys
import glob
import os

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# =============================================================================
# MECHANICAL PARAMETERS
# =============================================================================

G = 9.81

# From SolidWorks + drop-test validation
M_EQ = 0.58270        # kg
L_CM = 0.32714        # m
J = 0.080659          # kg.m^2, identified from drop tests
B = 0.009004          # N.m.s/rad, identified from drop tests

# ESC
PWM_IDLE = 1500       # us
PWM_DEADBAND = 25     # us, Basic ESC deadband around neutral

# Steady-state extraction
HOLD_PWM_TOL_US = 5.0
STEADY_STATE_WINDOW_S = 1.0
LOOP_HZ_EST = 200     # used to estimate number of samples in last 1 s

# For mean K_T computation:
# Small PWM commands near the ESC deadband are often strongly nonlinear.
# Keep all points printed, but compute an "active region" mean separately.
MIN_ABS_DELTA_PWM_ACTIVE = 100.0


# =============================================================================
# HELPERS
# =============================================================================

def find_steady_state(df):
    """
    For each PWM target level, find the steady-state angle.

    The script takes the last STEADY_STATE_WINDOW_S during which the applied PWM
    is close to the target PWM.
    """

    results = []
    n_last = int(STEADY_STATE_WINDOW_S * LOOP_HZ_EST)

    for pwm_target, group in df.groupby("pwm_target_us"):
        pwm_target = float(pwm_target)

        if abs(pwm_target - PWM_IDLE) < 1e-9:
            continue

        mask = np.abs(group["pwm_current_us"] - pwm_target) < HOLD_PWM_TOL_US

        if mask.sum() < max(20, n_last // 2):
            print(f"[WARN] Skipping PWM {pwm_target:.0f} us: not enough hold samples.")
            continue

        steady = group[mask].iloc[-n_last:]

        theta_eq_deg = steady["angle_deg"].mean()
        theta_eq_std = steady["angle_deg"].std()

        results.append({
            "pwm_us": pwm_target,
            "delta_pwm": pwm_target - PWM_IDLE,
            "theta_eq_deg": theta_eq_deg,
            "theta_eq_std": theta_eq_std,
            "n_samples": len(steady),
        })

    eq_df = pd.DataFrame(results)

    if len(eq_df) == 0:
        return eq_df

    # Sort from smallest absolute thrust to largest absolute thrust
    eq_df = eq_df.sort_values("delta_pwm", ascending=False).reset_index(drop=True)

    return eq_df


def compute_kt(eq_df):
    """
    Compute K_T per equilibrium point.
    """

    theta_eq_rad = np.deg2rad(eq_df["theta_eq_deg"].to_numpy(dtype=float))
    delta_pwm = eq_df["delta_pwm"].to_numpy(dtype=float)

    gravity_torque = M_EQ * G * L_CM * np.sin(theta_eq_rad)

    K_T_per_point = gravity_torque / delta_pwm

    return theta_eq_rad, delta_pwm, gravity_torque, K_T_per_point


def print_kt_results(eq_df, K_T_per_point):
    """
    Print K_T values and summary.
    """

    print("\n--- K_T identification ---")
    print("Static model:")
    print("  K_T * (PWM - PWM_IDLE) = M_EQ * g * L_CM * sin(theta_eq)")
    print()
    print("K_T per point [N.m/us]:")

    for i, row in eq_df.iterrows():
        print(
            f"  PWM {row['pwm_us']:7.0f} us, "
            f"delta = {row['delta_pwm']:7.0f} us, "
            f"theta_eq = {row['theta_eq_deg']:8.2f} deg, "
            f"std = {row['theta_eq_std']:6.2f} deg, "
            f"K_T = {K_T_per_point[i]:.4e} N.m/us"
        )

    K_T_mean_all = float(np.mean(K_T_per_point))
    K_T_std_all = float(np.std(K_T_per_point))

    print()
    print("All points:")
    print(f"  Mean K_T = {K_T_mean_all:.4e} N.m/us")
    print(f"  Std  K_T = {K_T_std_all:.4e} N.m/us "
          f"({K_T_std_all / abs(K_T_mean_all) * 100:.1f}% CV)")

    active_mask = np.abs(eq_df["delta_pwm"].to_numpy(dtype=float)) >= MIN_ABS_DELTA_PWM_ACTIVE

    if active_mask.sum() >= 2:
        K_T_active = K_T_per_point[active_mask]
        K_T_mean_active = float(np.mean(K_T_active))
        K_T_std_active = float(np.std(K_T_active))

        print()
        print(f"Active region |delta_pwm| >= {MIN_ABS_DELTA_PWM_ACTIVE:.0f} us:")
        print(f"  Mean K_T = {K_T_mean_active:.4e} N.m/us")
        print(f"  Std  K_T = {K_T_std_active:.4e} N.m/us "
              f"({K_T_std_active / abs(K_T_mean_active) * 100:.1f}% CV)")
    else:
        K_T_mean_active = K_T_mean_all
        K_T_std_active = K_T_std_all

        print()
        print("[WARN] Not enough active-region points. Using all-points mean.")

    return K_T_mean_all, K_T_std_all, K_T_mean_active, K_T_std_active


def plot_results(df, eq_df, theta_eq_rad, delta_pwm, K_T_mean_active, csv_file):
    """
    Plot time series and static equilibrium fit.
    """

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharex=False)

    # -------------------------------------------------------------------------
    # Top plot: time series
    # -------------------------------------------------------------------------
    ax1b = ax1.twinx()

    angle_line, = ax1.plot(
        df["t_s"],
        df["angle_deg"],
        "b-",
        linewidth=1.2,
        label="angle [deg]",
    )

    pwm_line, = ax1b.plot(
        df["t_s"],
        df["pwm_current_us"],
        "r-",
        alpha=0.65,
        linewidth=1.0,
        label="PWM [us]",
    )

    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Angle [deg]", color="b")
    ax1b.set_ylabel("PWM [us]", color="r")

    ax1.tick_params(axis="y", labelcolor="b")
    ax1b.tick_params(axis="y", labelcolor="r")

    ax1.set_title(f"Step response time series\n{os.path.basename(csv_file)}")
    ax1.grid(True, alpha=0.3)

    lines = [angle_line, pwm_line]
    labels = [line.get_label() for line in lines]
    ax1.legend(lines, labels, loc="best")

    # -------------------------------------------------------------------------
    # Bottom plot: static equilibrium
    # -------------------------------------------------------------------------
    ax2.errorbar(
        delta_pwm,
        np.rad2deg(theta_eq_rad),
        yerr=eq_df["theta_eq_std"],
        fmt="o",
        markersize=8,
        capsize=4,
        label="measured equilibrium",
    )

    # Prediction over the full measured delta_pwm range
    dpwm_min = min(delta_pwm.min(), 0.0)
    dpwm_max = max(delta_pwm.max(), 0.0)

    # Add a small margin
    margin = 0.1 * max(abs(dpwm_min), abs(dpwm_max), 1.0)
    dpwm_range = np.linspace(dpwm_min - margin, dpwm_max + margin, 300)

    arg = K_T_mean_active * dpwm_range / (M_EQ * G * L_CM)
    arg = np.clip(arg, -1.0, 1.0)

    theta_pred_deg = np.rad2deg(np.arcsin(arg))

    ax2.plot(
        dpwm_range,
        theta_pred_deg,
        "k--",
        linewidth=1.5,
        label=f"prediction with K_T = {K_T_mean_active:.3e} N.m/us",
    )

    ax2.axvline(0, color="gray", linewidth=1)
    ax2.axhline(0, color="gray", linewidth=1)

    # ESC deadband display
    ax2.axvspan(
        -PWM_DEADBAND,
        PWM_DEADBAND,
        color="gray",
        alpha=0.15,
        label=f"ESC deadband ±{PWM_DEADBAND} us",
    )

    ax2.set_xlabel("delta PWM = PWM - 1500 [us]")
    ax2.set_ylabel("Equilibrium angle [deg]")
    ax2.set_title("Static equilibrium: measured vs predicted")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    os.makedirs("data", exist_ok=True)
    output_plot = "data/test03_step_fit.png"
    plt.savefig(output_plot, dpi=120)

    print(f"\n[OK] Plot saved to {output_plot}")
    plt.show()


# =============================================================================
# MAIN
# =============================================================================

def main():
    if len(sys.argv) < 2:
        files = sorted(glob.glob("data/test03_step_response_20260516_2140*.csv"))

        if not files:
            print("No data files found.")
            sys.exit(1)

        csv_file = files[-1]
    else:
        csv_file = sys.argv[1]

    print(f"[FIT] Loading {csv_file}")
    print()
    print("Using mechanical parameters:")
    print(f"  M_EQ = {M_EQ:.5f} kg")
    print(f"  L_CM = {L_CM:.5f} m")
    print(f"  J    = {J:.6f} kg.m^2")
    print(f"  B    = {B:.6f} N.m.s/rad")
    print(f"  M_EQ*g*L_CM = {M_EQ * G * L_CM:.4f} N.m")

    df = pd.read_csv(csv_file)

    required_cols = {
        "t_s",
        "pwm_current_us",
        "angle_deg",
        "pwm_target_us",
    }

    missing = required_cols - set(df.columns)

    if missing:
        raise RuntimeError(f"Missing columns in CSV: {missing}")

    eq_df = find_steady_state(df)

    if len(eq_df) == 0:
        print("[ERROR] No valid equilibrium point found.")
        sys.exit(1)

    print("\nSteady-state equilibrium points:")
    print(eq_df.to_string(index=False))

    theta_eq_rad, delta_pwm, gravity_torque, K_T_per_point = compute_kt(eq_df)

    K_T_mean_all, K_T_std_all, K_T_mean_active, K_T_std_active = print_kt_results(
        eq_df,
        K_T_per_point,
    )

    plot_results(
        df,
        eq_df,
        theta_eq_rad,
        delta_pwm,
        K_T_mean_active,
        csv_file,
    )


if __name__ == "__main__":
    main()