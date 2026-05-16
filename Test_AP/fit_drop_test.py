#!/usr/bin/env python3
"""
fit_drop_test.py

Identify J (moment of inertia) and b (viscous damping) from passive drop-test data.

Model:
    J * theta_ddot + b * theta_dot + M_EQ * g * L_CM * sin(theta) = 0

Important correction:
    The script automatically detects the release time and trims the initial
    holding plateau before fitting. This avoids fitting the time interval where
    the arm was still held by hand.

Usage:
    python3 fit_drop_test.py
    python3 fit_drop_test.py data/test02_drop_run*.csv

Inputs:
    CSV files with columns:
        t_s, angle_deg

Outputs:
    Printed J, b values
    Plot:
        data/test02_drop_fit.png
"""

import sys
import os
import glob

import numpy as np
import pandas as pd
from scipy.integrate import odeint
from scipy.optimize import least_squares
import matplotlib.pyplot as plt


# =============================================================================
# MECHANICAL PARAMETERS FROM SOLIDWORKS
# =============================================================================

G = 9.81

M_EQ = 0.58270       # kg, moving AP assembly mass from SolidWorks
L_CM = 0.32714       # m, pivot-to-centre-of-mass distance
J_CAD = 0.07756      # kg.m^2, inertia about pivot axis from SolidWorks

# Point-mass reference, only used as a sanity check
J_POINT_MASS = M_EQ * L_CM**2


# =============================================================================
# FIT SETTINGS
# =============================================================================

# Release detection
RELEASE_VEL_THRESHOLD_DEG_S = 3.0     # angular velocity threshold
RELEASE_SUSTAINED_SAMPLES = 5         # must move for this many samples
SMOOTHING_WINDOW_S = 0.05             # velocity smoothing window

# Fit range
FIT_START_PADDING_S = 0.00            # no plateau before release
MAX_FIT_DURATION_S = 8.0              # use at most this duration after release

# Bounds for fitted parameters
J_MIN = 0.25 * J_CAD
J_MAX = 3.00 * J_CAD

B_MIN = 0.0
B_MAX = 0.10                          # N.m.s/rad, avoids nonphysical huge damping

OMEGA0_MIN = np.deg2rad(-60.0)        # rad/s
OMEGA0_MAX = np.deg2rad(+60.0)        # rad/s


# =============================================================================
# DYNAMICS
# =============================================================================

def model(state, t, J, b):
    """
    Free pendulum dynamics.

    State:
        theta [rad]
        omega [rad/s]
    """

    theta, omega = state

    theta_dot = omega
    omega_dot = -(b * omega + M_EQ * G * L_CM * np.sin(theta)) / J

    return [theta_dot, omega_dot]


def simulate(t, theta0, omega0, J, b):
    """
    Simulate the passive pendulum model.
    """

    sol = odeint(model, [theta0, omega0], t, args=(J, b))
    return sol[:, 0]


# =============================================================================
# DATA PRE-PROCESSING
# =============================================================================

def moving_average(x, window_samples):
    """
    Simple centred moving average.
    """

    if window_samples <= 1:
        return x

    window = np.ones(window_samples) / window_samples
    return np.convolve(x, window, mode="same")


def detect_release_index(t, theta_rad):
    """
    Detect the index at which the arm is released.

    The release is detected when the smoothed angular velocity exceeds
    RELEASE_VEL_THRESHOLD_DEG_S for RELEASE_SUSTAINED_SAMPLES consecutive samples.
    """

    dt = np.median(np.diff(t))

    if dt <= 0:
        return 0, np.zeros_like(theta_rad)

    omega_rad_s = np.gradient(theta_rad, t)
    omega_deg_s = np.rad2deg(omega_rad_s)

    window_samples = max(1, int(SMOOTHING_WINDOW_S / dt))
    if window_samples % 2 == 0:
        window_samples += 1

    omega_smooth_deg_s = moving_average(omega_deg_s, window_samples)

    moving = np.abs(omega_smooth_deg_s) > RELEASE_VEL_THRESHOLD_DEG_S

    for i in range(0, len(moving) - RELEASE_SUSTAINED_SAMPLES):
        if np.all(moving[i:i + RELEASE_SUSTAINED_SAMPLES]):
            padding_samples = int(FIT_START_PADDING_S / dt)
            release_idx = max(i - padding_samples, 0)
            return release_idx, np.deg2rad(omega_smooth_deg_s)

    # Fallback: no clear release detected
    return 0, np.deg2rad(omega_smooth_deg_s)


def trim_data_after_release(t_raw, theta_raw_rad):
    """
    Trim the initial holding plateau and return time reset to zero.
    """

    release_idx, omega_smooth_rad_s = detect_release_index(t_raw, theta_raw_rad)

    t_release_raw = t_raw[release_idx]

    t = t_raw[release_idx:] - t_release_raw
    theta = theta_raw_rad[release_idx:]
    omega_smooth = omega_smooth_rad_s[release_idx:]

    # Limit duration if needed
    keep = t <= MAX_FIT_DURATION_S
    t = t[keep]
    theta = theta[keep]
    omega_smooth = omega_smooth[keep]

    return t, theta, omega_smooth, release_idx, t_release_raw


# =============================================================================
# FIT
# =============================================================================

def residuals(params, t, theta_meas, theta0):
    """
    Residual vector for least-squares optimisation.

    params:
        J
        b
        omega0
    """

    J, b, omega0 = params

    theta_sim = simulate(t, theta0, omega0, J, b)

    return theta_sim - theta_meas


def fit_one(csv_file):
    """
    Fit one CSV file.
    """

    df = pd.read_csv(csv_file)

    t_raw = df["t_s"].to_numpy(dtype=float)
    theta_deg_raw = df["angle_deg"].to_numpy(dtype=float)
    theta_rad_raw = np.deg2rad(theta_deg_raw)

    # Make time start at zero in the raw file
    t_raw = t_raw - t_raw[0]

    # Trim initial holding plateau
    t, theta_rad, omega_smooth_rad_s, release_idx, t_release_raw = (
        trim_data_after_release(t_raw, theta_rad_raw)
    )

    if len(t) < 20:
        raise RuntimeError(f"Not enough samples after release in {csv_file}")

    theta0 = theta_rad[0]

    # Estimate a reasonable omega0 after release
    # This is only an initial guess. omega0 is fitted too.
    omega0_guess = float(omega_smooth_rad_s[0])

    # Initial guesses
    x0 = np.array([
        J_CAD,
        0.010,
        omega0_guess,
    ])

    lower_bounds = np.array([
        J_MIN,
        B_MIN,
        OMEGA0_MIN,
    ])

    upper_bounds = np.array([
        J_MAX,
        B_MAX,
        OMEGA0_MAX,
    ])

    result = least_squares(
        residuals,
        x0,
        bounds=(lower_bounds, upper_bounds),
        args=(t, theta_rad, theta0),
        loss="soft_l1",
        f_scale=np.deg2rad(0.5),
        max_nfev=500,
        xtol=1e-10,
        ftol=1e-10,
        gtol=1e-10,
    )

    J_fit, b_fit, omega0_fit = result.x

    theta_sim_rad = simulate(t, theta0, omega0_fit, J_fit, b_fit)

    rss = float(np.sum((theta_sim_rad - theta_rad) ** 2))
    rmse_deg = float(np.sqrt(np.mean((np.rad2deg(theta_sim_rad - theta_rad)) ** 2)))

    print(f"\n--- {os.path.basename(csv_file)} ---")
    print(f"  Raw duration:             {t_raw[-1]:8.3f} s")
    print(f"  Release detected at:      {t_release_raw:8.3f} s")
    print(f"  Samples used:             {len(t):8d}")
    print(f"  Fit duration:             {t[-1]:8.3f} s")
    print(f"  Initial angle used:       {np.rad2deg(theta0):8.3f} deg")
    print(f"  omega0_fit:               {np.rad2deg(omega0_fit):8.3f} deg/s")
    print()
    print(f"  J_fit:                    {J_fit*1000:8.3f} g.m^2")
    print(f"                            {J_fit:8.6f} kg.m^2")
    print(f"  b_fit:                    {b_fit*1000:8.3f} mN.m.s/rad")
    print(f"                            {b_fit:8.6f} N.m.s/rad")
    print()
    print(f"  J_CAD:                    {J_CAD:8.6f} kg.m^2")
    print(f"  J point-mass:             {J_POINT_MASS:8.6f} kg.m^2")
    print(f"  Ratio J_fit / J_CAD:      {J_fit/J_CAD:8.3f}")
    print(f"  Ratio J_fit / J_point:    {J_fit/J_POINT_MASS:8.3f}")
    print()
    print(f"  Cost RSS:                 {rss:8.6f}")
    print(f"  RMSE:                     {rmse_deg:8.4f} deg")

    # Return raw and fitted data for plotting
    return {
        "file": csv_file,
        "t_raw": t_raw,
        "theta_raw_rad": theta_rad_raw,
        "t": t,
        "theta_rad": theta_rad,
        "theta_sim_rad": theta_sim_rad,
        "release_idx": release_idx,
        "t_release_raw": t_release_raw,
        "J": J_fit,
        "b": b_fit,
        "omega0": omega0_fit,
        "rss": rss,
        "rmse_deg": rmse_deg,
    }


# =============================================================================
# MAIN
# =============================================================================

def main():
    if len(sys.argv) < 2:
        pattern = "data/test02_drop_run*.csv"
        files = sorted(glob.glob(pattern))

        if not files:
            print(f"No files matching {pattern}")
            sys.exit(1)
    else:
        files = sys.argv[1:]

    print(f"Fitting {len(files)} run(s)...")
    print(f"Using:")
    print(f"  M_EQ  = {M_EQ:.5f} kg")
    print(f"  L_CM  = {L_CM:.5f} m")
    print(f"  J_CAD = {J_CAD:.5f} kg.m^2")
    print()
    print("The script automatically detects release time and removes the initial plateau.")

    results = []

    fig, ax = plt.subplots(figsize=(10, 6))

    for f in files:
        try:
            res = fit_one(f)
            results.append(res)
        except Exception as e:
            print(f"\n[SKIP] {os.path.basename(f)}: {e}")
            continue

        base = os.path.basename(f)

        if "_run" in base:
            run = base.split("_run", 1)[1].split("_", 1)[0]
        else:
            run = os.path.splitext(base)[0]

        ax.plot(
            res["t"],
            np.rad2deg(res["theta_rad"]),
            "o",
            markersize=2,
            alpha=0.35,
            label=f"run {run} measured",
        )

        ax.plot(
            res["t"],
            np.rad2deg(res["theta_sim_rad"]),
            "-",
            linewidth=2,
            label=(
                f"run {run} fit: "
                f"J={res['J']*1000:.1f} g.m², "
                f"b={res['b']*1000:.1f} mN.m.s/rad"
            ),
        )

    if not results:
        print("\nNo valid result.")
        sys.exit(1)

    ax.set_xlabel("Time after release [s]")
    ax.set_ylabel("Angle [deg]")
    ax.set_title("AP drop test: measured vs identified passive model")
    ax.legend(fontsize=8, loc="best")
    ax.grid(True)

    # Summary
    Js = np.array([r["J"] for r in results])
    bs = np.array([r["b"] for r in results])
    rmses = np.array([r["rmse_deg"] for r in results])

    print("\n--- SUMMARY ---")
    print(f"Valid runs: {len(results)}")
    print()
    print(f"J mean:  {Js.mean()*1000:8.3f} g.m^2")
    print(f"J std:   {Js.std()*1000:8.3f} g.m^2")
    print(f"J mean:  {Js.mean():8.6f} kg.m^2")
    print(f"J CAD:   {J_CAD:8.6f} kg.m^2")
    print(f"J/J_CAD: {Js.mean()/J_CAD:8.3f}")
    print()
    print(f"b mean:  {bs.mean()*1000:8.3f} mN.m.s/rad")
    print(f"b std:   {bs.std()*1000:8.3f} mN.m.s/rad")
    print(f"b mean:  {bs.mean():8.6f} N.m.s/rad")
    print()
    print(f"RMSE mean: {rmses.mean():8.4f} deg")

    plt.tight_layout()

    os.makedirs("data", exist_ok=True)
    output_plot = "data/test02_drop_fit.png"
    plt.savefig(output_plot, dpi=120)

    print(f"\n[OK] Plot saved to {output_plot}")
    plt.show()


if __name__ == "__main__":
    main()