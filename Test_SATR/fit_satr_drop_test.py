#!/usr/bin/env python3
"""
fit_satr_drop_test.py

Fit J et b pour le SATR a partir des drop tests passifs.

Modele physique :
    J*theta_ddot + b*theta_dot + m_eq*g*l_cm*sin(theta) = 0

avec theta mesure depuis la verticale-bas (au repos).

Sur le SATR, theta_repos correspond a la position d'equilibre du beam,
qui peut etre legerement biaisee par l'asymetrie de masse (IMU cote main).
Sur ce fit, on suppose theta_repos = 0 (donc encoder zero est aligne
avec l'equilibre).

Optimisation Nelder-Mead sur (J, b, omega0_initial).

A REMPLIR : valeurs CAD du SATR :
    M_EQ_CAD : masse totale rotative (kg)
    L_CM_CAD : distance pivot -> CdG (m), signe selon convention
    J_CAD    : moment d'inertie autour pivot (kg.m^2)

Usage:
    python3 fit_satr_drop_test.py logs/satr_drop_run*.csv
"""

import sys
import glob
import argparse
import numpy as np
import pandas as pd
from scipy.optimize import minimize
from scipy.integrate import odeint


# ===========================================================================
# A REMPLIR depuis SolidWorks (proprietes de masse SATR dans repere pivot)
# ===========================================================================
M_EQ_CAD = 0.800       # kg     - A METTRE A JOUR
L_CM_CAD = 0.000       # m      - A METTRE A JOUR (~ 0 si beam symetrique)
J_CAD    = 0.020       # kg.m^2 - A METTRE A JOUR
# ===========================================================================

G = 9.81  # m/s^2


def pendulum_ode(state, t, J, b, m_eq, l_cm):
    """
    Modele du pendule physique avec amortissement visqueux.

    state = [theta, theta_dot]  en rad et rad/s
    """
    theta, theta_dot = state
    theta_ddot = -(b * theta_dot + m_eq * G * l_cm * np.sin(theta)) / J
    return [theta_dot, theta_ddot]


def simulate(theta0, omega0, t_array, J, b, m_eq, l_cm):
    """
    Simule la trajectoire du pendule sur t_array.
    Retourne theta en degres.
    """
    sol = odeint(pendulum_ode, [theta0, omega0], t_array,
                 args=(J, b, m_eq, l_cm))
    return np.degrees(sol[:, 0])


def cost(params, t_data, angle_data, m_eq, l_cm):
    """
    Cout = somme des erreurs au carre entre simulation et donnees.
    Parametres : J, b, omega0 (vitesse initiale au lacher)
    """
    J, b, omega0 = params
    if J <= 0 or b < 0:
        return 1e10

    theta0_rad = np.radians(angle_data[0])
    angle_sim = simulate(theta0_rad, omega0, t_data, J, b, m_eq, l_cm)

    rss = np.sum((angle_sim - angle_data) ** 2)
    return rss


def detect_release(t, angle, threshold_dps=10.0):
    """
    Detecte le moment du lacher : premier point ou |dangle/dt| > threshold.
    """
    dt = np.diff(t)
    da = np.diff(angle)
    velocity = da / dt
    for i, v in enumerate(velocity):
        if abs(v) > threshold_dps:
            return i
    return 0


def fit_one_run(csv_path, m_eq, l_cm, j_cad, truncate_at=None,
                detect_release_auto=True, verbose=True):
    """
    Fit J, b, omega0 sur un fichier CSV.

    truncate_at : si pas None, ne fitte que les N premieres secondes.
                  Utile pour exclure les rebonds contre la butee.
    """
    df = pd.read_csv(csv_path)
    t_all = df['t_s'].values
    a_all = df['angle_deg'].values

    if verbose:
        print(f"\n--- {csv_path.split('/')[-1]} ---")
        print(f"  Raw duration:    {t_all[-1]:.3f} s")
        print(f"  Raw range:       [{a_all.min():.2f}, {a_all.max():.2f}] deg")

    # Detect release
    if detect_release_auto:
        rel_idx = detect_release(t_all, a_all)
        if verbose:
            print(f"  Release detected at idx {rel_idx} (t={t_all[rel_idx]:.3f}s, "
                  f"angle={a_all[rel_idx]:.2f}°)")
    else:
        rel_idx = 0

    t = t_all[rel_idx:] - t_all[rel_idx]
    a = a_all[rel_idx:]

    # Truncate if requested
    if truncate_at is not None:
        mask = t <= truncate_at
        t = t[mask]
        a = a[mask]
        if verbose:
            print(f"  Truncated to {truncate_at}s -> {len(t)} samples")

    # Initial parameter guess
    J0 = j_cad
    b0 = 0.010      # 10 mN.m.s/rad (proche de AP)
    omega0_0 = 0.0  # rest

    # Run optimisation
    result = minimize(
        cost,
        x0=[J0, b0, omega0_0],
        args=(t, a, m_eq, l_cm),
        method='Nelder-Mead',
        options={'xatol': 1e-6, 'fatol': 1e-6, 'maxiter': 5000}
    )

    J_fit, b_fit, omega0_fit = result.x

    # Compute residuals
    theta0_rad = np.radians(a[0])
    a_sim = simulate(theta0_rad, omega0_fit, t, J_fit, b_fit, m_eq, l_cm)
    rmse = np.sqrt(np.mean((a_sim - a) ** 2))
    rss = result.fun

    if verbose:
        print(f"  Initial angle:   {a[0]:.2f} deg")
        print(f"  omega0_fit:      {np.degrees(omega0_fit):.2f} deg/s")
        print()
        print(f"  J_fit:           {J_fit*1000:.3f} g.m^2")
        print(f"                   {J_fit:.6f} kg.m^2")
        print(f"  b_fit:           {b_fit*1000:.3f} mN.m.s/rad")
        print(f"                   {b_fit:.6f} N.m.s/rad")
        print()
        print(f"  J_CAD:           {j_cad:.6f} kg.m^2")
        print(f"  Ratio J_fit/CAD: {J_fit/j_cad:.3f}")
        print()
        print(f"  Cost RSS:        {rss:.4f}")
        print(f"  RMSE:            {rmse:.4f} deg")

    return {
        'file': csv_path,
        'J_fit': J_fit,
        'b_fit': b_fit,
        'omega0': omega0_fit,
        'rmse': rmse,
        'rss': rss,
        'initial_angle': a[0],
        't_data': t,
        'angle_data': a,
        'angle_sim': a_sim,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("files", nargs="+", help="CSV file(s) to fit")
    p.add_argument("--truncate", type=float, default=None,
                   help="Truncate fit at N seconds (useful if bump at butée)")
    p.add_argument("--m-eq", type=float, default=M_EQ_CAD)
    p.add_argument("--l-cm", type=float, default=L_CM_CAD)
    p.add_argument("--j-cad", type=float, default=J_CAD)
    p.add_argument("--no-plot", action="store_true")
    args = p.parse_args()

    print(f"Using:")
    print(f"  M_EQ  = {args.m_eq:.5f} kg")
    print(f"  L_CM  = {args.l_cm:.5f} m")
    print(f"  J_CAD = {args.j_cad:.5f} kg.m^2")
    if args.truncate:
        print(f"  Truncating fits at {args.truncate}s")
    print()

    # Expand globs
    file_list = []
    for f in args.files:
        if '*' in f:
            file_list.extend(sorted(glob.glob(f)))
        else:
            file_list.append(f)

    print(f"Fitting {len(file_list)} run(s)...")
    print()

    results = [fit_one_run(f, args.m_eq, args.l_cm, args.j_cad,
                           truncate_at=args.truncate)
               for f in file_list]

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)

    Js = [r['J_fit'] for r in results]
    bs = [r['b_fit'] for r in results]
    rmses = [r['rmse'] for r in results]

    print(f"\nValid runs: {len(results)}")
    print()
    print(f"J mean:    {np.mean(Js)*1000:.3f} g.m^2")
    print(f"J std:     {np.std(Js)*1000:.3f} g.m^2")
    print(f"J mean:    {np.mean(Js):.6f} kg.m^2")
    print(f"J CAD:     {args.j_cad:.6f} kg.m^2")
    print(f"Ratio:     {np.mean(Js)/args.j_cad:.3f}")
    print()
    print(f"b mean:    {np.mean(bs)*1000:.3f} mN.m.s/rad")
    print(f"b std:     {np.std(bs)*1000:.3f} mN.m.s/rad")
    print(f"b mean:    {np.mean(bs):.6f} N.m.s/rad")
    print()
    print(f"RMSE mean: {np.mean(rmses):.4f} deg")

    # Plot
    if not args.no_plot:
        try:
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(len(results), 1, figsize=(10, 3*len(results)),
                                       sharex=True)
            if len(results) == 1:
                axes = [axes]
            for ax, r in zip(axes, results):
                ax.plot(r['t_data'], r['angle_data'], 'b-', label='data', alpha=0.7)
                ax.plot(r['t_data'], r['angle_sim'], 'r--', label='fit', linewidth=1.5)
                ax.set_ylabel('angle [deg]')
                ax.grid(True, alpha=0.3)
                ax.legend(loc='upper right', fontsize=8)
                fname = r['file'].split('/')[-1]
                ax.set_title(f"{fname} — J={r['J_fit']*1000:.1f} g.m², "
                              f"b={r['b_fit']*1000:.1f} mN.m.s/rad, "
                              f"RMSE={r['rmse']:.2f}°", fontsize=9)
            axes[-1].set_xlabel('time [s]')
            plt.tight_layout()
            plt.savefig('satr_drop_fit.png', dpi=100, bbox_inches='tight')
            print(f"\n[OK] Plot saved to satr_drop_fit.png")
        except ImportError:
            print("\n[WARN] matplotlib not available, skipping plot")


if __name__ == "__main__":
    main()