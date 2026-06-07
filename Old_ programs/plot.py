import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


# ============================================================================
# Paramètres
# ============================================================================

csv_path = Path(__file__).resolve().parent / "logs" / "trms_kp0.1_kd0.1_kg0.9_sp30.0_thr0.7_20260326_151141.csv"
output_path = csv_path.with_suffix(".png")

if not csv_path.exists():
    raise FileNotFoundError(f"Fichier CSV introuvable : {csv_path}")

with open(csv_path, newline="") as f:
    reader = csv.DictReader(f)
    rows = list(reader)

if not rows:
    raise ValueError("Le CSV est vide.")

# ============================================================================
# Lecture des données
# ============================================================================

t   = np.array([float(r["t_s"]) for r in rows])
psi = np.array([float(r["psi_imu_deg"]) for r in rows])
enc = np.array([float(r["psi_enc_deg"]) for r in rows])
dot = np.array([float(r["psi_dot_dps"]) for r in rows])
err = np.array([float(r["error_deg"]) for r in rows])
u   = np.array([float(r["u_norm"]) for r in rows])
up  = np.array([float(r["u_p"]) for r in rows])
ud  = np.array([float(r["u_d"]) for r in rows])
ug  = np.array([float(r["u_g"]) for r in rows])
pw  = np.array([float(r["pwm_us"]) for r in rows])

setpoint = float(rows[0]["setpoint_deg"])

# ============================================================================
# Stats
# ============================================================================

ss_mask = t > 5.0
if np.any(ss_mask):
    avg_psi = np.mean(psi[ss_mask])
    avg_pwm = np.mean(pw[ss_mask])
    avg_err = np.mean(np.abs(err[ss_mask]))
else:
    avg_psi = np.mean(psi)
    avg_pwm = np.mean(pw)
    avg_err = np.mean(np.abs(err))

max_psi = np.max(psi)

if abs(setpoint) > 1e-9:
    os_pct = max(0.0, (max_psi - setpoint) / abs(setpoint) * 100.0)
else:
    os_pct = 0.0

# ============================================================================
# Figure
# ============================================================================

plt.style.use("dark_background")
fig, axes = plt.subplots(5, 1, figsize=(14, 18), sharex=True)

fig.suptitle(
    f"Aero-Pendulum avec cage PLA — PD + Gravité\n"
    f"Setpoint={setpoint:.1f}°",
    fontsize=14,
    fontweight="bold",
    color="white",
    y=0.98
)

# 1 - Position angulaire
ax = axes[0]
ax.plot(t, psi, color="#3b82f6", linewidth=1.5, label="ψ IMU")
ax.plot(t, enc, color="#888888", linewidth=0.9, linestyle="--", label="ψ Encodeur")
ax.axhline(y=setpoint, color="#e8a84a", linewidth=1.5, linestyle="--", alpha=0.8,
           label=f"Consigne {setpoint:.1f}°")
ax.axhline(y=0, color="#333333", linewidth=0.5)
ax.set_ylabel("Angle [°]")
ax.set_title("Position angulaire ψ(t)", fontsize=11, color="#aaaaaa")
ax.legend(loc="upper right", fontsize=9)
ax.set_ylim(min(np.min(enc), np.min(psi), -10) - 2, max(np.max(psi), setpoint, 40) + 2)
ax.grid(True, alpha=0.15)

ax.text(
    0.98, 0.05,
    f"Régime permanent: {avg_psi:.1f}°  |  Erreur moy: {avg_err:.2f}°  |  Overshoot: {os_pct:.1f}%",
    transform=ax.transAxes,
    fontsize=9,
    color="#888888",
    ha="right",
    va="bottom",
    bbox=dict(boxstyle="round,pad=0.3", facecolor="#111111", edgecolor="#333333")
)

# 2 - Erreur
ax = axes[1]
ax.fill_between(t, err, 0, alpha=0.15, color="#e85d4a")
ax.plot(t, err, color="#e85d4a", linewidth=1.2)
ax.axhline(y=0, color="#4ae870", linewidth=0.8, linestyle="--", alpha=0.5)
ax.set_ylabel("Erreur [°]")
ax.set_title("Erreur de suivi e(t) = consigne - ψ", fontsize=11, color="#aaaaaa")
ax.grid(True, alpha=0.15)

# 3 - Commande décomposée
ax = axes[2]
ax.plot(t, u,  color="#e0e0e0", linewidth=1.8, label="u total", zorder=5)
ax.plot(t, up, color="#3b82f6", linewidth=1.0, label="u_P")
ax.plot(t, ud, color="#e85d4a", linewidth=1.0, label="u_D")
ax.plot(t, ug, color="#e8a84a", linewidth=1.0, linestyle="--", label="u_G")
ax.set_ylabel("Commande [-]")
ax.set_title("Commande décomposée u(t) = u_P + u_D + u_G", fontsize=11, color="#aaaaaa")
ax.legend(loc="upper right", fontsize=9, ncol=2)
ax.grid(True, alpha=0.15)

ax.text(
    0.98, 0.05,
    f"u_G final ≈ {ug[-1]:.3f}",
    transform=ax.transAxes,
    fontsize=9,
    color="#888888",
    ha="right",
    va="bottom",
    bbox=dict(boxstyle="round,pad=0.3", facecolor="#111111", edgecolor="#333333")
)

# 4 - PWM
ax = axes[3]
ax.plot(t, pw, color="#a855f7", linewidth=1.5)
ax.axhline(y=1500, color="#555555", linewidth=0.8, linestyle="--", label="Neutre (1500 µs)")
ax.set_ylabel("PWM [µs]")
ax.set_title("Signal PWM vers ESC", fontsize=11, color="#aaaaaa")
ax.legend(loc="upper right", fontsize=9)
ax.set_ylim(min(np.min(pw) - 20, 1050), max(np.max(pw) + 20, 1550))
ax.grid(True, alpha=0.15)

ax.text(
    0.98, 0.05,
    f"PWM régime permanent: {avg_pwm:.0f} µs",
    transform=ax.transAxes,
    fontsize=9,
    color="#888888",
    ha="right",
    va="bottom",
    bbox=dict(boxstyle="round,pad=0.3", facecolor="#111111", edgecolor="#333333")
)

# 5 - Vitesse angulaire
ax = axes[4]
ax.plot(t, dot, color="#14b8a6", linewidth=1.2)
ax.axhline(y=0, color="#333333", linewidth=0.5)
ax.set_ylabel("ψ̇ [°/s]")
ax.set_xlabel("Temps [s]")
ax.set_title("Vitesse angulaire ψ̇(t)", fontsize=11, color="#aaaaaa")
ax.grid(True, alpha=0.15)

plt.tight_layout(rect=(0, 0, 1, 0.96))
plt.savefig(output_path, dpi=200, bbox_inches="tight")
print(f"Graphe sauvegardé dans : {output_path}")