#!/usr/bin/env python3
"""
Test HEDS-5540 - Encodeur optique quadrature
CH.A → GPIO 27 (pin 13)
CH.B → GPIO 17 (pin 11)

Dépendance : pip install pigpio
Démarrer le daemon avant : sudo pigpiod
"""

import pigpio
import time
import signal
import sys

# ── Configuration ──────────────────────────────────────────────
GPIO_A = 27       # CH.A
GPIO_B = 17       # CH.B
CPR    = 500      # Counts Per Revolution du HEDS-5540 A12
                  # (500 impulsions × 4 états quadrature = 2000 ticks/tour)

# ── Variables globales ─────────────────────────────────────────
position    = 0   # ticks absolus
last_A      = 0
last_B      = 0
last_time   = time.time()
last_pos    = 0

# Table de décodage quadrature (état précédent << 2 | état actuel)
# +1 = sens horaire, -1 = sens anti-horaire, 0 = erreur/rebond
QUAD_TABLE = [
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
]

last_state = 0

def encoder_callback(gpio, level, tick):
    global position, last_state, last_A, last_B

    # Lire l'état actuel des deux canaux
    a = pi.read(GPIO_A)
    b = pi.read(GPIO_B)

    current_state = (a << 1) | b
    index         = (last_state << 2) | current_state
    position     += QUAD_TABLE[index]
    last_state    = current_state

def get_angle_deg():
    """Angle en degrés (position absolue, peut dépasser 360)"""
    return (position / (CPR * 4)) * 360.0

def get_turns():
    """Nombre de tours complets"""
    return position / (CPR * 4)

def get_rpm():
    """Vitesse en RPM calculée sur la dernière période"""
    global last_time, last_pos
    now     = time.time()
    dt      = now - last_time
    if dt < 0.05:          # éviter division par zéro
        return 0.0
    delta   = position - last_pos
    rpm     = (delta / (CPR * 4)) / dt * 60.0
    last_time = now
    last_pos  = position
    return rpm

def reset_position():
    global position, last_state
    position   = 0
    last_state = (pi.read(GPIO_A) << 1) | pi.read(GPIO_B)
    print("  → Position remise à zéro")

def signal_handler(sig, frame):
    print("\n\nArrêt propre...")
    cb_a.cancel()
    cb_b.cancel()
    pi.stop()
    sys.exit(0)

# ── Init pigpio ────────────────────────────────────────────────
pi = pigpio.pi()
if not pi.connected:
    print("ERREUR : pigpiod n'est pas lancé.")
    print("Lancer d'abord : sudo pigpiod")
    sys.exit(1)

pi.set_mode(GPIO_A, pigpio.INPUT)
pi.set_mode(GPIO_B, pigpio.INPUT)
pi.set_pull_up_down(GPIO_A, pigpio.PUD_OFF)  
pi.set_pull_up_down(GPIO_B, pigpio.PUD_OFF)

# État initial
last_state = (pi.read(GPIO_A) << 1) | pi.read(GPIO_B)

# Callbacks sur les deux fronts (montant ET descendant)
cb_a = pi.callback(GPIO_A, pigpio.EITHER_EDGE, encoder_callback)
cb_b = pi.callback(GPIO_B, pigpio.EITHER_EDGE, encoder_callback)

signal.signal(signal.SIGINT, signal_handler)

# ── Interface console ──────────────────────────────────────────
print("=" * 55)
print("  Test HEDS-5540 — Encodeur quadrature")
print("  CH.A → GPIO 27  |  CH.B → GPIO 17")
print(f"  Résolution : {CPR} CPR × 4 = {CPR*4} ticks/tour")
print("=" * 55)
print("  Commandes : [r] reset  |  [q] quitter")
print("=" * 55)
print()

try:
    while True:
        rpm   = get_rpm()
        angle = get_angle_deg()
        turns = get_turns()
        a_raw = pi.read(GPIO_A)
        b_raw = pi.read(GPIO_B)

        # Effacer la ligne et afficher
        print(
            f"\r  Ticks: {position:+7d}  |  "
            f"Angle: {angle:+9.2f}°  |  "
            f"Tours: {turns:+7.3f}  |  "
            f"RPM: {rpm:+7.1f}  |  "
            f"A={a_raw} B={b_raw}   ",
            end="", flush=True
        )

except KeyboardInterrupt:
    signal_handler(None, None)