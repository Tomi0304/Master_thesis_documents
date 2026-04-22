#!/usr/bin/env python3
"""
TRMS — Test automatique du dwell minimum ESC BLHeli_S
Determine le temps minimum au neutre avant changement de sens
sans provoquer de re-armement.

Protocole par essai :
  1. Reverse a PWM_REV pendant T_SPIN secondes (moteur qui tourne)
  2. Neutre pendant DWELL millisecondes (valeur testee)
  3. Forward a PWM_FWD pendant T_CHECK secondes
  4. Verifie si l'encodeur bouge dans le bon sens (forward = descente = psi diminue)
     Si oui : changement de sens OK
     Si non : re-armement ESC detecte

Usage :
    sudo pigpiod
    python3 trms_dwell_test.py
"""

import pigpio
import time
import threading
import sys

ENCODER_PIN_A = 27
ENCODER_PIN_B = 17
ESC_PIN       = 12

ESC_NEUTRAL   = 1500
PWM_REV       = 1300   # reverse : moteur pousse vers le haut
PWM_FWD       = 1650   # forward : moteur pousse vers le bas
T_SPIN        = 2.0    # secondes de spin avant le changement
T_CHECK       = 1.5    # secondes pour verifier la reponse apres changement
T_REST        = 3.0    # repos entre chaque essai (retour neutre)

ENCODER_CPR_X4 = 2000
_QUAD = [0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0]

_pos  = 0
_lock = threading.Lock()
_last = 0

def _cb(gpio, level, tick):
    global _last
    a = pi.read(ENCODER_PIN_A)
    b = pi.read(ENCODER_PIN_B)
    cur = (a << 1) | b
    with _lock:
        global _pos
        _pos += _QUAD[(_last << 2) | cur]
    _last = cur

def get_pos():
    with _lock:
        return _pos

def reset_pos():
    with _lock:
        global _pos
        _pos = 0

def set_pwm(pw):
    pi.set_servo_pulsewidth(ESC_PIN, int(pw))

def wait_ms(ms):
    time.sleep(ms / 1000.0)

pi = pigpio.pi()
if not pi.connected:
    print("ERREUR : sudo pigpiod d'abord")
    sys.exit(1)

pi.set_mode(ENCODER_PIN_A, pigpio.INPUT)
pi.set_mode(ENCODER_PIN_B, pigpio.INPUT)
pi.set_pull_up_down(ENCODER_PIN_A, pigpio.PUD_OFF)
pi.set_pull_up_down(ENCODER_PIN_B, pigpio.PUD_OFF)
a = pi.read(ENCODER_PIN_A)
b = pi.read(ENCODER_PIN_B)
_last = (a << 1) | b
pi.callback(ENCODER_PIN_A, pigpio.EITHER_EDGE, _cb)
pi.callback(ENCODER_PIN_B, pigpio.EITHER_EDGE, _cb)

print("=" * 60)
print("  TRMS — Test dwell minimum ESC")
print(f"  Reverse  : {PWM_REV} us pendant {T_SPIN}s")
print(f"  Forward  : {PWM_FWD} us pendant {T_CHECK}s")
print(f"  Encodeur : detection reponse moteur")
print("=" * 60)
print()

print("[ESC] Armement (neutre 3s)...")
set_pwm(ESC_NEUTRAL)
time.sleep(3)
print("[ESC] Arme\n")

# Valeurs de dwell a tester (ms), du plus grand au plus petit
dwell_values = [400, 300, 200, 150, 100, 75, 50, 30, 20, 10]

results = []

try:
    for dwell_ms in dwell_values:
        print(f"--- Essai dwell = {dwell_ms} ms ---")

        # 1. Spin en reverse
        print(f"  [1] Reverse {PWM_REV}us pendant {T_SPIN}s...")
        set_pwm(PWM_REV)
        time.sleep(T_SPIN)

        # Mesure encodeur pendant le reverse (pour confirmer que le moteur tourne)
        reset_pos()
        time.sleep(0.3)
        pos_during_rev = get_pos()

        # 2. Neutre pendant DWELL ms
        print(f"  [2] Neutre {dwell_ms}ms...")
        set_pwm(ESC_NEUTRAL)
        wait_ms(dwell_ms)

        # 3. Forward
        print(f"  [3] Forward {PWM_FWD}us pendant {T_CHECK}s...")
        reset_pos()
        t_start = time.monotonic()
        set_pwm(PWM_FWD)

        # Surveille l'encodeur pendant T_CHECK
        # Si le moteur repond en forward, psi doit diminuer → encodeur dans le bon sens
        time.sleep(0.5)   # laisse le temps de demarrer
        pos_after_500ms = get_pos()
        time.sleep(T_CHECK - 0.5)
        pos_final = get_pos()

        # Retour au neutre
        set_pwm(ESC_NEUTRAL)

        # Analyse
        # En forward, le bras descend → encodeur en sens positif (get_psi diminue)
        # On regarde juste si l'encodeur a bougé (>5 counts = moteur actif)
        encoder_moved = abs(pos_final) > 5
        encoder_dir_ok = pos_final > 0   # forward doit faire monter les counts (descente bras)

        if encoder_moved and encoder_dir_ok:
            status = "OK"
            print(f"  → SUCCES : encodeur a bouge de {pos_final} counts en {T_CHECK}s")
        elif encoder_moved and not encoder_dir_ok:
            status = "MAUVAIS_SENS"
            print(f"  → ATTENTION : moteur a tourne mais dans le mauvais sens ({pos_final} counts)")
        else:
            status = "REARMEMENT"
            print(f"  → RE-ARMEMENT ESC detecte (encodeur immobile : {pos_final} counts)")

        results.append((dwell_ms, status, pos_final))

        # Repos entre essais
        print(f"  Repos {T_REST}s...")
        time.sleep(T_REST)
        print()

finally:
    set_pwm(ESC_NEUTRAL)
    time.sleep(0.5)
    set_pwm(0)
    pi.stop()

print("=" * 60)
print("  RESULTATS")
print("=" * 60)
print(f"{'Dwell (ms)':>12} {'Statut':>15} {'Counts':>8}")
print("-" * 40)
min_ok = None
for dwell_ms, status, counts in results:
    marker = " ← LIMITE" if status == "REARMEMENT" and min_ok is not None else ""
    print(f"{dwell_ms:>12} {status:>15} {counts:>8}{marker}")
    if status == "OK" and min_ok is None:
        pass
    if status == "OK":
        min_ok = dwell_ms

print()
if min_ok is not None:
    print(f"→ Dwell minimum recommande : {min_ok} ms")
    print(f"→ Marge de securite (x1.5) : {int(min_ok*1.5)} ms")
else:
    print("→ Tous les essais ont echoue — verifier le cablage ESC")