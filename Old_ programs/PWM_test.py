import time
import pigpio

ESC_GPIO = 18
STOP = 1500
TEST = 1100

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpiod non connecté")

pi.set_servo_pulsewidth(ESC_GPIO, STOP)

time.sleep(1)
print("ESC armé")

try:
    input("Test")
    pi.set_servo_pulsewidth(ESC_GPIO, TEST)
    print(f"Commande {TEST} µs")
    time.sleep(10)

    print("Retour neutre")
    pi.set_servo_pulsewidth(ESC_GPIO, STOP)
    time.sleep(2)

finally:
    pi.set_servo_pulsewidth(ESC_GPIO, STOP)
    time.sleep(2)
    pi.stop()