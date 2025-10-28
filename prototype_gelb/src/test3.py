import pigpio
import time


pi = pigpio.pi()
if not pi.connected:
    print("Could not connect to pigpio daemon!")
    exit()
pin = 18  # Example GPIO pin
pi.set_servo_pulsewidth(pin, 1900)

print("PWM signal set to 1900 microseconds")
time.sleep(5)  # Wait for 5 seconds
print("waiting done.")

pi.set_servo_pulsewidth(pin, 0)  # stop pulses
print("PWM signal stopped")
pi.stop()
