import pigpio
import time


pi = pigpio.pi()
if not pi.connected:
    print("Could not connect to pigpio daemon!")
    exit()

pin = 18  # Example GPIO pin

pi.write(pin, 1)  # Set pin high
print("Pin set to HIGH")
time.sleep(5)  # Wait for 5 seconds
print("waiting for done.")

pi.write(pin, 0)  # Set pin low
print("Pin set to LOW")
pi.stop()
