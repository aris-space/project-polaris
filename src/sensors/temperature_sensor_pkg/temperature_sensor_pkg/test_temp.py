import Jetson.GPIO as GPIO
import time

# Pin 7 on the header
DQ_PIN = 7

GPIO.setmode(GPIO.BOARD)


def read_sensor():
    # This is where we would 'wiggle' the pin to talk to the DS18B20
    # For now, let's just see if we can trigger the pin
    GPIO.setup(DQ_PIN, GPIO.OUT)
    GPIO.output(DQ_PIN, GPIO.LOW)
    time.sleep(0.0005)  # Reset pulse
    GPIO.output(DQ_PIN, GPIO.HIGH)
    print("Sent reset pulse to Pin 7. Checking for response...")


def main():
    try:
        read_sensor()
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
