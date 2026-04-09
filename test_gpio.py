# Test GPIO state on Pin 11 (GPIO4_C6)
# Requirements: Install OPi.GPIO (sudo pip3 install OPi.GPIO)
# How to run: sudo python3 test_gpio.py

import OPi.GPIO as GPIO
import time

PWM_PIN = 26  # Physical pin 11 (GPIO4_C6)

GPIO.setmode(GPIO.BOARD)
try:
    GPIO.setup(PWM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)  # Disable pull-up/pull-down
    print(f"GPIO pin {PWM_PIN} (GPIO4_C6) set up as input with pull-up disabled")
except Exception as e:
    print(f"Error setting up GPIO: {e}")
    exit(1)

try:
    while True:
        state = GPIO.input(PWM_PIN)
        print(f"Pin 26 state: {state}")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    GPIO.cleanup()
    print("GPIO cleaned up")
    