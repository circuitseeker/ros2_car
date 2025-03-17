#!/usr/bin/env python3

import Jetson.GPIO as GPIO
import time

# Set up GPIO
led_pin = 40
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led_pin, GPIO.OUT)

print(f"LED control initialized on GPIO pin {led_pin}")
print("Press Ctrl+C to exit")

try:
    while True:
        # Turn LED on
        GPIO.output(led_pin, GPIO.HIGH)
        print("LED ON")
        time.sleep(1)
        
        # Turn LED off
        GPIO.output(led_pin, GPIO.LOW)
        print("LED OFF")
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    # Clean up GPIO
    GPIO.cleanup()
    print("GPIO cleaned up") 