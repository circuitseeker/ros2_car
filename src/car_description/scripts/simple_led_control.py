#!/usr/bin/env python3

import Jetson.GPIO as GPIO
import sys
import termios
import tty
import time

# Set up GPIO
led_pin = 40
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led_pin, GPIO.OUT)

# Key mapping to LED state
key_mapping = {
    'w': True,     # Forward - LED on
    's': True,     # Backward - LED on
    'a': True,     # Left - LED on
    'd': True,     # Right - LED on
    'q': False,    # Stop - LED off
    'l': None,     # Toggle
}

def get_key():
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)
    try:
        # Set terminal to raw mode
        tty.setraw(sys.stdin.fileno())
        # Read a single character
        key = sys.stdin.read(1)
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    print("Simple LED Control with Keyboard")
    print("--------------------------------")
    print("Use the following keys to control the LED:")
    print("  w: Turn LED ON (Forward)")
    print("  s: Turn LED ON (Backward)")
    print("  a: Turn LED ON (Left)")
    print("  d: Turn LED ON (Right)")
    print("  q: Turn LED OFF (Stop)")
    print("  l: Toggle LED ON/OFF")
    print("  Ctrl+C: Quit")
    
    # Initialize LED state
    led_state = False
    GPIO.output(led_pin, GPIO.LOW)
    
    try:
        while True:
            # Get key press
            print(f"\rLED state: {'ON' if led_state else 'OFF'}", end='')
            key = get_key()
            
            # Check for Ctrl+C
            if key == '\x03':
                break
                
            # Process key press
            if key in key_mapping:
                if key == 'l':
                    # Toggle LED
                    led_state = not led_state
                else:
                    # Set LED state based on key
                    led_state = key_mapping[key]
                
                # Update LED
                GPIO.output(led_pin, GPIO.HIGH if led_state else GPIO.LOW)
                print(f"\rLED state: {'ON' if led_state else 'OFF'}", end='')
                
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        GPIO.output(led_pin, GPIO.LOW)
        GPIO.cleanup()
        print("\nExiting...")

if __name__ == '__main__':
    main() 