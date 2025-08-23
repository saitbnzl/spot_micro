#!/usr/bin/env python3

import sys
import termios
import tty
import time
import signal
from adafruit_servokit import ServoKit
from spot_joints.spot_joints.motors import Motor, Leg, Dog

# Servo range
Motor.MIN = 0
Motor.MAX = 180

# Init PCA9685 (16 channels)
kit = ServoKit(channels=16)

# Setup motors (edit these for your Spot Micro)
servos = {
    "1": Motor(kit.servo[0], direction=1, origin=90),
    "2": Motor(kit.servo[1], direction=1, origin=90),
    "3": Motor(kit.servo[2], direction=1, origin=90),
    "4": Motor(kit.servo[4], direction=1, origin=90),
    "5": Motor(kit.servo[5], direction=1, origin=90),
    "6": Motor(kit.servo[6], direction=1, origin=90),
    "7": Motor(kit.servo[8], direction=1, origin=90),
    "8": Motor(kit.servo[9], direction=1, origin=90),
    "9": Motor(kit.servo[15], direction=1, origin=90),
}

selected = None
old_term_settings = termios.tcgetattr(sys.stdin)

def get_key():
    """Capture single keypress from terminal."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term_settings)
    return key

def cleanup_and_exit(signum=None, frame=None):
    """Reset terminal, center servos, exit cleanly."""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term_settings)
    print("\n[!] Exiting. Centering servos...")
    for key, motor in servos.items():
        motor.set_angle(0)  # Or set_angle(90) depending on your default
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup_and_exit)

print("🎮 Servo Test Ready")
print("  - Press 1–6 to select servo")
print("  - Use '+' or '-' to adjust")
print("  - Press 'q' to quit\n")

try:
    while True:
        key = get_key()

        if key in servos:
            selected = key
            print(f"> Servo {selected} selected")

        elif key == "+" and selected:
            servos[selected].adjust_angle(+5)
            print(f"  Servo {selected} +5°")

        elif key == "-" and selected:
            servos[selected].adjust_angle(-5)
            print(f"  Servo {selected} -5°")

        elif key == "q":
            cleanup_and_exit()

        else:
            print("  Invalid. Select [1–6], then use + / - to move.")

        time.sleep(0.05)

except Exception as e:
    print(f"[ERROR] {e}")
    cleanup_and_exit()
import sys
import termios
import tty
import time
from adafruit_servokit import ServoKit
from spot_joints.motors import Motor, Leg, Dog

# Setup servo kit (assuming 16-channel PCA9685)
kit = ServoKit(channels=16)

# Create Motor instances
# Example: Motor(channel_obj, direction, origin)
# Adjust channel numbers and tuning as per your setup
m1 = Motor(kit.servo[0], direction=1, origin=90)
m2 = Motor(kit.servo[1], direction=1, origin=90)
m3 = Motor(kit.servo[2], direction=1, origin=90)
m4 = Motor(kit.servo[3], direction=1, origin=90)
m5 = Motor(kit.servo[4], direction=1, origin=90)
m6 = Motor(kit.servo[5], direction=1, origin=90)

servos = {
    "1": m1,
    "2": m2,
    "3": m3,
    "4": m4,
    "5": m5,
    "6": m6,
}

print("Servo Test Ready.")
print("Press 1–6 to adjust individual servos. Use '+' or '-' to move them.")
print("Press 'q' to quit.\n")

def get_key():
    """Capture a single keypress from terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

selected = None

try:
    while True:
        key = get_key()
        if key in servos:
            selected = key
            print(f"> Selected servo {key}")
        elif key == "+" and selected:
            servos[selected].adjust_angle(5)
            print(f"  Servo {selected} +5°")
        elif key == "-" and selected:
            servos[selected].adjust_angle(-5)
            print(f"  Servo {selected} -5°")
        elif key == "q":
            print("Exiting test.")
            break
        else:
            print("Invalid input. Use 1–6 to select, + / - to move.")
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nInterrupted. Exiting.")
