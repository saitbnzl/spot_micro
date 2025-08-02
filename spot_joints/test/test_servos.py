import sys
import termios
import tty
import time
from adafruit_servokit import ServoKit
from motors import Motor, Leg, Dog

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
