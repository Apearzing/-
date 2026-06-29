#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO
import time
import tty
import termios

# ---------------------- GPIO Pin Definition ----------------------
left_car_wheels_1 = 11
left_car_wheels_2 = 13
right_car_wheels_1 = 15
right_car_wheels_2 = 16
all_wheels_pwm = 32

p = None
# Global speed variable
current_duty = 25

# ---------------------- Signal Handler ----------------------
def signal_handler(signum, frame):
    """Handle Ctrl+C signal, clean GPIO and exit"""
    GPIO.cleanup()
    print("\nExit program, GPIO cleaned up.")
    sys.exit(0)

# ---------------------- GPIO Initialize ----------------------
def init_IO():
    """Initialize all GPIO pins and PWM"""
    global p
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    GPIO.setup(left_car_wheels_1, GPIO.OUT)
    GPIO.setup(left_car_wheels_2, GPIO.OUT)
    GPIO.setup(right_car_wheels_1, GPIO.OUT)
    GPIO.setup(right_car_wheels_2, GPIO.OUT)

    # PWM init: frequency 48000, initial duty 25
    p = GPIO.PWM(all_wheels_pwm, 48000)
    p.ChangeDutyCycle(25)
    p.start(25)


# ---------------------- Speed Control ----------------------
def set_speed(val):
    """Set PWM duty cycle, limit 0 ~ 100"""
    global current_duty
    if 0 <= val <= 100:
        current_duty = val
        p.ChangeDutyCycle(current_duty)

# ---------------------- Car Motion Functions ----------------------
def car_forward():
    """W: Move forward"""
    GPIO.output(left_car_wheels_1, GPIO.HIGH)
    GPIO.output(right_car_wheels_1, GPIO.HIGH)
    GPIO.output(left_car_wheels_2, GPIO.LOW)
    GPIO.output(right_car_wheels_2, GPIO.LOW)

def car_backward():
    """S: Move backward"""
    GPIO.output(left_car_wheels_1, GPIO.LOW)
    GPIO.output(right_car_wheels_1, GPIO.LOW)
    GPIO.output(left_car_wheels_2, GPIO.HIGH)
    GPIO.output(right_car_wheels_2, GPIO.HIGH)

def car_rotate_left():
    """A: Rotate left in place"""
    GPIO.output(left_car_wheels_1, GPIO.LOW)
    GPIO.output(right_car_wheels_1, GPIO.HIGH)
    GPIO.output(left_car_wheels_2, GPIO.HIGH)
    GPIO.output(right_car_wheels_2, GPIO.LOW)

def car_rotate_right():
    """D: Rotate right in place"""
    GPIO.output(left_car_wheels_1, GPIO.HIGH)
    GPIO.output(right_car_wheels_1, GPIO.LOW)
    GPIO.output(left_car_wheels_2, GPIO.LOW)
    GPIO.output(right_car_wheels_2, GPIO.HIGH)

def car_stop_all():
    """K: Stop car & clear all pin level"""
    GPIO.output(left_car_wheels_1, GPIO.LOW)
    GPIO.output(right_car_wheels_1, GPIO.LOW)
    GPIO.output(left_car_wheels_2, GPIO.LOW)
    GPIO.output(right_car_wheels_2, GPIO.LOW)

# ---------------------- Keyboard Read Function ----------------------
def get_key():
    """Read single key without Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

# ---------------------- Main Function ----------------------
def main():
    init_IO()
    car_stop_all()
    set_speed(25)

    print("===== Car Keyboard Control =====")
    print("W : Forward")
    print("S : Backward")
    print("A : Rotate Left")
    print("D : Rotate Right")
    print("K : Stop & Clear all pins")
    print("U : Increase Speed (+5)")
    print("J : Decrease Speed (-5)")
    print("Ctrl + C : Exit\n")

    try:
        while True:
            key = get_key()
            # Catch Ctrl+C
            if key == '\x03':
                raise KeyboardInterrupt

            if key == 'w' or key == 'W':
                car_forward()
                print("Command: Forward")
            elif key == 's' or key == 'S':
                car_backward()
                print("Command: Backward")
            elif key == 'a' or key == 'A':
                car_rotate_left()
                print("Command: Rotate Left")
            elif key == 'd' or key == 'D':
                car_rotate_right()
                print("Command: Rotate Right")
            elif key == 'k' or key == 'K':
                car_stop_all()
                print("Command: Stop & Clear Pins")
            elif key == 'u' or key == 'U':
                set_speed(current_duty + 5)
                print(f"Speed Up, Current Duty: {current_duty}")
            elif key == 'j' or key == 'J':
                set_speed(current_duty - 5)
                print(f"Speed Down, Current Duty: {current_duty}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nReceived Ctrl+C, exiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        GPIO.cleanup()
        print("GPIO released successfully.")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()
