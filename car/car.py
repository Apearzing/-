#!/usr/bin/env python3

import sys
import signal
import Hobot.GPIO as GPIO
import time

def signal_handler(signal, frame):
	GPIO.cleanup()
	sys.exit(0)

left_car_wheels_1 = 11
left_car_wheels_2 = 13
right_car_wheels_1 = 15
right_car_wheels_2 = 16
all_whells_pwm = 32

p = None

def init_IO():
    global p
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(left_car_wheels_1,GPIO.OUT)
    GPIO.setup(left_car_wheels_2,GPIO.OUT)
    GPIO.setup(right_car_wheels_1,GPIO.OUT)
    GPIO.setup(right_car_wheels_2,GPIO.OUT)
    p = GPIO.PWM(all_whells_pwm, 48000)
    p.ChangeDutyCycle(25)
    p.start(25)

def set_speed(val):
    if 0<= val <= 100:
        p.ChangeDutyCycle(val)

def car_go():
    GPIO.output(left_car_wheels_1,GPIO.HIGH)
    GPIO.output(right_car_wheels_1,GPIO.HIGH)
    GPIO.output(left_car_wheels_2,GPIO.LOW)
    GPIO.output(right_car_wheels_2,GPIO.LOW)

def car_left():
    GPIO.output(left_car_wheels_1,GPIO.LOW)
    GPIO.output(right_car_wheels_1,GPIO.HIGH)
    GPIO.output(left_car_wheels_2,GPIO.HIGH)
    GPIO.output(right_car_wheels_2,GPIO.LOW)

def car_right():
    GPIO.output(left_car_wheels_1,GPIO.HIGH)
    GPIO.output(right_car_wheels_1,GPIO.LOW)
    GPIO.output(left_car_wheels_2,GPIO.LOW)
    GPIO.output(right_car_wheels_2,GPIO.HIGH)

def car_stop():
    GPIO.output(left_car_wheels_1, GPIO.LOW)
    GPIO.output(right_car_wheels_1, GPIO.LOW)
    GPIO.output(left_car_wheels_2, GPIO.LOW)
    GPIO.output(right_car_wheels_2, GPIO.LOW)

GPIO.setwarnings(False)

def main():
    init_IO()
    car_go()
    car_stop();
    set_speed(25)
  
    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
			
            time.sleep(1)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    main()
