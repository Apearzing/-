#!/usr/bin/env python3

import sys
import signal
import os
import time

# Import serial communication library
import serial
import serial.tools.list_ports

# Handle Ctrl+C exit signal
def signal_handler(signal, frame):
    print("\nProgram exit by user")
    sys.exit(0)

def serialParse():
    # List all available serial ports
    print("List of enabled UART:")
    os.system('ls /dev/tty[a-zA-Z]*')

    # Get serial device name from input
    uart_dev = input("Please enter serial port device name:")
    # Get baud rate from input
    baudrate = input("Please enter baud rate(9600,19200,38400,57600,115200,921600):")

    # Open serial port
    try:
        ser = serial.Serial(uart_dev, int(baudrate), timeout=1)
    except Exception as e:
        print("Open serial port failed! Error:", e)
        return -1

    print("Serial port opened successfully: ", ser)
    print("Start parsing serial data! Press CTRL+C to exit")

    # Buffer for receiving serial data
    recv_buffer = ""

    while True:
        # Read one byte each time
        raw_data = ser.read(1)
        if not raw_data:
            continue

        # Append data to buffer and decode
        recv_buffer += raw_data.decode("UTF-8", errors="ignore")

        # Check if received complete frame: ends with \r\n
        if recv_buffer.endswith("\r\n"):
            frame = recv_buffer.strip()
            recv_buffer = ""  # Clear buffer for next frame

            # Parse format: CNT1:%d  CNT2:%d
            try:
                if "CNT1:" in frame and "CNT2:" in frame:
                    # Split and extract numbers
                    part1, part2 = frame.split("  ")
                    cnt1 = int(part1.split(":")[1])
                    cnt2 = int(part2.split(":")[1])
                    print(f"Parsed Data -> CNT1: {cnt1}, CNT2: {cnt2}")
            except (ValueError, IndexError) as parse_err:
                print(f"Invalid data frame: {frame}, Parse error: {parse_err}")

        time.sleep(0.01)

    # Close serial port (unreachable here, for standard procedure)
    ser.close()
    return 0


if __name__ == '__main__':
    # Register Ctrl+C signal handler
    signal.signal(signal.SIGINT, signal_handler)
    ret = serialParse()
    if ret != 0:
        print("Serial parse test failed!")
    else:
        print("Serial parse test success!")
