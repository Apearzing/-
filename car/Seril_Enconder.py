#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Encoder Odometry Node
---------------------
Reads wheel encoder data from serial,
computes differential drive odometry,
publishes /odom and TF: odom -> base_link
"""

import sys
import signal
import os
import time
import math

import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

# ================== Robot Physical Parameters ==================
COUNTS_PER_REV = 1500        # Encoder counts per wheel revolution
WHEEL_RADIUS = 0.038         # Wheel radius in meters
WHEEL_BASE = 0.3             # Distance between wheels in meters
MAX_U16 = 65535              # Maximum uint16 encoder value

# Distance moved per encoder pulse
METERS_PER_PULSE = (2.0 * math.pi * WHEEL_RADIUS) / COUNTS_PER_REV

# ================== Global Variables ==================
ser = None
recv_buffer = ""

robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0

last_cnt1 = 0
last_cnt2 = 0
first_frame = True


# ================== Signal Handler ==================
def signal_handler(signum, frame):
    """Handle Ctrl+C and safely shut down."""
    global ser
    print("\nCtrl+C received, shutting down...")
    if ser and ser.is_open:
        ser.close()
    rclpy.shutdown()
    sys.exit(0)


# ================== Math Utilities ==================
def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle (radians) to quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


def normalize_angle(angle: float) -> float:
    """Normalize angle to range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def calc_delta(raw_now: int, raw_last: int) -> int:
    """Handle uint16 encoder overflow."""
    delta = raw_now - raw_last
    if delta > MAX_U16 // 2:
        delta -= (MAX_U16 + 1)
    elif delta < -MAX_U16 // 2:
        delta += (MAX_U16 + 1)
    return delta


# ================== ROS Node ==================
class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__("encoder_to_odom_node")

        # ROS publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Encoder deltas
        self.latest_d1 = 0
        self.latest_d2 = 0

        # Timing
        self.last_time = self.get_clock().now()

        # Timer: publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_publish)

        self.get_logger().info("Encoder Odometry Node started.")

    def update_odom(self, delta1: int, delta2: int):
        """Update robot pose based on encoder deltas."""
        global robot_x, robot_y, robot_yaw

        dl = delta1 * METERS_PER_PULSE
        dr = delta2 * METERS_PER_PULSE

        # Clamp large jumps caused by noise
        max_step = 0.1
        dl = max(min(dl, max_step), -max_step)
        dr = max(min(dr, max_step), -max_step)

        dc = (dl + dr) / 2.0
        dyaw = (dr - dl) / WHEEL_BASE

        robot_x += dc * math.cos(robot_yaw)
        robot_y += dc * math.sin(robot_yaw)
        robot_yaw += dyaw
        robot_yaw = normalize_angle(robot_yaw)

        self.latest_d1 = delta1
        self.latest_d2 = delta2

    def timer_publish(self):
        """
        Publish /odom and TF transform (odom -> base_link).
        NOTE: TF timestamp must be current time, not historical.
        """
        now = self.get_clock().now()
        stamp = now.to_msg()

        # Time delta for velocity calculation
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt < 0.01:
            dt = 0.1

        # Compute velocities
        dl = self.latest_d1 * METERS_PER_PULSE
        dr = self.latest_d2 * METERS_PER_PULSE
        dc = (dl + dr) / 2.0
        dyaw = (dr - dl) / WHEEL_BASE

        vx = dc / dt
        vyaw = dyaw / dt

        # ========== Odometry Message ==========
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = robot_x
        odom_msg.pose.pose.position.y = robot_y
        odom_msg.pose.pose.orientation = yaw_to_quaternion(robot_yaw)

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vyaw

        # Covariance (must be 36 floats)
        cov = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        odom_msg.pose.covariance = cov
        odom_msg.twist.covariance = cov

        self.odom_pub.publish(odom_msg)

        # ========== TF: odom -> base_link ==========
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = robot_x
        tf_msg.transform.translation.y = robot_y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)

        self.get_logger().info(
            f"X:{robot_x:.2f} Y:{robot_y:.2f} Yaw:{math.degrees(robot_yaw):.1f}"
        )


# ================== Main ==================
def main():
    global ser, recv_buffer, last_cnt1, last_cnt2, first_frame

    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init()
    node = EncoderOdomNode()

    print("\nAvailable serial ports:")
    os.system("ls /dev/tty[a-zA-Z]*")

    uart_dev = input("Enter serial port: ")
    baudrate = input("Enter baudrate: ")

    try:
        ser = serial.Serial(uart_dev, int(baudrate), timeout=0)
        ser.flushInput()
        ser.flushOutput()
        print("Serial port opened.")
    except Exception as e:
        print("Failed to open serial:", e)
        rclpy.shutdown()
        return

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)

        raw = ser.read_all()
        if raw:
            recv_buffer += raw.decode("utf-8", errors="ignore")

        if "\n" in recv_buffer:
            lines = recv_buffer.split("\n")
            recv_buffer = ""

            for line in lines:
                line = line.strip()
                if not line:
                    continue

                if "CNT1:" in line and "CNT2:" in line:
                    try:
                        p1, p2 = line.split("  ")
                        cnt1 = int(p1.split(":")[1])
                        cnt2 = int(p2.split(":")[1])

                        if first_frame:
                            last_cnt1 = cnt1
                            last_cnt2 = cnt2
                            first_frame = False
                            continue

                        d1 = calc_delta(cnt1, last_cnt1)
                        d2 = calc_delta(cnt2, last_cnt2)

                        last_cnt1 = cnt1
                        last_cnt2 = cnt2

                        node.update_odom(d1, d2)

                    except Exception as e:
                        print("Parse error:", e)

        time.sleep(0.001)

    if ser and ser.is_open:
        ser.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
