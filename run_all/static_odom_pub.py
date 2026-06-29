#!/usr/bin/env python3
"""
Static Odom Publisher
---------------------
Publishes a static odom->base_link TF and /odom topic.
Use when encoder hardware is not connected but you still need
slam_toolbox to work (it REQUIRES the odom frame).
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


class StaticOdomNode(Node):
    def __init__(self):
        super().__init__("static_odom_pub")

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.publish)  # 20 Hz

        self.get_logger().info("Static Odom Publisher started (odom=identity)")

    def publish(self):
        now = self.get_clock().now()
        stamp = now.to_msg()

        # Identity quaternion
        q = Quaternion()
        q.w = 1.0
        q.x = q.y = q.z = 0.0

        # --- Odometry ---
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0

        # Covariance 36 floats
        cov = [0.01] * 36
        odom.pose.covariance = cov
        odom.twist.covariance = cov
        self.odom_pub.publish(odom)

        # --- TF: odom -> base_link ---
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = StaticOdomNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
