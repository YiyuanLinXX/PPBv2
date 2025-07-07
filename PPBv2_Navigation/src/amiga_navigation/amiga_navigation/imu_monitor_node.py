#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_monitor_node.py

Monitors /imu/state and /imu/notification.
If tilt compensation is not active (state != 30) or warning exists (notification != 0),
publish zero velocity to /cmd_vel_stop.

Prints both raw value and human-readable interpretation.

Author: Yiyuan Lin
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist


class IMUStatusMonitor(Node):
    def __init__(self):
        super().__init__('imu_status_monitor')

        self.sub_state = self.create_subscription(Int32, '/imu/state', self.state_callback, 10)
        self.sub_notify = self.create_subscription(Int32, '/imu/notification', self.notification_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_stop', 10)

        self.state = -1
        self.notification = -1
        self.active = False

        self.state_map = {
            -1: "unknown",
            0: "fatal error",
            10: "setup",
            20: "alignment",
            30: "compensating"
        }

        self.notification_map = {
            -1: "unknown",
            0: "none",
            10: "fast motion",
            20: "bad GNSS",
            30: "filter fault"
        }

        self.get_logger().info("IMU Monitor Node started. Monitoring /imu/state and /imu/notification...")

    def state_callback(self, msg: Int32):
        self.state = msg.data
        self.evaluate()

    def notification_callback(self, msg: Int32):
        self.notification = msg.data
        self.evaluate()

    def evaluate(self):
        state_str = self.state_map.get(self.state, "unrecognized")
        notify_str = self.notification_map.get(self.notification, "unrecognized")

        if self.state != 30 or self.notification != 0:
            if not self.active:
                self.get_logger().warn(
                    f"IMU NOT VALID — state={self.state} ({state_str}), "
                    f"notification={self.notification} ({notify_str}) — STOPPING, "
                    f"move the robot to realign the IMU"
                )
            self.publish_stop()
            self.active = True
        else:
            if self.active:
                self.get_logger().info("IMU OK — state=30 (compensating), notification=0 (none) — Resuming")
            self.active = False

    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUStatusMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
