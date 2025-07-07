#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rtk_monitor_node.py

This node monitors the RTK status flag published on /gps/rtk_status_flag.
If the status indicates "not FIX" (True), it will publish zero velocity to /cmd_vel_stop.
If the status is FIX (False), it will stop publishing so that twist_mux can switch to other sources.

Author: Yiyuan Lin, yl3663@cornell.edu
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class RTKStatusMonitor(Node):
    def __init__(self):
        super().__init__('rtk_status_monitor')

        self.sub = self.create_subscription(
            Bool, '/gps/rtk_status_flag', self.status_callback, 10
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel_stop', 10)

        self.active = False  # whether we are currently publishing stop
        self.get_logger().info("RTK Monitor Node started. Waiting for RTK status updates...")

    def status_callback(self, msg: Bool):
        if msg.data:  # not FIX
            if not self.active:
                self.get_logger().warn("RTK not FIX — publishing /cmd_vel_stop")
                self.get_logger().warn("RTK not FIX — the robot will continue running when it gets FIX again")
            self.publish_stop()
            self.active = True
        else:
            if self.active:
                self.get_logger().info("RTK FIX restored — stop publishing /cmd_vel_stop")
                self.get_logger().info("RTK FIX restored — continue robot navigation")
            self.active = False

    def publish_stop(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RTKStatusMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()