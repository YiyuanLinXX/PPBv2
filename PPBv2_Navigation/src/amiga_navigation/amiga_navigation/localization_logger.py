#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Log `/robot/odom` and `/imu` to CSV for offline localization analysis."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import csv
import math
import tf_transformations
import datetime

def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw in degrees."""
    (_, _, yaw) = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
    return math.degrees(yaw)

class OdometryLoggerNode(Node):
    def __init__(self):
        super().__init__('localization_logger')
        self.get_logger().info("Starting localization logger (robot_odom + imu)")

        # Create timestamped filename
        now_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'localization_log_{now_str}.csv'
        self.get_logger().info(f"Logging to file: {filename}")

        # Open CSV file
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'source', 'x', 'y', 'z', 'yaw_deg'])

        # Subscribers
        self.create_subscription(Odometry, '/robot/odom', self.robot_odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

    def log_odometry(self, source, x, y, z, yaw_deg):
        timestamp = self.get_clock().now().seconds_nanoseconds()[0] + \
                    self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        self.csv_writer.writerow([timestamp, source, x, y, z, yaw_deg])
        self.csv_file.flush()

    def robot_odom_callback(self, msg):
        pose = msg.pose.pose
        yaw_deg = quaternion_to_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        self.log_odometry('robot_odom', pose.position.x, pose.position.y, pose.position.z, yaw_deg)

    def imu_callback(self, msg):
        yaw_deg = quaternion_to_yaw(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )
        self.log_odometry('imu', 0.0, 0.0, 0.0, yaw_deg)

    def destroy_node(self):
        self.get_logger().info("Shutting down localization logger")
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdometryLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
