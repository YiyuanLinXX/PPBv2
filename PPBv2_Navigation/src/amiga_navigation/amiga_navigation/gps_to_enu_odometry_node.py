#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
gps_to_enu_odometry_node.py

Subscribe to /gps/fix, /gps/datum, and /imu (/imu/filtered if needed).
Convert GPS coordinates to local ENU coordinates using pyproj.
Extract yaw from IMU orientation.
Publish combined x, y, yaw as nav_msgs/Odometry to /robot/odom.

Usage:
    ros2 run <your_package_name> gps_to_enu_odometry_node
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from pyproj import Transformer

class GpsToEnuOdometryNode(Node):
    def __init__(self):
        super().__init__('gps_to_enu_odometry_node')

        # Internal variables
        self.datum_received = False
        self.datum_lat = 0.0
        self.datum_lon = 0.0
        self.datum_alt = 0.0

        self.transformer = None  # pyproj Transformer, initialized when datum is received

        self.latest_yaw = 0.0  # yaw in radians

        # Publisher: Odometry
        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', 10)
        self.first_log_done = False

        # Subscribers
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Float64MultiArray, '/gps/datum', self.datum_callback, 10)
        # self.create_subscription(Imu, '/imu/filtered', self.imu_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        # there is a built-in imu filter in Reach RS3 and it is very precise, so we can use the raw imu data directly

        self.get_logger().info("gps_to_enu_odometry_node (pyproj version) started.")

    def datum_callback(self, msg):
        """Callback for /gps/datum"""
        self.datum_lat = msg.data[0]
        self.datum_lon = msg.data[1]
        self.datum_alt = msg.data[2]
        self.datum_received = True

        # Create pyproj Transformer for local ENU projection
        self.transformer = Transformer.from_crs(
            "epsg:4326",  # WGS84
            f"+proj=tmerc +lat_0={self.datum_lat} +lon_0={self.datum_lon} +k=1 +x_0=0 +y_0=0 +datum=WGS84",
            always_xy=True
        )

        self.get_logger().debug(
            f"Received datum and created Transformer: lat={self.datum_lat:.8f}, lon={self.datum_lon:.8f}, alt={self.datum_alt:.2f}"
        )

    def imu_callback(self, msg):
        """Callback for /imu/filtered"""
        # Extract yaw from quaternion
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.latest_yaw = yaw

    def gps_callback(self, msg):
        """Callback for /gps/fix"""
        if not self.datum_received or self.transformer is None:
            self.get_logger().warn("No datum/Transformer yet, cannot convert GPS to ENU.")
            return

        lon = msg.longitude
        lat = msg.latitude

        # Use pyproj Transformer to convert to ENU (east, north)
        east, north = self.transformer.transform(lon, lat)

        # Prepare Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'map'  # Recommended: 'map'
        odom_msg.child_frame_id = 'base_link'  # Robot base frame

        # Set position
        odom_msg.pose.pose.position.x = east
        odom_msg.pose.pose.position.y = north
        odom_msg.pose.pose.position.z = msg.altitude - self.datum_alt

        # Set orientation (copy latest IMU yaw as quaternion)
        q = self.yaw_to_quaternion(self.latest_yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Optionally set covariances (example values)
        odom_msg.pose.covariance = [
            0.0004, 0.0,    0.0,    0.0,    0.0,    0.0,
            0.0,    0.0004, 0.0,    0.0,    0.0,    0.0,
            0.0,    0.0,    0.0025, 0.0,    0.0,    0.0,
            0.0,    0.0,    0.0,    0.1,    0.0,    0.0,
            0.0,    0.0,    0.0,    0.0,    0.1,    0.0,
            0.0,    0.0,    0.0,    0.0,    0.0,    0.0003
        ]


        # Set velocity to zero (you can update this later if needed)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Publish Odometry
        self.odom_pub.publish(odom_msg)

        # Log only once
        if not self.first_log_done:
            self.get_logger().info(
                f"GPS to ENU conversion succeeded."
            )
            self.first_log_done = True

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle (radians) to quaternion"""
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = GpsToEnuOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
