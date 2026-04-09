#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from pyproj import Transformer

class GpsToEnuOdometryNode(Node):
    def __init__(self):
        super().__init__('gnss2enu_odometry')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('max_imu_age_sec', 0.25)
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.max_imu_age_sec = self.get_parameter('max_imu_age_sec').get_parameter_value().double_value

        # Datum and transformer
        self.datum_received = False
        self.datum_lat = 0.0
        self.datum_lon = 0.0
        self.datum_alt = 0.0
        self.transformer = None

        self.first_log_done = False
        self.latest_imu_msg = None
        self.last_warn_missing_imu = None
        self.last_warn_missing_datum = None

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', 10)

        # Datum subscriber
        datum_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(Float64MultiArray, '/gps/datum', self.datum_callback, datum_qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f"gnss2enu_odometry started. GNSS fixes will use latest IMU from {self.imu_topic}."
        )

    def datum_callback(self, msg):
        self.datum_lat = msg.data[0]
        self.datum_lon = msg.data[1]
        self.datum_alt = msg.data[2]
        self.datum_received = True

        self.transformer = Transformer.from_crs(
            "epsg:4326",
            f"+proj=tmerc +lat_0={self.datum_lat} +lon_0={self.datum_lon} +k=1 +x_0=0 +y_0=0 +datum=WGS84",
            always_xy=True
        )

        self.get_logger().debug(
            f"Received datum: lat={self.datum_lat:.8f}, lon={self.datum_lon:.8f}, alt={self.datum_alt:.2f}"
        )

    def imu_callback(self, imu_msg):
        self.latest_imu_msg = imu_msg

    def gps_callback(self, gps_msg):
        if not self.datum_received or self.transformer is None:
            self._warn_throttled('missing_datum', "Datum not received. Cannot convert GPS to ENU yet.")
            return

        if self.latest_imu_msg is None:
            self._warn_throttled('missing_imu', f"No IMU available on {self.imu_topic}; skipping odometry publish.")
            return

        imu_msg = self.latest_imu_msg
        if self._get_age_sec(gps_msg.header.stamp, imu_msg.header.stamp) > self.max_imu_age_sec:
            self._warn_throttled(
                'stale_imu',
                f"Latest IMU on {self.imu_topic} is older than {self.max_imu_age_sec:.2f}s; skipping odometry publish."
            )
            return

        # === Convert GPS to ENU ===
        lon = gps_msg.longitude
        lat = gps_msg.latitude
        alt = gps_msg.altitude

        east, north = self.transformer.transform(lon, lat)
        z = alt - self.datum_alt

        # === Extract yaw from IMU ===
        q = imu_msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # === Construct Odometry ===
        odom_msg = Odometry()
        odom_msg.header.stamp = gps_msg.header.stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = east
        odom_msg.pose.pose.position.y = north
        odom_msg.pose.pose.position.z = z

        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Pose covariance (optional)
        odom_msg.pose.covariance = [
            0.0004, 0.0,    0.0,    0.0,    0.0,    0.0,
            0.0,    0.0004, 0.0,    0.0,    0.0,    0.0,
            0.0,    0.0,    0.0025, 0.0,    0.0,    0.0,
            0.0,    0.0,    0.0,    0.1,    0.0,    0.0,
            0.0,    0.0,    0.0,    0.0,    0.1,    0.0,
            0.0,    0.0,    0.0,    0.0,    0.0,    0.0003
        ]

        self.odom_pub.publish(odom_msg)

        if not self.first_log_done:
            self.get_logger().info("GPS + latest IMU -> ENU odometry publishing started.")
            self.first_log_done = True

    def _get_age_sec(self, newer_stamp, older_stamp):
        newer = Time.from_msg(newer_stamp)
        older = Time.from_msg(older_stamp)
        return abs((newer - older).nanoseconds) / 1e9

    def _warn_throttled(self, key, message):
        now = self.get_clock().now()
        last_warn = getattr(self, f'last_warn_{key}', None)
        if last_warn is None or (now - last_warn).nanoseconds > int(2e9):
            self.get_logger().warn(message)
            setattr(self, f'last_warn_{key}', now)

    def yaw_to_quaternion(self, yaw):
        """Convert yaw to quaternion [x, y, z, w]"""
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
