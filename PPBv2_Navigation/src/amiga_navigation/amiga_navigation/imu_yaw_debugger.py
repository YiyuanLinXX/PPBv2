#!/usr/bin/env python3
"""Publish human-readable yaw angles derived from /imu and /imu/data."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String
import tf_transformations


def quaternion_to_yaw_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw angle in signed degrees [-180, 180)."""
    (_, _, yaw) = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
    return math.degrees(yaw)


def wrap_360(angle_deg: float) -> float:
    """Wrap a degree angle to [0, 360)."""
    return angle_deg % 360.0


class ImuYawDebuggerNode(Node):
    def __init__(self):
        super().__init__('imu_yaw_debugger')

        self.imu_signed_pub = self.create_publisher(Float64, '/imu/yaw_deg', 10)
        self.imu_compass_pub = self.create_publisher(Float64, '/imu/yaw_deg_360', 10)
        self.imu_text_pub = self.create_publisher(String, '/imu/yaw_text', 10)

        self.imu_data_signed_pub = self.create_publisher(Float64, '/imu/data/yaw_deg', 10)
        self.imu_data_compass_pub = self.create_publisher(Float64, '/imu/data/yaw_deg_360', 10)
        self.imu_data_text_pub = self.create_publisher(String, '/imu/data/yaw_text', 10)

        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_data_callback, 10)

        self.get_logger().info(
            'IMU yaw debugger started. Publishing readable yaw on '
            '/imu/yaw_deg, /imu/yaw_deg_360, /imu/yaw_text, '
            '/imu/data/yaw_deg, /imu/data/yaw_deg_360, /imu/data/yaw_text. '
            'Raw/conversion debug topics from imu_publisher: '
            '/imu/raw_yaw_deg, /imu/ros_yaw_deg'
        )

    def _publish_yaw(self, prefix: str, msg: Imu) -> None:
        yaw_deg = quaternion_to_yaw_deg(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        yaw_deg_360 = wrap_360(yaw_deg)

        text_msg = String()
        text_msg.data = f'{prefix}: yaw_deg={yaw_deg:.2f}, yaw_deg_360={yaw_deg_360:.2f}'

        signed_msg = Float64()
        signed_msg.data = yaw_deg

        compass_msg = Float64()
        compass_msg.data = yaw_deg_360

        if prefix == '/imu':
            self.imu_signed_pub.publish(signed_msg)
            self.imu_compass_pub.publish(compass_msg)
            self.imu_text_pub.publish(text_msg)
        else:
            self.imu_data_signed_pub.publish(signed_msg)
            self.imu_data_compass_pub.publish(compass_msg)
            self.imu_data_text_pub.publish(text_msg)

    def imu_callback(self, msg: Imu) -> None:
        self._publish_yaw('/imu', msg)

    def imu_data_callback(self, msg: Imu) -> None:
        self._publish_yaw('/imu/data', msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuYawDebuggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
