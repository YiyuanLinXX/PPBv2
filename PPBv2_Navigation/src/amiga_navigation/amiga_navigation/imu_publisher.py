#!/usr/bin/env python3
"""ROS 2 publisher for the HWT905 IMU."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler

from amiga_navigation.utils.hwt905_driver import HWT905


DEFAULT_IMU_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
DEFAULT_BAUDRATE = 9600


def normalize_radians(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ImuOrientationPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameter('imu_port', DEFAULT_IMU_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('serial_timeout_sec', 0.02)
        self.declare_parameter('poll_period_sec', 0.01)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('yaw_offset_deg', 90.0)
        self.declare_parameter('yaw_multiplier', 1.0)

        imu_port = self.get_parameter('imu_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        serial_timeout = self.get_parameter('serial_timeout_sec').get_parameter_value().double_value
        poll_period = self.get_parameter('poll_period_sec').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.yaw_offset_deg = self.get_parameter('yaw_offset_deg').get_parameter_value().double_value
        self.yaw_multiplier = self.get_parameter('yaw_multiplier').get_parameter_value().double_value

        self.publisher_imu = self.create_publisher(Imu, '/imu', 10)
        self.publisher_imu_data = self.create_publisher(Imu, '/imu/data', 10)

        self.imu = HWT905(imu_port, baudrate=baudrate, timeout=serial_timeout)
        self.last_publish_time = None
        self.create_timer(poll_period, self.timer_callback)

        self.get_logger().info(f'HWT905 IMU connected on {imu_port} @ {baudrate}bps')

    def timer_callback(self):
        try:
            updated = self.imu.read_available()
        except Exception as exc:
            self.get_logger().warning(f'Failed to read HWT905 serial data: {exc}')
            return

        if not updated or not self.imu.state.ready:
            return

        state = self.imu.state
        roll = math.radians(state.roll_deg)
        pitch = math.radians(state.pitch_deg)
        yaw = normalize_radians(math.radians(self.yaw_multiplier * state.yaw_deg + self.yaw_offset_deg))
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        msg.orientation_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.05
        ]

        msg.angular_velocity.x = math.radians(state.wx)
        msg.angular_velocity.y = math.radians(state.wy)
        msg.angular_velocity.z = math.radians(state.wz)
        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]

        msg.linear_acceleration.x = state.ax
        msg.linear_acceleration.y = state.ay
        msg.linear_acceleration.z = state.az
        msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]

        self.publisher_imu.publish(msg)
        self.publisher_imu_data.publish(msg)
        self.last_publish_time = self.get_clock().now()

    def destroy_node(self):
        try:
            self.imu.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuOrientationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
