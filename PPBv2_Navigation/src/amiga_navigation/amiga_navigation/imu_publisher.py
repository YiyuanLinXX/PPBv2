#!/usr/bin/env python3
"""ROS 2 publisher for Witmotion IMU orientation messages."""

import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf_transformations import quaternion_from_euler


DEFAULT_IMU_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
DEFAULT_BAUDRATE = 9600


def normalize_radians(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ImuOrientationPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameter('imu_port', DEFAULT_IMU_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('yaw_offset_deg', 90.0)
        self.declare_parameter('yaw_multiplier', 1.0)
        self.declare_parameter('desired_update_rate_hz', 20)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.yaw_offset_deg = self.get_parameter('yaw_offset_deg').get_parameter_value().double_value
        self.yaw_multiplier = self.get_parameter('yaw_multiplier').get_parameter_value().double_value
        imu_port = self.get_parameter('imu_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        desired_update_rate_hz = self.get_parameter('desired_update_rate_hz').get_parameter_value().integer_value

        self.publisher_imu = self.create_publisher(Imu, '/imu', 10)
        self.publisher_imu_data = self.create_publisher(Imu, '/imu/data', 10)
        self.publisher_raw_yaw_deg = self.create_publisher(Float64, '/imu/raw_yaw_deg', 10)
        self.publisher_ros_yaw_deg = self.create_publisher(Float64, '/imu/ros_yaw_deg', 10)
        self.last_publish_time = None
        self.publish_count = 0

        try:
            from witmotion import IMU as WitmotionIMU
        except ImportError as exc:
            raise RuntimeError(
                'Python package "witmotion" is required for imu_publisher. '
                'Install it in the runtime environment before launching this node.'
            ) from exc

        self.imu = WitmotionIMU(path=imu_port, baudrate=baudrate)
        if hasattr(self.imu, 'set_update_rate'):
            try:
                self.imu.set_update_rate(desired_update_rate_hz)
                self.get_logger().info(f'Configured Witmotion update rate to {desired_update_rate_hz} Hz.')
            except Exception as exc:
                self.get_logger().warn(f'Could not set Witmotion update rate to {desired_update_rate_hz} Hz: {exc}')

        self.imu.subscribe(self.imu_callback)

        self.get_logger().info(
            f'Witmotion IMU connected on {imu_port} @ {baudrate}bps '
            f'(target stream {desired_update_rate_hz} Hz).'
        )

    def imu_callback(self, raw_message):
        line = str(raw_message).strip()
        if 'angle message' not in line:
            return

        try:
            roll_deg, pitch_deg, yaw_deg = self._parse_angle_message(line)
        except Exception as exc:
            self.get_logger().warn(f'Failed to parse Witmotion angle line: {line} | {exc}')
            return

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = normalize_radians(math.radians(self.yaw_multiplier * yaw_deg + self.yaw_offset_deg))
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        )
        msg.orientation_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.02,
        ]

        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.publisher_imu.publish(msg)
        self.publisher_imu_data.publish(msg)

        raw_yaw_msg = Float64()
        raw_yaw_msg.data = yaw_deg
        self.publisher_raw_yaw_deg.publish(raw_yaw_msg)

        ros_yaw_msg = Float64()
        ros_yaw_msg.data = math.degrees(yaw)
        self.publisher_ros_yaw_deg.publish(ros_yaw_msg)

        self.publish_count += 1
        self.last_publish_time = self.get_clock().now()

    @staticmethod
    def _parse_angle_message(line: str) -> tuple[float, float, float]:
        normalized = line.replace(',', ' ')
        values = {}
        for token in normalized.split():
            if ':' not in token:
                continue
            key, value = token.split(':', 1)
            values[key.strip().lower()] = float(value)

        return values['roll'], values['pitch'], values['yaw']

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
        node.get_logger().info('IMU orientation publisher stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
