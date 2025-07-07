#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
amiga_com.py

Serial bridge between MCU (Feather M4 CAN) and ROS2:
  • Subscribe to /cmd_vel_out (geometry_msgs/Twist), forward commands to MCU.
  • Implement watchdog: if no cmd_vel received within timeout, send STOP command.
Usage:
  ros2 run amiga_navigation amiga_com --port /dev/ttyUSB0 --baudrate 115200
"""

import serial
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AmigaCom(Node):
    def __init__(self, port: str, baud: int):
        super().__init__('amiga_com')
        self.get_logger().info(f'Starting AmigaCom node on port {port} @ {baud}bps')

        # Initialize serial port for MCU communication
        self.ser = serial.Serial(port, baud, timeout=1)

        # Subscribe to cmd_vel_out and forward to MCU
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_out', self.cmd_vel_callback, 10)

        # Watchdog settings
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timeout = 2.0  # seconds
        self.watchdog_triggered = False

        # Timer to check watchdog and clear serial buffer at 10 Hz
        self.create_timer(0.1, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist):
        """
        Forward received Twist on '/cmd_vel_out' to MCU.
        Format: "linear_x,angular_z\n"
        """
        try:
            cmd_str = f"{msg.linear.x:.6f},{msg.angular.z:.6f}\n"
            self.ser.write(cmd_str.encode('ascii'))
            self.ser.flush()
            self.get_logger().debug(f"Sent cmd_vel to MCU: {cmd_str.strip()}")
            self.last_cmd_time = self.get_clock().now()
            self.watchdog_triggered = False
        except Exception as e:
            self.get_logger().warning(f"Failed to send cmd_vel: {e}")

    def timer_callback(self):
        """
        Watchdog: send STOP if no cmd_vel within timeout.
        Also clears serial buffer.
        """
        now = self.get_clock().now()
        dt_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9
        if dt_cmd > self.watchdog_timeout and not self.watchdog_triggered:
            try:
                self.ser.write(b"0.000000,0.000000\n")
                self.ser.flush()
                self.get_logger().warn(
                    f"Watchdog triggered: no cmd_vel for {dt_cmd:.2f}s, sending STOP.")
                self.watchdog_triggered = True
            except Exception as e:
                self.get_logger().warning(f"Watchdog send failed: {e}")

        # Clear serial buffer
        try:
            while self.ser.in_waiting > 0:
                _ = self.ser.readline()
        except Exception as e:
            self.get_logger().warning(f"Failed to clear serial buffer: {e}")

    def destroy_node(self):
        """
        Send final STOP command and close serial port before shutdown.
        """
        try:
            self.get_logger().info("Shutting down: sending final STOP command.")
            self.ser.write(b"0.000000,0.000000\n")
            self.ser.flush()
        except Exception as e:
            self.get_logger().warning(f"Failed to send final STOP command: {e}")
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    parser = argparse.ArgumentParser(
        description="Serial bridge between MCU and ROS2: forward /cmd_vel_out to MCU with watchdog.")
    parser.add_argument('--port',      type=str,
                        default='/dev/serial/by-id/usb-Adafruit_Industries_LLC_Feather_M4_CAN_26CD336C48364C5320202054142F0DFF-if00',
                        help='Serial port for MCU communication')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='Baudrate for the serial port')
    parsed, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = AmigaCom(port=parsed.port, baud=parsed.baudrate)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
