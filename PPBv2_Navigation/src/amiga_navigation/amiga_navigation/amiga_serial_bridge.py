#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Serial bridge between ROS 2 `/cmd_vel_out` and the Amiga MCU."""

import serial
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AmigaCom(Node):
    def __init__(self, port: str, baud: int):
        super().__init__('amiga_serial_bridge')
        self.get_logger().info(f'Starting AmigaCom node on port {port} @ {baud}bps')

        # Single serial instance for read & write
        self.ser = serial.Serial(port, baud, timeout=1)

        # ROS interfaces
        self.cmd_sub  = self.create_subscription(
            Twist, '/cmd_vel_out', self.cmd_vel_callback, 10)

        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timeout = 2.0  # seconds
        self.watchdog_triggered = False
        # Timer at 10 Hz to enforce watchdog and drain MCU feedback
        self.create_timer(0.1, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist):
        """
        Receive Twist on /cmd_vel_out and forward to MCU.
        Format: "v,omega\n"
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
        """Enforce watchdog and discard any pending MCU feedback."""

        now = self.get_clock().now()
        dt_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9
        if dt_cmd > self.watchdog_timeout:
            if not self.watchdog_triggered:
                try:
                    self.ser.write(b"0.000000,0.000000\n")
                    self.ser.flush()
                    self.get_logger().warn(f"Watchdog triggered: {dt_cmd:.2f}s since last cmd_vel, sending STOP.")
                    self.watchdog_triggered = True
                except Exception as e:
                    self.get_logger().warning(f"Watchdog send failed: {e}")

        # Drain any feedback lines so the serial buffer does not grow unbounded.
        try:
            while self.ser.in_waiting > 0:
                self.ser.readline()
        except Exception as e:
            self.get_logger().warning(f"Serial read failed: {e}")
                
    def destroy_node(self):
        try:
            self.get_logger().info("Shutting down AmigaCom: sending final stop command")
            self.ser.write(b"0.000000,0.000000\n")
            self.ser.flush()
        except Exception as e:
            self.get_logger().warning(f"Failed to send final stop command: {e}")
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',      type=str,
                        default='/dev/serial/by-id/usb-Adafruit_Industries_LLC_Feather_M4_CAN_26CD336C48364C5320202054142F0DFF-if00',
                        help='Serial port for MCU communication')
    parser.add_argument('--baudrate',  type=int, default=115200,
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
