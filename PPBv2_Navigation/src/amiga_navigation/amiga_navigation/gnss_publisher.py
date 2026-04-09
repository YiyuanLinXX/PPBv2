#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish RTK GPS fixes and RTK status from GGA serial messages."""

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool
import pynmea2


DEFAULT_GPS_PORT = '/dev/serial/by-id/usb-Emlid_ReachRS3_8243ABB34D7B2976-if02'
DEFAULT_BAUDRATE = 115200

class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gnss_publisher')
        self.declare_parameter('gps_port', DEFAULT_GPS_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('poll_period_sec', 0.02)
        self.declare_parameter('serial_timeout_sec', 0.02)

        port = self.get_parameter('gps_port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        poll_period = self.get_parameter('poll_period_sec').get_parameter_value().double_value
        serial_timeout = self.get_parameter('serial_timeout_sec').get_parameter_value().double_value

        self.get_logger().info(f"Starting GPS Publisher on {port} @ {baud}bps")

        # Publishers
        self.pub_gps = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pub_rtk_status = self.create_publisher(Bool, '/gps/rtk_status_flag', 10)

        # Serial port
        self.ser = serial.Serial(port, baud, timeout=serial_timeout)

        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.create_timer(poll_period, self.timer_callback)

    def timer_callback(self):
        latest = None
        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()
                if raw:
                    latest = raw
        except Exception as e:
            self.get_logger().warning(f"Serial read error: {e}")
            return

        if latest is None:
            return

        # Only parse GGA
        try:
            msg = pynmea2.parse(latest)
        except Exception:
            return

        # pynmea2 normalizes both $GPGGA and $GNGGA to sentence_type == 'GGA'.
        if msg.sentence_type != 'GGA':
            return

        if not msg.latitude or not msg.longitude:
            return

        try:
            lat = float(msg.latitude)
            lon = float(msg.longitude)
            alt = float(msg.altitude)
            qual = int(msg.gps_qual)
        except Exception:
            return

        if qual == 4:  # FIX RTK only
            self.latitude = lat
            self.longitude = lon
            self.altitude = alt

            now = self.get_clock().now().to_msg()

            gps_msg = NavSatFix()
            gps_msg.header.stamp = now
            gps_msg.header.frame_id = 'gps_link'
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = alt
            gps_msg.status.status = NavSatStatus.STATUS_FIX
            gps_msg.status.service = NavSatStatus.SERVICE_GPS
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            gps_msg.position_covariance = [
                0.02**2, 0.0, 0.0,
                0.0, 0.02**2, 0.0,
                0.0, 0.0, 0.05**2
            ]
            self.pub_gps.publish(gps_msg)

            self.pub_rtk_status.publish(Bool(data=False))  # FIX OK
        else:
            self.pub_rtk_status.publish(Bool(data=True))  # NOT FIX

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
