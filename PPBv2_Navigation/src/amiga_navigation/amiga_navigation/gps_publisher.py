#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_publisher.py

Reads GGA messages from Emlid Reach RS3 over serial.
Publishes GPS fixes to /gps/fix.

Additionally publishes:
- /gps/rtk_status_flag: Bool, True if not FIX RTK

Author: Yiyuan Lin
"""

import math
import serial
import pynmea2
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool
from pyproj import Transformer

class GpsPublisher(Node):
    def __init__(self, port: str, baud: int):
        super().__init__('gps_publisher')
        self.get_logger().info(f"Starting GPS Publisher on {port} @ {baud}bps")

        # Publishers
        self.pub_gps = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pub_rtk_status = self.create_publisher(Bool, '/gps/rtk_status_flag', 10)

        # Serial port
        self.ser = serial.Serial(port, baud, timeout=1)

        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.datum_lat = 0.0
        self.datum_lon = 0.0

        self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        try:
            raw = self.ser.readline().decode('ascii', errors='ignore').strip()
        except Exception as e:
            self.get_logger().warning(f"Serial read error: {e}")
            return

        if not raw:
            return

        # Only parse GGA
        try:
            msg = pynmea2.parse(raw)
        except Exception:
            return
        self.ser.reset_output_buffer()

        if msg.sentence_type in ('GGA', 'GNGGA'):
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--gps_port', type=str,
                        default='/dev/serial/by-id/usb-Emlid_ReachRS3_8243ABB34D7B2976-if02')
    parser.add_argument('--baudrate', type=int, default=115200)
    parsed, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = GpsPublisher(port=parsed.gps_port, baud=parsed.baudrate)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
