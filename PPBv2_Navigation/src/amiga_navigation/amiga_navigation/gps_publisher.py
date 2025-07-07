#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_publisher.py

Reads GGA and GPETC messages from Emlid Reach RS3 over serial.
Publishes GPS fixes to /gps/fix, and IMU heading to /imu and /imu/data.
Only publishes heading when GPETC state == 30 (compensating).

Additionally publishes:
- /gps/rtk_status_flag: Bool, True if not FIX RTK
- /imu/state: Int32, IMU tilt state (""=off, 0=fatal, 10=setup, 20=alignment, 30=compensating)
- /imu/notification: Int32, IMU tilt warning (""=off, 0=none, 10=fast motion, etc.)

Author: Yiyuan Lin
"""

import math
import serial
import pynmea2
import argparse

import rclpy
from rclpy.node import Node
from amiga_navigation.utils.gps_utils import quaternion_from_euler
from sensor_msgs.msg import NavSatFix, Imu, NavSatStatus
from std_msgs.msg import Bool, Int32


class GpsPublisher(Node):
    def __init__(self, port: str, baud: int):
        super().__init__('gps_publisher')
        self.get_logger().info(f"Starting GPS Publisher on {port} @ {baud}bps")

        # Publishers
        self.pub_gps = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu', 10)
        self.pub_imu_data = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_rtk_status = self.create_publisher(Bool, '/gps/rtk_status_flag', 10)
        self.pub_state = self.create_publisher(Int32, '/imu/state', 10)
        self.pub_notification = self.create_publisher(Int32, '/imu/notification', 10)

        # Serial port
        self.ser = serial.Serial(port, baud, timeout=1)

        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.heading_deg = float('nan')
        self.imu_state = -1
        self.imu_notification = -1

        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            raw = self.ser.readline().decode('ascii', errors='ignore').strip()
        except Exception as e:
            self.get_logger().warning(f"Serial read error: {e}")
            return

        if not raw:
            return

        # --- GPETC: parse tilt compensation status ---
        if raw.startswith('$GPETC'):
            parts = raw.split(',')
            if len(parts) >= 9:
                try:
                    state_str = parts[1]
                    notif_str = parts[2]
                    heading_str = parts[3]

                    state = int(state_str) if state_str.isdigit() else -1
                    notification = int(notif_str) if notif_str.isdigit() else -1
                    heading = float(heading_str) if heading_str else float('nan')

                    self.pub_state.publish(Int32(data=state))
                    self.pub_notification.publish(Int32(data=notification))

                    if state == 30 and not math.isnan(heading):
                        self.heading_deg = heading
                    else:
                        self.heading_deg = float('nan')
                except Exception as e:
                    self.get_logger().warn(f"GPETC parse failed: {e}")
                    self.pub_state.publish(Int32(data=-1))
                    self.pub_notification.publish(Int32(data=-1))
                    self.heading_deg = float('nan')
            else:
                self.get_logger().warn(f"GPETC field count too short: {raw}")
            return  # ✅ VERY IMPORTANT! don't process again in GGA



        # --- GGA: parse GPS fix ---
        try:
            msg = pynmea2.parse(raw)
        except Exception:
            return

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

                if not math.isnan(self.heading_deg):
                    yaw_enu = math.pi / 2 - math.radians(self.heading_deg)
                    imu_msg = Imu()
                    imu_msg.header.stamp = now
                    imu_msg.header.frame_id = 'imu_link'
                    imu_msg.orientation = quaternion_from_euler(0.0, 0.0, yaw_enu)
                    imu_msg.orientation_covariance = [
                        0.5, 0.0, 0.0,
                        0.0, 0.5, 0.0,
                        0.0, 0.0, 0.0304
                    ]
                    imu_msg.angular_velocity_covariance = [-1.0] * 9
                    imu_msg.linear_acceleration_covariance = [-1.0] * 9

                    self.pub_imu.publish(imu_msg)
                    self.pub_imu_data.publish(imu_msg)

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
                        default='/dev/serial/by-id/usb-Emlid_ReachRS3_824368A16D8C568C-if02')
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
