#!/usr/bin/env python3
# coding=utf-8
# =============================================================================
# gps_publisher_node.py
# ROS2 node: read NMEA GPGGA from serial and publish:
#   - sensor_msgs/NavSatFix on /gps/fix
#   - std_msgs/String on /gps/utc for raw satellite UTC time hh:mm:ss.ss
#   - std_msgs/String on /gps/fix_detail as JSON with UTC + fix payload
# =============================================================================

import json
import threading

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        self.initialized = False
        self.ser = None
        self._stop_event = threading.Event()
        self._read_thread = None

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('utc_topic', '/gps/utc')
        self.declare_parameter('detail_topic', '/gps/fix_detail')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        fix_topic = self.get_parameter('fix_topic').get_parameter_value().string_value
        utc_topic = self.get_parameter('utc_topic').get_parameter_value().string_value
        detail_topic = self.get_parameter('detail_topic').get_parameter_value().string_value

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Opened GPS serial on {port} @ {baud}')
        except Exception as exc:
            self.get_logger().error(f'Failed to open GPS serial: {exc}')
            return

        self.pub_fix = self.create_publisher(NavSatFix, fix_topic, 10)
        self.pub_utc = self.create_publisher(String, utc_topic, 10)
        self.pub_detail = self.create_publisher(String, detail_topic, 10)

        self._read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self._read_thread.start()
        self.initialized = True

    @staticmethod
    def _format_satellite_utc(raw_utc: str) -> str:
        if len(raw_utc) < 6:
            return ''
        return f'{raw_utc[0:2]}:{raw_utc[2:4]}:{raw_utc[4:]}'

    @staticmethod
    def _map_navsat_status(quality: int) -> int:
        if quality <= 0:
            return NavSatStatus.STATUS_NO_FIX
        if quality == 1:
            return NavSatStatus.STATUS_FIX
        if quality == 2:
            return NavSatStatus.STATUS_SBAS_FIX
        if quality in (4, 5):
            return NavSatStatus.STATUS_GBAS_FIX
        return NavSatStatus.STATUS_FIX

    def _publish_detail(
        self,
        ros_time: str,
        satellite_utc: str,
        latitude: float,
        longitude: float,
        altitude: float,
        fix_quality: int,
        navsat_status: int,
    ) -> None:
        detail = String()
        detail.data = json.dumps(
            {
                'ros_time': ros_time,
                'satellite_utc': satellite_utc,
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude,
                'fix_quality': fix_quality,
                'navsat_status': navsat_status,
            },
            separators=(',', ':'),
        )
        self.pub_detail.publish(detail)

    def read_loop(self):
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warn(f'GPS serial read failed: {exc}')
                continue

            if not line.startswith('$GPGGA'):
                continue

            parts = line.split(',')
            if len(parts) < 10:
                continue

            satellite_utc = self._format_satellite_utc(parts[1])
            if satellite_utc:
                utc_msg = String()
                utc_msg.data = satellite_utc
                self.pub_utc.publish(utc_msg)

            try:
                raw_lat = parts[2]
                lat_deg = float(raw_lat[:2])
                lat_min = float(raw_lat[2:])
                latitude = lat_deg + lat_min / 60.0
                if parts[3] == 'S':
                    latitude = -latitude

                raw_lon = parts[4]
                lon_deg = float(raw_lon[:3])
                lon_min = float(raw_lon[3:])
                longitude = lon_deg + lon_min / 60.0
                if parts[5] == 'W':
                    longitude = -longitude
            except Exception:
                continue

            try:
                fix_quality = int(parts[6])
            except ValueError:
                fix_quality = 0

            altitude = 0.0
            if parts[9]:
                try:
                    altitude = float(parts[9])
                except ValueError:
                    altitude = 0.0

            stamp = self.get_clock().now().to_msg()
            ros_time = f'{stamp.sec}.{stamp.nanosec:09d}'
            navsat_status = self._map_navsat_status(fix_quality)

            msg = NavSatFix()
            msg.header.stamp = stamp
            msg.header.frame_id = 'gps'
            msg.latitude = latitude
            msg.longitude = longitude
            msg.altitude = altitude

            status = NavSatStatus()
            status.service = NavSatStatus.SERVICE_GPS
            status.status = navsat_status
            msg.status = status

            self.pub_fix.publish(msg)
            self._publish_detail(
                ros_time=ros_time,
                satellite_utc=satellite_utc,
                latitude=latitude,
                longitude=longitude,
                altitude=altitude,
                fix_quality=fix_quality,
                navsat_status=navsat_status,
            )

    def destroy_node(self):
        self._stop_event.set()
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        if self._read_thread is not None and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()
    try:
        if node.initialized and rclpy.ok():
            rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
