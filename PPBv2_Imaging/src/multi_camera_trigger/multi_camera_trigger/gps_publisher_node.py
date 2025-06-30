#!/usr/bin/env python3
# coding=utf-8
# =============================================================================
# gps_publisher_node.py
# ROS2 node: read NMEA GPGGA from serial and publish:
#   - sensor_msgs/NavSatFix on /gps/fix (including raw fix quality)
#   - std_msgs/String on /gps/utc for raw satellite UTC time hh:mm:ss.ss
# =============================================================================

import threading
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        # parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        # open serial port
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Opened GPS serial on {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open GPS serial: {e}')
            rclpy.shutdown()
            return

        # publishers
        self.pub_fix = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pub_utc = self.create_publisher(String, '/gps/utc', 10)

        # start background read thread
        thread = threading.Thread(target=self.read_loop, daemon=True)
        thread.start()

    def read_loop(self):
        while rclpy.ok():
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            if not line.startswith('$GPGGA'):
                continue
            parts = line.split(',')
            # need at least fields up to quality
            if len(parts) < 7:
                continue

            # parse satellite UTC (hhmmss.ss -> hh:mm:ss.ss)
            raw_utc = parts[1]
            utc_str = ''
            if len(raw_utc) >= 6:
                hh = raw_utc[0:2]
                mm = raw_utc[2:4]
                ss = raw_utc[4:]
                utc_str = f"{hh}:{mm}:{ss}"
                # publish raw UTC
                utc_msg = String()
                utc_msg.data = utc_str
                self.pub_utc.publish(utc_msg)

            # parse latitude
            try:
                raw_lat = parts[2]
                lat_deg = float(raw_lat[:2])
                lat_min = float(raw_lat[2:])
                lat = lat_deg + lat_min / 60.0
                if parts[3] == 'S':
                    lat = -lat
                # parse longitude
                raw_lon = parts[4]
                lon_deg = float(raw_lon[:3])
                lon_min = float(raw_lon[3:])
                lon = lon_deg + lon_min / 60.0
                if parts[5] == 'W':
                    lon = -lon
            except Exception:
                continue

            # fix quality and altitude
            try:
                quality = int(parts[6])
            except ValueError:
                quality = 0
            alt = 0.0
            if len(parts) > 9 and parts[9]:
                try:
                    alt = float(parts[9])
                except ValueError:
                    pass

            # build NavSatFix
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps'
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt

            status = NavSatStatus()
            status.service = NavSatStatus.SERVICE_GPS
            # use raw quality directly:
            status.status = quality
            msg.status = status

            # publish fix (including quality=0,1,4,5,...)
            self.pub_fix.publish(msg)

    def destroy_node(self):
        try:
            self.ser.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
