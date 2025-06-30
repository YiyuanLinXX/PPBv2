#!/usr/bin/env python3
# coding=utf-8
# gps_logger_node.py
# ROS2 node: subscribe to /gps/utc and /gps/fix, log data into a CSV file

import os
import csv
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class GpsLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')
        # Declare and read log file parameter
        self.declare_parameter('log_file', '/tmp/gps_log.csv')
        log_file = self.get_parameter('log_file').get_parameter_value().string_value

        # Ensure directory exists
        directory = os.path.dirname(log_file)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)

        # Open CSV and write header
        self.csv_file = open(log_file, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'Satellite UTC',
            'ROS Time Stamp',
            'Latitude',
            'Longitude',
            'Altitude',
            'Fix Quality'
        ])
        self.csv_file.flush()

        # Cache latest UTC string
        self.latest_utc = ''

        # Subscribe to /gps/utc (String) and /gps/fix (NavSatFix)
        self.create_subscription(String, '/gps/utc', self.utc_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.fix_callback, 10)

    def utc_callback(self, msg: String):
        # Update latest UTC string on each message
        self.latest_utc = msg.data

    def fix_callback(self, msg: NavSatFix):
        # Format ROS timestamp as sec.nanosec
        t = msg.header.stamp
        ros_ts = f"{t.sec}.{t.nanosec:09d}"
        # Extract coordinates and fix quality
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        quality = msg.status.status

        # Write one CSV row for every fix message (including no-fix)
        self.writer.writerow([
            self.latest_utc,
            ros_ts,
            f"{lat:.9f}",
            f"{lon:.9f}",
            f"{alt:.2f}",
            quality
        ])
        self.csv_file.flush()

    def destroy_node(self):
        # Close the CSV file on shutdown
        try:
            self.csv_file.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GpsLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
