#!/usr/bin/env python3
# coding=utf-8
# gps_logger_node.py
# ROS2 node: subscribe to /gps/fix_detail and log synchronized GPS data

import csv
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GpsLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')

        self.declare_parameter('log_file', '/tmp/gps_log.csv')
        self.declare_parameter('gps_detail_topic', '/gps/fix_detail')

        log_file = self.get_parameter('log_file').get_parameter_value().string_value
        gps_detail_topic = self.get_parameter('gps_detail_topic').get_parameter_value().string_value

        directory = os.path.dirname(log_file)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)

        self.csv_file = open(log_file, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'Satellite UTC',
            'ROS Time Stamp',
            'Latitude',
            'Longitude',
            'Altitude',
            'Fix Quality',
        ])
        self.csv_file.flush()

        self.create_subscription(String, gps_detail_topic, self.detail_callback, 10)

    def detail_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring malformed /gps/fix_detail payload: {exc}')
            return

        latitude = payload.get('latitude')
        longitude = payload.get('longitude')
        altitude = payload.get('altitude')

        self.writer.writerow([
            payload.get('satellite_utc', ''),
            payload.get('ros_time', ''),
            f'{latitude:.9f}' if latitude is not None else '',
            f'{longitude:.9f}' if longitude is not None else '',
            f'{altitude:.2f}' if altitude is not None else '',
            payload.get('fix_quality', ''),
        ])
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
