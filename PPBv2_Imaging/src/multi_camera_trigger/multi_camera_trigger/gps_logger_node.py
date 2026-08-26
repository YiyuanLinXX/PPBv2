#!/usr/bin/env python3
"""Log detailed GNSS fixes to CSV without silently overwriting prior data."""

import csv
import json
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class GpsLogger(Node):
    """Subscribe to the detailed GPS topic and write validated CSV rows."""

    def __init__(self):
        """Open a new CSV log and create the GPS subscription."""
        super().__init__('gps_logger')

        self.initialized = False
        self.csv_file = None
        self.subscription = None
        self.declare_parameter('log_file', '/tmp/gps_log.csv')
        self.declare_parameter('gps_detail_topic', '/imaging/gps/fix_detail')
        self.declare_parameter('overwrite_existing', False)

        log_file = str(self.get_parameter('log_file').value)
        gps_detail_topic = str(self.get_parameter('gps_detail_topic').value)
        overwrite_existing = bool(self.get_parameter('overwrite_existing').value)

        try:
            directory = os.path.dirname(log_file)
            if directory:
                os.makedirs(directory, exist_ok=True)

            mode = 'w' if overwrite_existing else 'x'
            self.csv_file = open(log_file, mode, newline='')
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow([
                'Satellite UTC',
                'ROS Time Stamp',
                'ANT1 Latitude',
                'ANT1 Longitude',
                'ANT1 Altitude',
                'Midpoint Latitude',
                'Midpoint Longitude',
                'Midpoint Altitude',
                'Midpoint Computed',
                'Fix Quality',
                'Differential Age(s)',
                'Baseline Heading(deg)',
                'Robot Heading(deg)',
                'Pitch(deg)',
                'Heading StdDev(deg)',
                'Pitch StdDev(deg)',
                'Heading Baseline(m)',
                'Heading Satellites Tracked',
                'Heading Satellites Used',
                'Heading Solution Status',
                'Heading Position Type',
                'Heading ROS Time(s.ns)',
                'GGA-Heading Delta(s)',
                'Heading Valid',
                'Dual Solution Valid',
                'Dual Solution Reason',
            ])
            self.csv_file.flush()

            self.subscription = self.create_subscription(
                String,
                gps_detail_topic,
                self.detail_callback,
                10,
            )
            self.initialized = True
        except Exception as exc:
            self.get_logger().error(f'Unable to initialize GPS log {log_file}: {exc}')
            if self.csv_file is not None:
                self.csv_file.close()
                self.csv_file = None

    def detail_callback(self, msg: String):
        """Validate and append one detailed GPS message."""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().warn(
                f'Ignoring malformed /imaging/gps/fix_detail payload: {exc}'
            )
            return

        ant1_latitude = payload.get('ant1_latitude', payload.get('latitude'))
        ant1_longitude = payload.get('ant1_longitude', payload.get('longitude'))
        ant1_altitude = payload.get('ant1_altitude', payload.get('altitude'))

        try:
            self.writer.writerow([
                payload.get('satellite_utc', ''),
                payload.get('ros_time', ''),
                self._format_optional_float(ant1_latitude, 9),
                self._format_optional_float(ant1_longitude, 9),
                self._format_optional_float(ant1_altitude, 3),
                self._format_optional_float(
                    payload.get('midpoint_latitude'), 9
                ),
                self._format_optional_float(
                    payload.get('midpoint_longitude'), 9
                ),
                self._format_optional_float(
                    payload.get('midpoint_altitude'), 3
                ),
                self._format_bool(payload.get('midpoint_computed')),
                payload.get('fix_quality', ''),
                self._format_optional_float(
                    payload.get('differential_age_sec'), 3
                ),
                self._format_optional_float(
                    payload.get('baseline_heading_deg'), 6
                ),
                self._format_optional_float(
                    payload.get('robot_heading_deg'), 6
                ),
                self._format_optional_float(payload.get('pitch_deg'), 6),
                self._format_optional_float(
                    payload.get('heading_stddev_deg'), 6
                ),
                self._format_optional_float(
                    payload.get('pitch_stddev_deg'), 6
                ),
                self._format_optional_float(
                    payload.get('heading_baseline_m'), 4
                ),
                payload.get('heading_satellites_tracked', ''),
                payload.get('heading_satellites_used', ''),
                payload.get('heading_solution_status', ''),
                payload.get('heading_position_type', ''),
                payload.get('heading_ros_time', ''),
                self._format_optional_float(payload.get('heading_age_sec'), 6),
                self._format_bool(payload.get('heading_valid')),
                self._format_bool(payload.get('dual_solution_valid')),
                payload.get('dual_solution_reason', ''),
            ])
            self.csv_file.flush()
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f'Ignoring invalid GPS detail payload: {exc}')

    @staticmethod
    def _format_optional_float(value, precision):
        if value is None or value == '':
            return ''
        return f'{float(value):.{precision}f}'

    @staticmethod
    def _format_bool(value):
        if value is None or value == '':
            return ''
        return int(bool(value))

    def destroy_node(self):
        """Close the CSV file before destroying the ROS node."""
        try:
            if self.csv_file is not None:
                self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """Run the GPS logger node."""
    rclpy.init(args=args)
    node = None
    success = False
    try:
        node = GpsLogger()
        if node.initialized and rclpy.ok():
            success = True
            try:
                rclpy.spin(node)
            except ExternalShutdownException:
                pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
