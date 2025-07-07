#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_waypoint_logger_keyboard.py

Record GPS (NavSatFix) pose to two CSV files when SPACE is pressed:
  • latest_waypoints.csv        (overwritten each time, session-local)
  • waypoints_YYYY_MM_DD_HH_MM_SS.csv  (historical snapshot created once per session)

Press 'q' to quit.

Usage:
  ros2 run amiga_navigation gps_waypoint_logger_keyboard \
    --output_dir /home/cairlab/navigation_waypoints
"""

import os
import sys
import csv
import select
import termios
import tty
import argparse
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GpsKeyboardLogger(Node):
    def __init__(self, output_dir: str):
        super().__init__('gps_keyboard_logger')
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        self.latest_path = os.path.join(self.output_dir, 'latest_waypoints.csv')
        ts = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self.hist_path = os.path.join(self.output_dir, f"waypoints_{ts}.csv")

        self.last_gps = None  # type: NavSatFix
        self.logged_waypoints = []

        # Subscriptions
        self.create_subscription(NavSatFix, 'gps/fix', self.gps_cb, 10)

        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info(f"Historical snapshot file: {os.path.basename(self.hist_path)}")
        self.get_logger().info("Press SPACE to log waypoint, 'q' to quit.")

    def gps_cb(self, msg: NavSatFix):
        """Store the latest GPS fix."""
        self.last_gps = msg

    def log_waypoint(self):
        """Append the current (lat, lon) to memory and write to CSV files."""
        if self.last_gps is None:
            self.get_logger().warn("No GPS data received yet.")
            return

        lat = self.last_gps.latitude
        lon = self.last_gps.longitude
        self.logged_waypoints.append((lat, lon))

        # Save latest CSV (overwrite)
        try:
            with open(self.latest_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['latitude', 'longitude'])
                writer.writerows(self.logged_waypoints)
        except Exception as e:
            self.get_logger().error(f"Failed to write latest CSV: {e}")
            return

        # Save historical snapshot CSV (overwrite each time for safety)
        try:
            with open(self.hist_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['latitude', 'longitude'])
                writer.writerows(self.logged_waypoints)
        except Exception as e:
            self.get_logger().error(f"Failed to write history CSV: {e}")
            return

        idx = len(self.logged_waypoints)
        self.get_logger().info(
            f"Logged waypoint #{idx}: lat={lat:.6f}, lon={lon:.6f} → saved to CSV"
        )


def configure_terminal():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)
    return old_settings


def restore_terminal(old_settings):
    fd = sys.stdin.fileno()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output_dir',
        dest='output_dir',
        type=str,
        default='/home/cairlab/navigation_waypoints',
        help='Directory to save latest and timestamped waypoint files'
    )
    parsed, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = GpsKeyboardLogger(parsed.output_dir)

    old_settings = configure_terminal()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == ' ':
                    node.log_waypoint()
                elif ch.lower() == 'q' or ch == '\x03':
                    break
    except KeyboardInterrupt:
        pass
    finally:
        restore_terminal(old_settings)
        node.destroy_node()
        rclpy.shutdown()

        print("\nExited GPS Keyboard Logger.")
        print(f"Waypoints saved to:")
        print(f"  Latest → {node.latest_path}")
        print(f"  Snapshot → {node.hist_path}")


if __name__ == '__main__':
    main()
