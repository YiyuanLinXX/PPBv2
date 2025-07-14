#!/usr/bin/env python3
"""
logged_waypoint_follower.py

ROS 2 node that reads GPS waypoints from a CSV file (latitude, longitude),
converts them to ENU coordinates using a datum, and performs navigation using
zero-point turning and line tracking based on imported control modules.

Uses:
- linear_drive.LineTrackingController
- linear_drive.is_goal_reached
- linear_drive.is_aligned
- zero_point_turn.get_cmd_turning

Logs progress to:
- status.txt (current waypoint index)
- yaw_log_{timestamp}.csv (pose, yaw, heading, etc.)

Package: amiga_navigation
ROS 2 Version: Jazzy
"""

import math
import sys
import csv
import time
import shutil
import rclpy
from rclpy.node import Node
import numpy as np
from pyproj import Transformer
from argparse import ArgumentParser
from rclpy.utilities import remove_ros_args

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from amiga_navigation.utils.linear_drive import LineTrackingController, is_goal_reached, is_aligned
from amiga_navigation.utils.zero_point_turn import get_cmd_turning


class LoggedWaypointFollower(Node):
    def __init__(self, csv_path):
        super().__init__('logged_waypoint_follower')

        # ROS publishers/subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.create_subscription(Odometry, '/robot/odom', self.odom_callback, 10)
        self.create_subscription(Float64MultiArray, '/gps/datum', self.datum_callback, 10)

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)

        # Internal state
        self.pose = [None, None, None]  # [x, y, yaw]
        self.transformer = None
        self.datum_received = False
        self.current_index = 0
        self.reached_final = False
        self.last_time = self.get_clock().now()
        self.route = None
        self.is_aligned = False

        # Parameters and paths
        self.csv_path = csv_path
        self.status_path = '/home/cairlab/navigation_waypoints/status.txt'
        self.last_wp_path = '/home/cairlab/navigation_waypoints/last_waypoints.csv'
        if self.csv_path != self.last_wp_path:
            shutil.copy(self.csv_path, self.last_wp_path)

        timestamp = int(time.time())
        self.csv_log_path = f'/home/cairlab/navigation_waypoints/yaw_log_{timestamp}.csv'

        # Load waypoints
        self.waypoints_gps = self.load_csv_waypoints(self.csv_path)
        self.waypoints_enu = []

        # Controller
        self.controller = LineTrackingController(self.get_logger())
        self.get_logger().info(
            f"LineTrackingController PID params: kp={self.controller.pid.Kp}, "
            f"ki={self.controller.pid.Ki}, kd={self.controller.pid.Kd}"
        )

        # Initialize log file
        with open(self.csv_log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'yaw_deg', 'target_x', 'target_y', 'path_angle_deg', 'yaw_error_deg'])

        self.get_logger().info('LoggedWaypointFollower initialized')

        # Check resume point
        if 'last_waypoints.csv' in self.csv_path:
            try:
                with open(self.status_path, 'r') as f:
                    lines = f.readlines()
                    if len(lines) >= 1:
                        saved_index = int(lines[0].strip())
                        if saved_index < len(self.waypoints_gps) - 1:
                            self.current_index = saved_index
                            self.get_logger().info(f'Resuming from waypoint index {self.current_index}')
            except Exception as e:
                self.get_logger().warn(f'Could not read saved index from status.txt: {e}')
        else:
            self.get_logger().info('Starting fresh navigation from index 0.')
        self.update_status_file()

    def load_csv_waypoints(self, path):
        waypoints = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # skip header line
            for row in reader:
                if len(row) >= 2:
                    lat, lon = float(row[0]), float(row[1])
                    waypoints.append((lat, lon))
        return waypoints

    def datum_callback(self, msg):
        if self.transformer is not None:
            return
        lat0, lon0, _ = msg.data
        self.transformer = Transformer.from_crs(
            'epsg:4326',
            f'+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0 +datum=WGS84',
            always_xy=True
        )
        for lat, lon in self.waypoints_gps:
            x, y = self.transformer.transform(lon, lat)
            self.waypoints_enu.append([x, y])
        self.get_logger().info('Converted waypoints to ENU coordinates.')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        self.pose = [pos.x, pos.y, yaw]

        # Insert current pose as waypoint[0], only once
        if self.pose[0] is not None and self.transformer is not None and not hasattr(self, 'init_pose_inserted'):
            init_xy = [self.pose[0], self.pose[1]]
            self.waypoints_enu.insert(0, init_xy)
            self.get_logger().info(f'Inserted current position as waypoint 0: {init_xy}')
            self.init_pose_inserted = True

    def control_loop(self):
        if self.reached_final or self.pose[0] is None or self.transformer is None:
            return

        if self.current_index >= len(self.waypoints_enu) - 1:
            self.cmd_pub.publish(Twist())
            self.reached_final = True
            self.update_status_file()
            self.get_logger().info('Navigation complete.')
            return

        pt1 = self.waypoints_enu[self.current_index]
        pt2 = self.waypoints_enu[self.current_index + 1]
        self.route = [pt1, pt2]

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Phase 1: alignment
        if not self.is_aligned:
            angular = get_cmd_turning(self.route, self.pose)
            twist = Twist()
            twist.angular.z = float(angular)
            self.cmd_pub.publish(twist)
            if is_aligned(self.route, self.pose, self.get_logger()):
                self.is_aligned = True
                self.get_logger().info('Alignment complete.')
            return

        # Phase 2: linear tracking
        v, w, error = self.controller.get_cmd_linear(self.route, self.pose)
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

        # Log
        yaw_error = math.atan2(pt2[1] - pt1[1], pt2[0] - pt1[0]) - self.pose[2]
        with open(self.csv_log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                round(now.nanoseconds / 1e9, 3),
                round(self.pose[0], 3),
                round(self.pose[1], 3),
                round(math.degrees(self.pose[2]), 2),
                round(pt2[0], 3),
                round(pt2[1], 3),
                round(math.degrees(math.atan2(pt2[1] - pt1[1], pt2[0] - pt1[0])), 2),
                round(math.degrees(yaw_error), 2)
            ])

        # Check goal
        if is_goal_reached(self.route, self.pose, self.get_logger()):
            self.get_logger().info(f'Reached waypoint {self.current_index + 1}')
            self.current_index += 1
            self.is_aligned = False
            self.update_status_file()

    def update_status_file(self):
        with open(self.status_path, 'w') as f:
            f.write(f'{self.current_index}\n')

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)


def main(args=None):
    rclpy.init(args=args)
    args_without_ros = remove_ros_args(sys.argv)
    parser = ArgumentParser()
    parser.add_argument('--waypoints', required=True, help='Path to CSV file containing latitude,longitude waypoints')
    parsed_args = parser.parse_args(args_without_ros[1:])

    node = LoggedWaypointFollower(parsed_args.waypoints)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
