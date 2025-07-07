#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
bringup.launch.py

Launch the full robot navigation pipeline:
- datum_publisher_node
- gps_publisher
- rtk_status_monitor_node
- imu_filter_node
- gps_to_enu_odometry_node
- logged_waypoint_follower
- twist_mux

Usage:
    ros2 launch amiga_navigation bringup.launch.py waypoints:=<your_waypoints.yaml>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('amiga_navigation')

    # Declare waypoints argument
    waypoints_arg = DeclareLaunchArgument(
        'waypoints',
        default_value='/home/cairlab/navigation_waypoints/latest_waypoints.csv',
        description='Path to waypoints.csv file (latitude,longitude)'
    )

    waypoints = LaunchConfiguration('waypoints')

    # twist_mux launch file (includes yaml)
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'twist_mux_launch.launch.py')
        )
    )

    # datum_publisher_node
    datum_publisher_node = Node(
        package='amiga_navigation',  
        executable='datum_publisher_node',
        name='datum_publisher_node',
        output='screen'
    )

    # gps_publisher_node
    gps_publisher_node = Node(
        package='amiga_navigation',  
        executable='gps_publisher',
        name='gps_publisher',
        output='screen',
        parameters=[{
            'gps_port': '/dev/serial/by-id/usb-Emlid_ReachRS3_824368A16D8C568C-if02',
            'baudrate': 115200
        }]
    )

    # rtk_status_monitor_node
    rtk_status_monitor_node = Node(
        package='amiga_navigation',
        executable='rtk_monitor_node',
        name='rtk_monitor_node',
        output='screen'
    )

    # serial bridge (amiga_com)
    com = Node(
        package='amiga_navigation',
        executable='amiga_com',
        name='amiga_com',
        output='screen',
        arguments=[
            '--port', '/dev/serial/by-id/usb-Adafruit_Industries_LLC_Feather_M4_CAN_26CD336C48364C5320202054142F0DFF-if00',
            '--baudrate', '115200'
        ]
    )

    # imu_filter_node
    imu_filter_node = Node(
        package='amiga_navigation',  
        executable='imu_filter_node',
        name='imu_filter_node',
        output='screen'
    )

    imu_monitor_node = Node(
        package='amiga_navigation',  
        executable='imu_monitor_node',
        name='imu_monitor_node',
        output='screen'
    )

    # gps_to_enu_odometry_node
    gps_to_enu_odometry_node = Node(
        package='amiga_navigation',  
        executable='gps_to_enu_odometry_node',
        name='gps_to_enu_odometry_node',
        output='screen'
    )

    # logged_waypoint_follower
    logged_waypoint_follower = Node(
        package='amiga_navigation',  
        executable='logged_waypoint_follower',
        name='logged_waypoint_follower',
        output='screen',
        arguments=['--waypoints', waypoints]
    )

    return LaunchDescription([
        waypoints_arg,
        twist_mux,
        datum_publisher_node,
        gps_publisher_node,
        rtk_status_monitor_node,
        com,
        imu_filter_node,
        imu_monitor_node,
        gps_to_enu_odometry_node,
        logged_waypoint_follower
    ])
