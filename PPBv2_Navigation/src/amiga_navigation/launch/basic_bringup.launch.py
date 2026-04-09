#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch the base GPS + HWT905 navigation nodes without starting waypoint following."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('amiga_navigation')

    # twist_mux launch file (includes yaml)
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'twist_mux_launch.launch.py')
        )
    )

    # GPS datum publisher
    datum_publisher = Node(
        package='amiga_navigation',  
        executable='datum_publisher',
        name='datum_publisher',
        output='screen'
    )

    # RTK GPS serial publisher
    gnss_publisher = Node(
        package='amiga_navigation',  
        executable='gnss_publisher',
        name='gnss_publisher',
        output='screen',
        parameters=[{
            'gps_port': '/dev/serial/by-id/usb-Emlid_ReachRS3_8243ABB34D7B2976-if02',
            'baudrate': 115200
        }]
    )

    # HWT905 IMU publisher
    imu_publisher = Node(
        package='amiga_navigation',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen',
    )

    # RTK safety monitor
    rtk_monitor = Node(
        package='amiga_navigation',
        executable='rtk_monitor',
        name='rtk_monitor',
        output='screen'
    )

    # Amiga serial bridge
    amiga_serial_bridge = Node(
        package='amiga_navigation',
        executable='amiga_serial_bridge',
        name='amiga_serial_bridge',
        output='screen',
        arguments=[
            '--port', '/dev/serial/by-id/usb-Adafruit_Industries_LLC_Feather_M4_CAN_C06A5AE248364C532020205439190DFF-if00',
            '--baudrate', '115200'
        ]
    )

    # GPS + IMU odometry fusion
    gnss2enu_odometry = Node(
        package='amiga_navigation',  
        executable='gnss2enu_odometry',
        name='gnss2enu_odometry',
        output='screen',
        parameters=[{
            'imu_topic': '/imu'
        }]
    )

    return LaunchDescription([
        twist_mux,
        datum_publisher,
        gnss_publisher,
        imu_publisher,
        rtk_monitor,
        amiga_serial_bridge,
        gnss2enu_odometry,
    ])
