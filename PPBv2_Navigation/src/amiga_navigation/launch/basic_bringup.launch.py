#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch the UM982 navigation stack without starting waypoint following."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amiga_navigation')
    um982_config_argument = DeclareLaunchArgument(
        'um982_config',
        default_value=os.path.join(pkg, 'config', 'um982.yaml'),
        description='Absolute path to the runtime UM982 YAML configuration.',
    )

    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'twist_mux_launch.launch.py')
        )
    )

    datum_publisher = Node(
        package='amiga_navigation',
        executable='datum_publisher',
        name='datum_publisher',
        output='screen'
    )

    um982_driver = Node(
        package='amiga_navigation',
        executable='um982_driver',
        name='um982_driver',
        output='screen',
        parameters=[LaunchConfiguration('um982_config')],
    )

    rtk_monitor = Node(
        package='amiga_navigation',
        executable='rtk_monitor',
        name='rtk_monitor',
        output='screen'
    )

    amiga_serial_bridge = Node(
        package='amiga_navigation',
        executable='amiga_serial_bridge',
        name='amiga_serial_bridge',
        output='screen',
        arguments=[
            '--port',
            (
                '/dev/serial/by-id/usb-Adafruit_Industries_LLC_Feather_M4_CAN_C06A5AE248364C532020205439190DFF-if00'
            ),
            '--baudrate', '115200'
        ]
    )

    return LaunchDescription([
        um982_config_argument,
        twist_mux,
        datum_publisher,
        um982_driver,
        rtk_monitor,
        amiga_serial_bridge,
    ])
