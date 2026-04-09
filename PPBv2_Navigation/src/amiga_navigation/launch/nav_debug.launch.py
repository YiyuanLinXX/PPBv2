#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch the base navigation stack plus all debug logging nodes.

This debug launch intentionally does not start ``waypoint_follower``.
Run the follower separately in another terminal so the waypoint file and
controller can still be chosen interactively.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('amiga_navigation')

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'basic_bringup.launch.py')
        )
    )

    localization_logger = Node(
        package='amiga_navigation',
        executable='localization_logger',
        name='localization_logger',
        output='screen'
    )

    robot_odom_logger = Node(
        package='amiga_navigation',
        executable='robot_odom_logger',
        name='robot_odom_logger',
        output='screen'
    )

    return LaunchDescription([
        base_bringup,
        localization_logger,
        robot_odom_logger,
    ])
