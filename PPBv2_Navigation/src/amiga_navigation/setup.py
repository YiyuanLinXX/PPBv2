from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'amiga_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'simple-pid',
        'pyserial',
        'pynmea2',
        'pyproj',
        'witmotion',
    ],
    zip_safe=True,
    maintainer='Yiyuan Lin',
    maintainer_email='yl3663@cornell.edu',
    description='Amiga Navigation integration package for GPS+IMU navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_publisher = amiga_navigation.gnss_publisher:main',
            'datum_publisher = amiga_navigation.datum_publisher:main',
            'gnss2enu_odometry = amiga_navigation.gnss2enu_odometry:main',
            'gnss_waypoint_keyboard_logger = amiga_navigation.gnss_waypoint_keyboard_logger:main',
            'imu_publisher = amiga_navigation.imu_publisher:main',
            'imu_yaw_debugger = amiga_navigation.imu_yaw_debugger:main',
            'waypoint_follower = amiga_navigation.waypoint_follower:main',
            'localization_logger = amiga_navigation.localization_logger:main',
            'robot_odom_logger = amiga_navigation.robot_odom_logger:main',
            'nav_topic_debug_logger = amiga_navigation.nav_topic_debug_logger:main',
            'rtk_monitor = amiga_navigation.rtk_monitor:main',
            'amiga_serial_bridge = amiga_navigation.amiga_serial_bridge:main',
        ],
    },
)
