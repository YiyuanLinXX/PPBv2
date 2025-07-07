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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yiyuan Lin',
    maintainer_email='yl3663@cornell.edu',
    description='Amiga Navigation integration package for GPS+IMU navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_publisher = amiga_navigation.gps_publisher:main',
            'rtk_monitor_node = amiga_navigation.rtk_monitor_node:main',
            'amiga_com = amiga_navigation.amiga_com:main',
            'gps_waypoint_logger_keyboard = amiga_navigation.gps_waypoint_logger_keyboard:main',
            'logged_waypoint_follower = amiga_navigation.logged_waypoint_follower:main',
            'datum_publisher_node = amiga_navigation.datum_publisher_node:main',
            'gps_to_enu_odometry_node = amiga_navigation.gps_to_enu_odometry_node:main',
            'imu_monitor_node = amiga_navigation.imu_monitor_node:main',
        ],
    },
)
