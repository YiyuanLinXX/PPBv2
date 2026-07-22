from setuptools import find_packages, setup

package_name = 'multi_camera_trigger'

setup(
    name='multi_camera_trigger',
    version='0.1.0',
    # packages=find_packages(include=['multi_camera_trigger', 'multi_camera_trigger.*']),
    packages=find_packages(),
    
    install_requires=[
        'setuptools',
        'rclpy',
        'pyserial',
        'sensor_msgs',
        'std_msgs',
    ],

    data_files=[
        # 1) tell ament_index about your bag
        ('share/ament_index/resource_index/packages',
         ['resource/multi_camera_trigger']),
        # 2) install package.xml under share/<pkg>
        ('share/multi_camera_trigger', ['package.xml']),
    ],
    zip_safe=True,
    author='Yiyuan Lin',
    maintainer='Yiyuan Lin',
    maintainer_email='yl3663@cornell.edu',
    description='Multi-camera hardware trigger node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_camera_trigger_node = multi_camera_trigger.multi_camera_trigger_node:main',
            'gps_publisher = multi_camera_trigger.gps_publisher_node:main',
            'gps_logger = multi_camera_trigger.gps_logger_node:main',
        ],
    },

)
