from setuptools import find_packages, setup

package_name = 'multi_camera_trigger'

setup(
    name='multi_camera_trigger',
    version='0.3.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/multi_camera_trigger']),
        ('share/multi_camera_trigger', ['package.xml']),
    ],
    zip_safe=True,
    author='Yiyuan Lin',
    maintainer='Yiyuan Lin',
    maintainer_email='yl3663@cornell.edu',
    description='Multi-camera triggering with synchronized UM982 position and heading',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_camera_trigger_node = '
            'multi_camera_trigger.multi_camera_trigger_node:main',
            'gps_publisher = multi_camera_trigger.gps_publisher_node:main',
            'gps_logger = multi_camera_trigger.gps_logger_node:main',
        ],
    },

)
