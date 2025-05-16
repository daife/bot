from setuptools import setup
import os
from glob import glob

package_name = 'odom_imu_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HwHiAiUser',
    maintainer_email='you@example.com',
    description='A ROS2 node that publishes odometry and IMU data from serial input',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_imu_node = odom_imu_publisher.odom_imu_node:main',
        ],
    },
)
