from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_ui_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HwHiAiUser',
    maintainer_email='you@example.com',
    description='Robot UI control interface with node management, topic viewer, content viewer and controller selector',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_ui_node = robot_ui_control.robot_ui_node:main',
        ],
    },
)
