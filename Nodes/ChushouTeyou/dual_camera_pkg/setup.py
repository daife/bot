from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dual_camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='HwHiAiUser',
    maintainer_email='you@example.com',
    description='用于发布上方和下方摄像头原始图像及校正图像数据的ROS 2功能包',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'top_camera_node = dual_camera_pkg.top_camera_node:main',
            'bottom_camera_node = dual_camera_pkg.bottom_camera_node:main',
            'camera_preview_node = dual_camera_pkg.camera_preview_node:main',
        ],
    },
)
