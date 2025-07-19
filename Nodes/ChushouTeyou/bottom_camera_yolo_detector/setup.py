from setuptools import setup
import os
from glob import glob

package_name = 'bottom_camera_yolo_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include model files
        (os.path.join('share', package_name, 'models'), glob('models/*.om')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HwHiAiUser',
    maintainer_email='user@todo.todo',
    description='YOLO detector for bottom camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = bottom_camera_yolo_detector.yolo_detector_node:main',
        ],
    },
)
