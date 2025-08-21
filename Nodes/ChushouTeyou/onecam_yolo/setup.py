from setuptools import setup
import os
from glob import glob

package_name = 'onecam_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 不再包含模型参数相关内容
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HwHiAiUser',
    maintainer_email='user@todo.todo',
    description='YOLO detector for one camera, publishes image topic',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'onecam_yolo_node = onecam_yolo.yolo_node:main',
        ],
    },
)
