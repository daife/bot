from setuptools import setup
import os
from glob import glob

package_name = 'keyboardcontroler'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if you have any
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HwHiAiUser',
    maintainer_email='user@todo.todo',
    description='Keyboard controller for mecanum wheeled robot, arm and claw',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control_node = keyboardcontroler.keyboard_control_node:main',
        ],
    },
)
