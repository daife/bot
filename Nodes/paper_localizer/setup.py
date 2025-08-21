from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'paper_localizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.om')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HwHiAiUser',
    maintainer_email='you@example.com',
    description='纸条定位节点，使用分割模型检测纸条并发布中心点坐标',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'paper_localizer_node = paper_localizer.paper_localizer_node:main',
            'paper_center_kalman_node = paper_localizer.paper_center_kalman_node:main',
        ],
    },
)
