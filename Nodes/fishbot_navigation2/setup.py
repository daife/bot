from setuptools import setup
import os

package_name = 'fishbot_navigation2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/fishbot_navigation2.launch.py']),
        (f'share/{package_name}/config', ['config/nav2_params.yaml']),
        (f'share/{package_name}/maps', [
            'maps/static_map.yaml',
            'maps/static_map.pgm',  # 添加地图图片文件
            'maps/keepout_mask.yaml',
            'maps/keepout_mask.pgm',  # 添加禁止区域图片文件
            # 你可以在这里添加更多地图文件
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HwHiAiUser',
    maintainer_email='your@email.com',
    description='Fishbot navigation2 python package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
