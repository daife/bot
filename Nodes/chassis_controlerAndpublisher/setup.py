from setuptools import find_packages, setup

package_name = 'chassis_controlerAndpublisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # 移除exclude，确保能找到所有包
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/chassis_controlerAndpublisher.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HwHiAiUser',
    maintainer_email='you@example.com',
    description='Chassis control and odometry publisher node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_controlerAndpublisher = chassis_controlerAndpublisher.chassis_controlerAndpublisher:main',
        ],
    },
)
