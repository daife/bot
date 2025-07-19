from setuptools import find_packages, setup

package_name = 'arm_action_rclpy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daife',
    maintainer_email='3263584801@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_arm_01 = arm_action_rclpy.action_arm_01:main',
            'action_control_01 = arm_action_rclpy.action_control_01:main'
        ],
    },
)
