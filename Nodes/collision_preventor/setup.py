from setuptools import setup

package_name = 'collision_preventor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/collision_preventor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO: Maintainer',
    maintainer_email='user@todo.todo',
    description='Collision preventor node for safety zone feedback',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collision_preventor_node = collision_preventor.collision_preventor_node:main'
        ],
    },
)
