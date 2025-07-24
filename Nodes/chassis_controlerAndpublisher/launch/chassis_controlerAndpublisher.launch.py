from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis_controlerAndpublisher',
            executable='chassis_controlerAndpublisher',
            name='chassis_controlerAndpublisher',
            output='screen'
        )
    ])
