from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_tracking',
            executable='camera_tracking_node',
            name='camera_tracking_node',
            output='screen'
        )
    ])
