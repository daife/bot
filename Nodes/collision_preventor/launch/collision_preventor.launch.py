from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='collision_preventor',
            executable='collision_preventor_node',
            name='collision_preventor_node',
            output='screen',
            parameters=[
                {'wall_radius': 0.2},
                {'mine_radius': 0.3},
                {'wall_max_speed': 0.5},
                {'mine_max_speed': 1.0}
            ]
        )
    ])
