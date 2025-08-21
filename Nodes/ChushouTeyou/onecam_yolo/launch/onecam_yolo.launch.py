from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='onecam_yolo',
            executable='onecam_yolo_node',
            name='onecam_yolo_node',
            output='screen',
        )
    ])
