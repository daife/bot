from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('bottom_camera_yolo_detector')
    model_path = os.path.join(pkg_dir, 'models', 'yolo11s16.om')
    
    return LaunchDescription([
        Node(
            package='bottom_camera_yolo_detector',
            executable='yolo_detector',
            name='bottom_camera_yolo_detector',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'device_id': 0,
                'input_size': 640,
            }]
        )
    ])
