from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_dir = get_package_share_directory('bottom_camera_yolo_detector')
    
    # 声明启动参数，允许命令行覆盖
    model_file = LaunchConfiguration('model_file', default='yolo11s.om')
    model_dir = LaunchConfiguration('model_dir', default=os.path.join(pkg_dir, 'models'))
    display_image = LaunchConfiguration('display_image', default='false')
    
    return LaunchDescription([
        # 声明可以从命令行传入的参数
        DeclareLaunchArgument(
            'model_file',
            default_value='yolo11s.om',
            description='YOLO模型文件名'
        ),
        
        DeclareLaunchArgument(
            'model_dir',
            default_value=os.path.join(pkg_dir, 'models'),
            description='YOLO模型所在目录'
        ),
        
        DeclareLaunchArgument(
            'display_image',
            default_value='true',
            description='是否显示检测结果窗口 (true/false)'
        ),
        
        Node(
            package='bottom_camera_yolo_detector',
            executable='yolo_detector',
            name='bottom_camera_yolo_detector',
            output='screen',
            parameters=[{
                'model_path': [model_dir, '/', model_file],  # 使用完全参数化的模型路径
                'device_id': 2,
                'input_size': 640,
                'num_classes': 4,
                'display_image': display_image,
            }]
        )
    ])
