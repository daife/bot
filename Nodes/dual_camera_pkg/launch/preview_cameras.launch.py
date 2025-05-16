#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 导入双摄像头启动文件
    dual_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dual_camera_pkg'), 
                        'launch', 
                        'dual_cameras.launch.py')
        ])
    )
    
    # 创建摄像头预览节点
    camera_preview_node = Node(
        package='dual_camera_pkg',
        executable='camera_preview_node',
        name='camera_preview',
        output='screen'
    )
    
    # 返回启动描述
    return LaunchDescription([
        dual_camera_launch,
        camera_preview_node
    ])