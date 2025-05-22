#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    视觉测试所需的所有节点的启动描述
    包括双摄像头、YOLO目标检测和物体定位器
    """
    # 声明参数
    use_camera_preview = LaunchConfiguration('use_camera_preview', default='false')
    
    # 双摄像头启动文件，根据参数决定是否使用预览版本
    camera_launch_file = 'preview_cameras.launch.py' if use_camera_preview.perform(None) == 'true' else 'dual_cameras.launch.py'
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dual_camera_pkg'), 'launch', camera_launch_file)
        ])
    )
    
    # YOLO检测器启动文件
    yolo_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('bottom_camera_yolo_detector'), 'launch', 'yolo_detector.launch.py')
        ])
    )
    
    # 物体定位器启动文件
    object_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('object_localizer'), 'launch', 'object_localizer.launch.py')
        ])
    )
    
    # 返回启动描述
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera_preview',
            default_value='false',
            description='Use camera preview if true'),
        camera_launch,
        yolo_detector_launch,
        object_localizer_launch
    ])
