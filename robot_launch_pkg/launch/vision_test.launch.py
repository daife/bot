#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    视觉测试所需的所有节点的启动描述
    包括双摄像头、YOLO目标检测和物体定位器
    """
    # 声明参数
    use_camera_preview_arg = DeclareLaunchArgument(
        'use_camera_preview',
        default_value='false',
        description='Use camera preview if true'
    )
    
    use_camera_preview = LaunchConfiguration('use_camera_preview')
    
    # 预览版摄像头启动（当use_camera_preview为true时）
    preview_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dual_camera_pkg'),
                'launch',
                'preview_cameras.launch.py'
            ])
        ]),
        condition=IfCondition(use_camera_preview)
    )
    
    # 常规双摄像头启动（当use_camera_preview为false时）
    dual_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dual_camera_pkg'),
                'launch',
                'dual_cameras.launch.py'
            ])
        ]),
        condition=UnlessCondition(use_camera_preview)
    )
    
    # YOLO检测器启动文件
    yolo_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bottom_camera_yolo_detector'),
                'launch',
                'yolo_detector.launch.py'
            ])
        ])
    )
    
    # 物体定位器启动文件
    object_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('object_localizer'),
                'launch',
                'object_localizer.launch.py'
            ])
        ])
    )
    
    # 返回启动描述
    return LaunchDescription([
        use_camera_preview_arg,
        preview_camera_launch,
        dual_camera_launch,
        yolo_detector_launch,
        object_localizer_launch
    ])