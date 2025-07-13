#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    创建摄像头跟踪节点的启动描述
    """
    # 获取包路径
    pkg_dir = get_package_share_directory('camera_tracking')
    
    # 配置文件路径
    config_file = os.path.join(pkg_dir, 'config', 'tracking_params.yaml')
    
    # 声明启动参数
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='跟踪参数配置文件路径'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='是否启用调参模式 (true/false)'
    )
    
    debug_config_path_arg = DeclareLaunchArgument(
        'debug_config_path',
        default_value='/home/HwHiAiUser/ros/src/Nodes/camera_tracking/config/tracking_params.yaml',
        description='调参模式配置文件路径'
    )
    
    # 创建摄像头跟踪节点
    camera_tracking_node = Node(
        package='camera_tracking',
        executable='camera_tracking_node',
        name='camera_tracking_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'debug_mode': LaunchConfiguration('debug_mode'),
                'debug_config_path': LaunchConfiguration('debug_config_path')
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        config_arg,
        debug_mode_arg,
        debug_config_path_arg,
        camera_tracking_node
    ])
