#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """生成机器人UI控制启动描述"""
    
    return LaunchDescription([
        # 启动机器人UI控制节点
        Node(
            package='robot_ui_control',
            executable='robot_ui_node',
            name='robot_ui_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        )
    ])
