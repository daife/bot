#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
import subprocess

def generate_launch_description():
    # 检查摄像头节点是否已经在运行
    def check_node_running(node_name):
        try:
            # 使用ros2 node list检查节点是否存在
            result = subprocess.run(['ros2', 'node', 'list'], 
                                    stdout=subprocess.PIPE, 
                                    stderr=subprocess.PIPE,
                                    text=True)
            return node_name in result.stdout
        except Exception:
            return False
    
    # 检查两个摄像头节点是否运行
    top_camera_running = check_node_running('/top_camera_publisher')
    bottom_camera_running = check_node_running('/bottom_camera_publisher')
    
    # 声明参数，控制是否启动相机节点
    start_camera_nodes_arg = DeclareLaunchArgument(
        'start_camera_nodes',
        default_value='true' if not (top_camera_running and bottom_camera_running) else 'false',
        description='是否启动摄像头节点'
    )
    
    # 获取包路径
    pkg_dir = get_package_share_directory('dual_camera_pkg')
    
    # 导入双摄像头启动文件（仅在需要时启动）
    dual_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'dual_cameras.launch.py')
        ]),
        condition=IfCondition(LaunchConfiguration('start_camera_nodes'))
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
        start_camera_nodes_arg,
        dual_camera_launch,
        camera_preview_node
    ])