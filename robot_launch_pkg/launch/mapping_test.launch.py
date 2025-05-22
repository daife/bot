#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    建图测试所需的所有节点的启动描述
    包括激光雷达、滤波器、里程计IMU、传感器融合、机器人可视化、控制节点和建图节点
    """
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_lidar_viz = LaunchConfiguration('use_lidar_viz', default='false')
    
    # 激光雷达启动文件，根据参数决定是否使用可视化版本
    lidar_launch_file = 'ms200_scan_view.launch.py' if use_lidar_viz.perform(None) == 'true' else 'ms200_scan.launch.py'
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('oradar_lidar'), 'launch', lidar_launch_file)
        ])
    )
    
    # 激光雷达滤波器节点
    lidar_filter_node = Node(
        package='lidar_filter_pkg',
        executable='lidar_filter',
        name='lidar_filter_node',
        output='screen'
    )
    
    # 里程计IMU发布节点
    odom_imu_node = Node(
        package='odom_imu_publisher',
        executable='odom_imu_node',
        name='odom_imu_publisher',
        output='screen'
    )
    
    # 传感器融合启动文件
    sensor_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sensor_fusion_pkg'), 'launch', 'sensor_fusion.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 机器人模型可视化启动文件
    robot_visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('sam_bot_description'), 'launch', 'display.launch.py')
        ])
    )
    
    # 机器人底盘控制节点
    chassis_control_node = Node(
        package='chassis_control_rclpy',
        executable='chassis_control_node',
        name='chassis_control_node',
        output='screen'
    )
    
    # 机械臂控制节点
    arm_action_node = Node(
        package='arm_action_rclpy',
        executable='action_arm_01',
        name='arm_action_node',
        output='screen'
    )
    
    # 手柄控制节点
    handle_controller_node = Node(
        package='handlecontroler',
        executable='handle_control_node',
        name='handle_controller_node',
        output='screen'
    )
    
    # 建图节点启动文件
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('fishbot_cartographer'), 'launch', 'cartographer.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 返回启动描述
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'use_lidar_viz',
            default_value='false',
            description='Use lidar visualization if true'),
        lidar_launch,
        lidar_filter_node,
        odom_imu_node,
        sensor_fusion_launch,
        robot_visualization_launch,
        chassis_control_node,
        arm_action_node,
        handle_controller_node,
        cartographer_launch
    ])
