#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    启动手柄测试所需的所有节点
    该启动文件包括底盘控制、机械臂控制、爪子控制和手柄控制节点
    """
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 机器人底盘控制节点
    chassis_control_node = Node(
        package='chassis_control_rclpy',
        executable='chassis_control_node',
        name='chassis_control_node',
        output='screen'
    )
    
    # 机械臂控制节点,包含部分部件坐标发布
    arm_action_node = Node(
        package='arm_action_rclpy',
        executable='action_arm_01',
        name='arm_action_node',
        output='screen'
    )
    
    # 爪子控制节点
    claw_action_node = Node(
        package='claw_action_rclpy',
        executable='action_claw_01',
        name='claw_action_node',
        output='screen'
    )
    
    # 手柄控制节点
    handle_controller_node = Node(
        package='handlecontroler',
        executable='handle_control_node',
        name='handle_controller_node',
        output='screen'
    )
    
    # 返回启动描述
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        chassis_control_node,
        arm_action_node,
        claw_action_node,
        handle_controller_node
    ])
