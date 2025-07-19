import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 package share 路径
    share_dir = get_package_share_directory('fishbot_navigation2')
    map_file = os.path.join(share_dir, 'maps', 'static_map.yaml')
    nav2_params = os.path.join(share_dir, 'config', 'nav2_params.yaml')

    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 获取当前 ROS 发行版
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    nav2_bringup_launch = f'/opt/ros/{ros_distro}/share/nav2_bringup/launch/bringup_launch.py'

    # 可选：指定rviz配置文件路径
    rviz_config_file = os.path.join(share_dir, 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params,
            description='Full path to nav2 parameters file'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_launch]),
            launch_arguments={
                'map': map_yaml_file,
                'params_file': nav2_params_file,
                'use_sim_time': use_sim_time
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
