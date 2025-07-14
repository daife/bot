import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('fishbot_navigation2')
    
    # Declare launch arguments
    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Map file path
    map_file = os.path.join(pkg_dir, 'maps', 'fishbot_map.yaml')
    # Nav2 params file path
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
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
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        # Include nav2 bringup launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/bringup_launch.py'
            ]),
            launch_arguments={
                'map': map_yaml_file,
                'params_file': nav2_params_file,
                'use_sim_time': use_sim_time
            }.items(),
        ),
    ])
