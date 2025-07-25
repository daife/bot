import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def include_launch(pkg, launch_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', launch_file)
        )
    )

def generate_launch_description():
    return LaunchDescription([
        include_launch('sam_bot_description', 'simple_display.launch.py'),
        include_launch('chassis_controlerAndpublisher', 'chassis_controlerAndpublisher.launch.py'),
        include_launch('imu_publisher', 'imu_publisher.launch.py'),
        include_launch('sensor_fusion_pkg', 'sensor_fusion.launch.py'),
        include_launch('oradar_lidar', 'ms200_scan.launch.py'),
        include_launch('nav2_localizer', 'nav2_localizer.launch.py'),
        include_launch('collision_preventor', 'collision_preventor.launch.py'),
        include_launch('paper_localizer', 'paper_localizer.launch.py'),
        # include_launch('camera_tracking', 'camera_tracking.launch.py'),
        TimerAction(
            period=10.0,
            actions=[
                include_launch('control_center', 'control_center.launch.py'),
            ]
        ),
    ])
