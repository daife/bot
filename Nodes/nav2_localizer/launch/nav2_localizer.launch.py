from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'map_config',
            default_value='/home/HwHiAiUser/ros/src/Nodes/nav2_localizer/config/map_server.yaml',
            description='Map server config path'
        ),
        DeclareLaunchArgument(
            'amcl_config',
            default_value='/home/HwHiAiUser/ros/src/Nodes/nav2_localizer/config/amcl_config.yaml',
            description='AMCL config path'
        ),
        # 地图服务器
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[LaunchConfiguration('map_config')]
        ),
        # AMCL节点
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[LaunchConfiguration('amcl_config')]
        ),
        # 生命周期管理器
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        )
    ])