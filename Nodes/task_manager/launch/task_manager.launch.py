from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 启动节点
    task_manager_node = Node(
        package='task_manager',
        executable='task_manager_node',
        name='task_manager_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        task_manager_node,
    ])
