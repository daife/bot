from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """启动纸条定位节点"""
    
    # 获取包路径
    pkg_dir = get_package_share_directory('paper_localizer')
    
    # 声明启动参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_dir, 'models', '/home/HwHiAiUser/yolo_test/yolo11n-seg-self-123-nodrop-150epo-1056.om'),
        description='分割模型路径'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='发布频率 (Hz)'
    )
    
    # 创建节点
    paper_localizer_node = Node(
        package='paper_localizer',
        executable='paper_localizer_node',
        name='paper_localizer',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )

    paper_center_kalman_node = Node(
        package='paper_localizer',
        executable='paper_center_kalman_node',
        name='paper_center_kalman',
        output='screen'
    )
    
    return LaunchDescription([
        model_path_arg,
        publish_rate_arg,
        paper_localizer_node,
        paper_center_kalman_node
    ])
