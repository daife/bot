from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """启动物体定位器节点"""
    return LaunchDescription([
        Node(
            package='object_localizer',
            executable='object_localizer_node',
            name='object_localizer_node',
            output='screen',
            parameters=[{
                'confidence_threshold': 0.65,  # 三角测量可信度阈值
                'feature_match_ratio': 0.7,    # 特征匹配比率阈值
                'max_feature_points': 100,     # 最大特征点数量
                'publish_debug_images': True,  # 是否发布调试图像
            }]
        )
    ])
