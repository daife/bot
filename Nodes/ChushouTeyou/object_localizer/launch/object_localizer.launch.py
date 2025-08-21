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
                # 基本参数
                'confidence_threshold': 0.65,     # 三角测量置信度阈值
                'show_debug_visualization': True, # 是否显示可视化窗口 (替代旧的publish_debug_images)
                
                # 模板匹配参数
                'template_match_method': 5,       # 5=TM_CCOEFF_NORMED，推荐的模板匹配方法
                'template_match_threshold': 0.5,  # 模板匹配置信度阈值
                'template_search_scale': 1.0,     # 搜索区域比例
                'template_scale_range': 0.2,      # 模板缩放范围，例如0.2表示尝试0.8-1.2倍尺寸
                'template_scale_steps': 3,        # 模板缩放步数
                
                # 位置约束参数
                'vertical_position_check': True,  # 是否检查垂直位置约束
                
                # 颜色相似度参数
                'color_similarity_check': True,   # 是否检查颜色相似度
                'color_similarity_threshold': 0.5, # 颜色相似度阈值
                'color_sample_size': 7,           # 颜色采样区域大小
                'top_match_count': 3,             # 考虑的匹配结果数量
            }]
        )
    ])
