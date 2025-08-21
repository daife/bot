#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    创建启动两个摄像头节点的启动描述。
    可以通过启动参数配置摄像头属性。
    """
    # 获取包路径
    pkg_dir = get_package_share_directory('dual_camera_pkg')
    
    # 配置文件默认路径
    top_calibration_file = os.path.join(pkg_dir, 'config', 'top_camera.yaml')
    bottom_calibration_file = os.path.join(pkg_dir, 'config', 'bottom_camera.yaml')
    
    # 声明校准文件路径的启动参数
    top_calibration_arg = DeclareLaunchArgument(
        'top_calibration_file',
        default_value=top_calibration_file,
        description='上方摄像头校准文件路径'
    )
    
    bottom_calibration_arg = DeclareLaunchArgument(
        'bottom_calibration_file',
        default_value=bottom_calibration_file,
        description='下方摄像头校准文件路径'
    )
    
    # 声明上方摄像头的启动参数
    top_camera_device_arg = DeclareLaunchArgument(
        'top_camera_device',
        default_value='0',
        description='上方摄像头设备ID'
    )
    
    top_frame_width_arg = DeclareLaunchArgument(
        'top_frame_width',
        default_value='320',
        description='上方摄像头图像宽度'
    )
    
    top_frame_height_arg = DeclareLaunchArgument(
        'top_frame_height',
        default_value='240',
        description='上方摄像头图像高度'
    )
    
    top_publish_rate_arg = DeclareLaunchArgument(
        'top_publish_rate',
        default_value='30.0',
        description='上方摄像头发布频率'
    )
    
    top_camera_fps_arg = DeclareLaunchArgument(
        'top_camera_fps',
        default_value='30.0',
        description='上方摄像头帧率'
    )
    
    top_camera_format_arg = DeclareLaunchArgument(
        'top_camera_format',
        default_value='YUYV',
        description='上方摄像头格式'
    )
    
    # 声明下方摄像头的启动参数
    bottom_camera_device_arg = DeclareLaunchArgument(
        'bottom_camera_device',
        default_value='2',
        description='下方摄像头设备ID'
    )
    
    bottom_frame_width_arg = DeclareLaunchArgument(
        'bottom_frame_width',
        default_value='640',
        description='下方摄像头图像宽度'
    )
    
    bottom_frame_height_arg = DeclareLaunchArgument(
        'bottom_frame_height',
        default_value='480',
        description='下方摄像头图像高度'
    )
    
    bottom_publish_rate_arg = DeclareLaunchArgument(
        'bottom_publish_rate',
        default_value='60.0',
        description='下方摄像头发布频率'
    )
    
    bottom_camera_fps_arg = DeclareLaunchArgument(
        'bottom_camera_fps',
        default_value='60.0',
        description='下方摄像头帧率'
    )
    
    bottom_camera_format_arg = DeclareLaunchArgument(
        'bottom_camera_format',
        default_value='YUYV',
        description='下方摄像头格式'
    )
    
    # 创建上方摄像头节点
    top_camera_node = Node(
        package='dual_camera_pkg',
        executable='top_camera_node',
        name='top_camera_publisher',
        parameters=[{
            'camera_device': LaunchConfiguration('top_camera_device'),
            'frame_width': LaunchConfiguration('top_frame_width'),
            'frame_height': LaunchConfiguration('top_frame_height'),
            'publish_rate': LaunchConfiguration('top_publish_rate'),
            'camera_fps': LaunchConfiguration('top_camera_fps'),
            'camera_format': LaunchConfiguration('top_camera_format'),
            'calibration_file': LaunchConfiguration('top_calibration_file'),
        }],
        output='screen'
    )
    
    # 创建下方摄像头节点
    bottom_camera_node = Node(
        package='dual_camera_pkg',
        executable='bottom_camera_node',
        name='bottom_camera_publisher',
        parameters=[{
            'camera_device': LaunchConfiguration('bottom_camera_device'),
            'frame_width': LaunchConfiguration('bottom_frame_width'),
            'frame_height': LaunchConfiguration('bottom_frame_height'),
            'publish_rate': LaunchConfiguration('bottom_publish_rate'),
            'camera_fps': LaunchConfiguration('bottom_camera_fps'),
            'camera_format': LaunchConfiguration('bottom_camera_format'),
            'calibration_file': LaunchConfiguration('bottom_calibration_file'),
        }],
        output='screen'
    )
    
    # 返回启动描述 - 只包含相机节点
    return LaunchDescription([
        # 校准文件参数
        top_calibration_arg,
        bottom_calibration_arg,
        # 上方摄像头参数
        top_camera_device_arg,
        top_frame_width_arg,
        top_frame_height_arg,
        top_publish_rate_arg,
        top_camera_fps_arg,
        top_camera_format_arg,
        # 下方摄像头参数
        bottom_camera_device_arg,
        bottom_frame_width_arg,
        bottom_frame_height_arg,
        bottom_publish_rate_arg,
        bottom_camera_fps_arg,
        bottom_camera_format_arg,
        # 节点 - 只包含相机节点
        top_camera_node,
        bottom_camera_node
    ])