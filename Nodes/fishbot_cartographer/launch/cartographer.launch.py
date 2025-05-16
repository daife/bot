import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    # 定位到功能包的地址
    pkg_share = FindPackageShare(package='fishbot_cartographer').find('fishbot_cartographer')
    
    #=====================运行节点需要的配置=======================================================================
    # 实机运行，确保use_sim_time为false
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='fishbot_2d.lua')
    # 是否启动RViz
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    rviz_config_dir = os.path.join(pkg_share, 'config')+"/cartographer.rviz"
    print(f"rviz config in {rviz_config_dir}")

    # 声明launch参数
    declare_launch_rviz_cmd = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to start RViz')
    
    #=====================声明节点=================================
    # 注意：不需要静态TF发布节点，因为robot_state_publisher已经根据URDF发布所有静态变换关系
    # 从sam_bot_description包中的display.launch.py可以看到，robot_state_publisher正在运行
    # 它会自动处理激光雷达与base_link之间的坐标变换
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(launch_rviz))

    # 麦克纳姆轮的特殊配置已经在fishbot_2d.lua文件中完成，包括：
    # 1. 使用里程计数据(use_odometry = true)
    # 2. 高度依赖IMU补偿侧向滑动(imu_sampling_ratio = 1.0)
    # 3. 启用实时回环检测(use_online_correlative_scan_matching = true)
    # 4. 优化各类权重参数
    
    #===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(declare_launch_rviz_cmd)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld