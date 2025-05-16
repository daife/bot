from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock if true')

    # Get the path to the EKF configuration file
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('sensor_fusion_pkg'), 'config', 'ekf_config.yaml'])

    # Create and return the launch description with the robot_localization node
    return LaunchDescription([
        use_sim_time_arg,
        
        # Launch the robot_localization node with EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # If your topics have different names, remap them here
                # Example: ('/input/odom', '/wheel_odom'),
                # Example: ('/input/imu', '/imu/data'),
            ]
        ),
    ])