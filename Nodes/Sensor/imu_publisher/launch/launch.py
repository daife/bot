import launch
import launch_ros

def generate_launch_description():
    imu_node = launch_ros.actions.Node(
        package='imu_publisher',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[
            {'device_serial_id': '1a86_USB_Serial'},
            {'baud_rate': 921600},
            {'imu_frame_id': 'imu_link'},
            {'publish_rate': 50.0}
        ]
    )

    return launch.LaunchDescription([
        imu_node
    ])
