source /home/HwHiAiUser/ros/install/setup.sh
echo "ros2 action send_goal /move_arm arm_control_interfaces/action/MoveArm \"{pose: 0}\""
ros2 run arm_action_rclpy action_arm_01