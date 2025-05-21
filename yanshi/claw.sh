source /home/HwHiAiUser/ros/install/setup.sh
echo "ros2 action send_goal /move_claw claw_control_interfaces/action/MoveClaw \"{command: 0}\""
ros2 run claw_action_rclpy action_claw_01