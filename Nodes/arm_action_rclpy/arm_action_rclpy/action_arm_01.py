#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from arm_control_interfaces.action import MoveArm
from arm_action_rclpy.arm import Arm
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState

class ActionArm01(Node):
    """Arm端Action服务"""

    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")

        self.arm_ = Arm()

        # Create JointState publisher for RViz visualization
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            'joint_states', 
            10
        )
        
        self.publish_initial_joints()  # 启动时发布一次所有关节
        # Create timer to publish joint states at regular intervals
        self.timer = self.create_timer(0.05, self.publish_joint_state)
        
        self.action_server_ = ActionServer(
            self, MoveArm, 'move_arm', self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def publish_initial_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'rear_left_wheel_joint', 'rear_right_wheel_joint',
            'arm_joint'
        ]
        msg.position = [0.0, 0.0, 0.0, 0.0, self.arm_.get_current_pose() * 3.14159 / 180.0]
        self.joint_state_publisher.publish(msg)

    def publish_joint_state(self):
        """Publish joint states for both arm and wheels"""
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            # 同时发布机械臂和四个轮子的关节信息
            msg.name = [
                'front_left_wheel_joint', 'front_right_wheel_joint',
                'rear_left_wheel_joint', 'rear_right_wheel_joint',
                'arm_joint'
            ]
            # 轮子位置为0，机械臂位置为当前角度（转换为弧度）
            msg.position = [
                0.0, 0.0, 0.0, 0.0,
                self.arm_.get_current_pose() * 3.14159 / 180.0
            ]
            self.joint_state_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing joint state: {e}")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """执行回调函数,若采用默认handle_goal函数则会自动调用"""
        self.get_logger().info('移动机械臂执行')
        feedback_msg = MoveArm.Feedback()
        self.arm_.set_goal(goal_handle.request.pose)
        self.get_logger().info(f"目标角度: {goal_handle.request.pose}")

        try:
            loop_count = 0
            STATUS_MOVING = 3
            STATUS_STOP = 4
            while rclpy.ok():
                loop_count += 1
                # 检查是否被取消
                if goal_handle.is_cancel_requested:
                    self.arm_.cancel()
                    result = MoveArm.Result()
                    result.pose = self.arm_.get_current_pose()
                    self.get_logger().info(f"动作被中断，已停止，当前角度: {result.pose}")
                    return result
                # feedback
                feedback_msg.pose = self.arm_.get_current_pose()
                # 判断是否到达目标
                if abs(self.arm_.goal_pose - feedback_msg.pose) < 1e-2:
                    feedback_msg.status = STATUS_STOP
                    goal_handle.publish_feedback(feedback_msg)
                    break
                else:
                    feedback_msg.status = STATUS_MOVING
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"execute_callback异常: {e}")

        goal_handle.succeed()
        result = MoveArm.Result()
        result.pose = self.arm_.get_current_pose()
        self.get_logger().info(f"动作完成，最终角度: {result.pose}")
        return result
        
def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    action_arm_01 = ActionArm01("action_arm_01")
    executor = MultiThreadedExecutor()
    executor.add_node(action_arm_01)
    executor.spin()