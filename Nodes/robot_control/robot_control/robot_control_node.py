#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point, Twist
from robot_control_interfaces.action import ManipulateObject
from object_detection_interfaces.msg import DetectionResult
from sensor_msgs.msg import JointState
import time
import threading
import math

class RobotControlNode(Node):
    """
    机器人控制节点，负责执行抓取和放置操作
    """
    
    def __init__(self):
        super().__init__('robot_control_node')
        self.get_logger().info('Robot Control Node 启动中...')
        
        # 创建回调组，允许并发执行回调
        self.callback_group = ReentrantCallbackGroup()
        
        # 最大重试次数
        self.max_retries = 3
        
        # 状态常量
        self.STATUS_INIT = 0        # 初始化
        self.STATUS_ADJUSTING = 1   # 调整中
        self.STATUS_GRASPING = 2    # 抓握中
        self.STATUS_FAILED = 3      # 抓握失败
        self.STATUS_SUCCEEDED = 4   # 抓握成功
        
        # 机械臂和抓手控制相关变量
        self.current_joint_positions = {}
        self.object_position = None
        self.is_adjusting = False
        self.is_grasping = False
        
        # 线程锁，用于线程安全访问共享数据
        self.lock = threading.RLock()
        
        # 创建发布者
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )
        
        # 订阅者
        self.detection_sub = self.create_subscription(
            DetectionResult,
            '/object_detection/result',
            self.detection_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 创建动作服务器
        self._action_server = ActionServer(
            self,
            ManipulateObject,
            'manipulate_object',
            self.execute_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Robot Control Node 初始化完成')
    
    def joint_state_callback(self, msg):
        """处理关节状态信息"""
        with self.lock:
            for i, name in enumerate(msg.name):
                self.current_joint_positions[name] = msg.position[i]
    
    def detection_callback(self, msg):
        """处理物体检测结果"""
        if msg.detected_objects > 0:
            self.object_position = msg.object_position
            self.get_logger().debug(f'检测到物体，位置: [{self.object_position.x:.2f}, {self.object_position.y:.2f}, {self.object_position.z:.2f}]')
    
    def execute_callback(self, goal_handle):
        """执行动作回调函数"""
        request = goal_handle.request
        operation = request.operation
        feedback_msg = ManipulateObject.Feedback()
        
        if operation == ManipulateObject.Goal.OPERATION_PICK:
            # 抓取操作
            return self.execute_pick(goal_handle, request, feedback_msg)
        elif operation == ManipulateObject.Goal.OPERATION_DROP:
            # 放置操作
            return self.execute_drop(goal_handle, request, feedback_msg)
        else:
            # 无效操作
            self.get_logger().error(f'无效的操作类型: {operation}')
            goal_handle.abort()
            result = ManipulateObject.Result()
            result.success = False
            result.message = f"无效的操作类型: {operation}"
            return result
    
    def execute_pick(self, goal_handle, request, feedback_msg):
        """执行抓取操作"""
        self.get_logger().info('开始执行抓取操作')
        
        # 初始化反馈
        feedback_msg.status = self.STATUS_INIT
        feedback_msg.retry_count = 0
        goal_handle.publish_feedback(feedback_msg)
        
        # 获取目标位置
        target_position = request.object_position
        if target_position.x == 0 and target_position.y == 0 and target_position.z == 0:
            # 如果目标位置未指定，使用检测到的物体位置
            if self.object_position is None:
                self.get_logger().error('未指定目标位置且未检测到物体')
                goal_handle.abort()
                result = ManipulateObject.Result()
                result.success = False
                result.message = "未指定目标位置且未检测到物体"
                return result
            target_position = self.object_position
        
        # 重试逻辑
        retry_count = 0
        max_retries = self.max_retries
        success = False
        
        while retry_count <= max_retries and not success:
            if retry_count > 0:
                self.get_logger().info(f'抓取失败，正在重试 ({retry_count}/{max_retries})')
                feedback_msg.status = self.STATUS_FAILED
                feedback_msg.retry_count = retry_count
                goal_handle.publish_feedback(feedback_msg)
                
                # 重试前稍微调整位置
                target_position.x += (random.random() - 0.5) * 0.05
                target_position.y += (random.random() - 0.5) * 0.05
            
            # 1. 调整阶段
            feedback_msg.status = self.STATUS_ADJUSTING
            goal_handle.publish_feedback(feedback_msg)
            
            # 机械臂靠近物体
            self.approach_object(target_position)
            
            # 检查是否被取消
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('抓取操作已取消')
                result = ManipulateObject.Result()
                result.success = False
                result.message = "操作被取消"
                return result
            
            # 2. 抓握阶段
            feedback_msg.status = self.STATUS_GRASPING
            goal_handle.publish_feedback(feedback_msg)
            
            # 执行抓取
            success = self.grasp_object()
            
            if success:
                feedback_msg.status = self.STATUS_SUCCEEDED
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info('抓取成功')
            else:
                retry_count += 1
        
        # 完成操作并返回结果
        result = ManipulateObject.Result()
        result.success = success
        
        if success:
            result.message = "抓取成功"
            goal_handle.succeed()
        else:
            result.message = f"抓取失败，已重试{retry_count}次"
            goal_handle.abort()
        
        return result
    
    def execute_drop(self, goal_handle, request, feedback_msg):
        """执行放置操作"""
        self.get_logger().info('开始执行放置操作')
        
        # 初始化反馈
        feedback_msg.status = self.STATUS_INIT
        feedback_msg.retry_count = 0
        goal_handle.publish_feedback(feedback_msg)
        
        # 放置操作通常不需要重试，直接执行
        feedback_msg.status = self.STATUS_ADJUSTING
        goal_handle.publish_feedback(feedback_msg)
        
        # 移动到放置位置
        self.move_to_drop_position()
        
        # 检查是否被取消
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('放置操作已取消')
            result = ManipulateObject.Result()
            result.success = False
            result.message = "操作被取消"
            return result
        
        # 执行放置
        feedback_msg.status = self.STATUS_GRASPING  # 虽然是放置，但仍使用相同的状态
        goal_handle.publish_feedback(feedback_msg)
        
        success = self.release_object()
        
        if success:
            feedback_msg.status = self.STATUS_SUCCEEDED
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('放置成功')
        else:
            feedback_msg.status = self.STATUS_FAILED
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().error('放置失败')
        
        # 完成操作并返回结果
        result = ManipulateObject.Result()
        result.success = success
        result.message = "放置成功" if success else "放置失败"
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        return result
    
    def approach_object(self, target_position):
        """机械臂靠近物体"""
        self.get_logger().info(f'机械臂靠近物体，目标位置: [{target_position.x:.2f}, {target_position.y:.2f}, {target_position.z:.2f}]')
        
        # 这里实现机械臂运动逻辑
        # 例如：计算逆运动学，控制关节运动等
        
        # 模拟机械臂运动
        time.sleep(2.0)
        return True
    
    def grasp_object(self):
        """抓取物体"""
        self.get_logger().info('执行抓取')
        
        # 这里实现抓手控制逻辑
        # 例如：闭合抓手，检测抓取力反馈等
        
        # 模拟抓取过程，80%成功率
        time.sleep(1.0)
        import random
        success = random.random() < 0.8
        
        if success:
            self.get_logger().info('抓取成功')
        else:
            self.get_logger().info('抓取失败')
        
        return success
    
    def move_to_drop_position(self):
        """移动到放置位置"""
        self.get_logger().info('移动到放置位置')
        
        # 这里实现机械臂移动到放置位置的逻辑
        
        # 模拟移动过程
        time.sleep(2.0)
        return True
    
    def release_object(self):
        """放下物体"""
        self.get_logger().info('释放物体')
        
        # 这里实现抓手释放的逻辑
        # 例如：张开抓手
        
        # 模拟放置过程，通常放置成功率很高
        time.sleep(1.0)
        success = True
        
        return success

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
