#!/usr/bin/env python3

import threading
import time
import math
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from arm_control_interfaces.action import MoveArm
from claw_control_interfaces.action import MoveClaw

class CarController:
    """
    小车控制器，负责执行各种控制命令
    """
    def __init__(self, node):
        self.node = node
        
        # 创建发布器和动作客户端
        self.cmd_vel_publisher = node.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_action_client = ActionClient(node, MoveArm, 'move_arm')
        self.claw_action_client = ActionClient(node, MoveClaw, 'move_claw')
        
        # 控制参数（参考handle_control_node）
        self.linear_speed = 0.2  # 线性速度 (m/s)
        self.angular_speed = 0.2  # 角速度 (rad/s)
        
        # 机械臂参数（参考handle_control_node）
        self.arm_angle = 0.0  # 当前角度
        self.arm_min_angle = -20.0  # 最小角度
        self.arm_max_angle = 19.0   # 最大角度
        self.arm_step = 1.0  # 步进值
        
        # 爪子状态 (0: 抓取, 1: 释放)
        self.claw_state = 0
        
        # 当前状态
        self.current_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.is_moving = False
        self.active_commands = set()  # 当前活跃的命令
        self.command_lock = threading.Lock()
        
        self.node.get_logger().info('小车控制器已初始化')
    
    def move(self, direction, duration=1.0, speed=None):
        """移动控制（参考handle_control_node的方向键控制）"""
        if speed is None:
            speed = self.linear_speed
        
        with self.command_lock:
            command_id = f"move_{direction}_{time.time()}"
            self.active_commands.add(command_id)
        
        try:
            # 计算移动向量（参考handle_control_node的逻辑）
            linear_x, linear_y = self._direction_to_velocity(direction, speed)
            
            # 开始移动
            self._publish_velocity(linear_x, linear_y, 0.0)
            self.node.get_logger().info(f'开始移动: {direction}, 速度: {speed}, 持续时间: {duration}s')
            
            # 持续指定时间
            time.sleep(duration)
            
            # 检查命令是否还活跃（可能被其他命令取消）
            with self.command_lock:
                if command_id in self.active_commands:
                    self._publish_velocity(0.0, 0.0, 0.0)
                    self.active_commands.discard(command_id)
            
            return f'移动完成: {direction}'
            
        except Exception as e:
            with self.command_lock:
                self.active_commands.discard(command_id)
            raise e
    
    def rotate(self, direction, angle=90.0):
        """旋转控制（参考handle_control_node的旋转按钮控制）"""
        with self.command_lock:
            command_id = f"rotate_{direction}_{time.time()}"
            self.active_commands.add(command_id)
        
        try:
            # 计算旋转速度（参考handle_control_node）
            if direction == 'left':  # 逆时针旋转
                angular_z = self.angular_speed
            elif direction == 'right':  # 顺时针旋转
                angular_z = -self.angular_speed
            else:
                angular_z = 0.0
            
            # 计算旋转时间
            angle_rad = math.radians(angle)
            duration = abs(angle_rad / abs(angular_z)) if angular_z != 0 else 0
            
            # 开始旋转
            self._publish_velocity(0.0, 0.0, angular_z)
            self.node.get_logger().info(f'开始旋转: {direction}, 角度: {angle}度, 持续时间: {duration:.2f}s')
            
            # 持续指定时间
            if duration > 0:
                time.sleep(duration)
            
            # 检查命令是否还活跃
            with self.command_lock:
                if command_id in self.active_commands:
                    self._publish_velocity(0.0, 0.0, 0.0)
                    self.active_commands.discard(command_id)
            
            return f'旋转完成: {direction} {angle}度'
            
        except Exception as e:
            with self.command_lock:
                self.active_commands.discard(command_id)
            raise e
    
    def control_arm(self, target_angle):
        """机械臂控制（参考handle_control_node的机械臂控制）"""
        # 角度范围检查
        target_angle = max(self.arm_min_angle, min(self.arm_max_angle, target_angle))
        
        self.node.get_logger().info(f'控制机械臂角度: {target_angle}')
        
        # 等待动作服务器可用
        if not self.arm_action_client.wait_for_server(timeout_sec=1.0):
            raise Exception('机械臂动作服务器不可用')
        
        # 创建并发送目标
        goal_msg = MoveArm.Goal()
        goal_msg.pose = target_angle
        
        # 发送异步目标
        future = self.arm_action_client.send_goal_async(goal_msg)
        
        # 更新当前角度
        self.arm_angle = target_angle
        
        return f'机械臂角度设置为: {target_angle}度'
    
    def control_claw(self, action):
        """爪子控制（参考handle_control_node的爪子控制）"""
        # 参考handle_control_node: 0: 抓取, 1: 释放
        if action == 'grasp':
            command = 0
            self.claw_state = 0
        elif action == 'release':
            command = 1
            self.claw_state = 1
        else:
            raise Exception(f'未知的爪子动作: {action}')
        
        self.node.get_logger().info(f'控制爪子: {action} (命令: {command})')
        
        # 等待动作服务器可用
        if not self.claw_action_client.wait_for_server(timeout_sec=1.0):
            raise Exception('爪子动作服务器不可用')
        
        # 创建并发送目标
        goal_msg = MoveClaw.Goal()
        goal_msg.command = command
        
        # 发送异步目标
        future = self.claw_action_client.send_goal_async(goal_msg)
        
        return f'爪子{action}命令已发送'
    
    def stop_all(self):
        """停止所有动作"""
        with self.command_lock:
            self.active_commands.clear()
        
        # 停止移动
        self._publish_velocity(0.0, 0.0, 0.0)
        self.node.get_logger().info('已停止所有动作')
        return '已停止所有动作'
    
    def _direction_to_velocity(self, direction, speed):
        """将方向转换为速度向量（参考handle_control_node的逻辑）"""
        # 参考handle_control_node中的坐标系统和移动逻辑
        direction_map = {
            'forward': (speed, 0.0),      # 前进: linear_x = speed
            'backward': (-speed, 0.0),    # 后退: linear_x = -speed  
            'left': (0.0, speed),         # 左移: linear_y = speed
            'right': (0.0, -speed),       # 右移: linear_y = -speed
            'stop': (0.0, 0.0)
        }
        
        # 对角线移动（参考handle_control_node的归一化处理）
        if direction in ['forward_left', 'forward_right', 'backward_left', 'backward_right']:
            normalize_factor = 1.0 / math.sqrt(2.0)  # 参考handle_control_node
            
            if direction == 'forward_left':
                return (speed * normalize_factor, speed * normalize_factor)
            elif direction == 'forward_right':
                return (speed * normalize_factor, -speed * normalize_factor)
            elif direction == 'backward_left':
                return (-speed * normalize_factor, speed * normalize_factor)
            elif direction == 'backward_right':
                return (-speed * normalize_factor, -speed * normalize_factor)
        
        return direction_map.get(direction, (0.0, 0.0))
    
    def _publish_velocity(self, x_vel, y_vel, z_ang):
        """发布速度命令（参考handle_control_node）"""
        twist = Twist()
        twist.linear.x = x_vel
        twist.linear.y = y_vel
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = z_ang
        
        self.cmd_vel_publisher.publish(twist)
        
        # 更新当前速度状态
        self.current_velocity = {'x': x_vel, 'y': y_vel, 'z': z_ang}
        self.is_moving = (x_vel != 0.0 or y_vel != 0.0 or z_ang != 0.0)
        
        # 调试信息（参考handle_control_node）
        self.node.get_logger().debug(f'发布速度: x={x_vel}, y={y_vel}, z_ang={z_ang}')