#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from arm_control_interfaces.action import MoveArm
from claw_control_interfaces.action import MoveClaw
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import threading
import time
import math
import pygame
import os
import glob

class HandleControlNode(Node):
    """
    手柄控制节点，用于通过游戏手柄控制机器人移动、机械臂和爪子。
    """
    def __init__(self):
        super().__init__('handle_control_node')
        
        # 初始化pygame
        pygame.init()
        pygame.joystick.init()
        
        # 检查是否有手柄连接
        if pygame.joystick.get_count() == 0:
            self.get_logger().error('未检测到游戏手柄，请连接手柄后重启节点')
            return
        
        # 连接第一个手柄
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f'已连接手柄: {self.joystick.get_name()}')
        
        # 创建发布器和动作客户端
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_action_client = ActionClient(self, MoveArm, 'move_arm')
        self.claw_action_client = ActionClient(self, MoveClaw, 'move_claw')
        
        # 图像订阅器
        self.cv_bridge = CvBridge()
        self.latest_image = None
        self.image_subscription = self.create_subscription(
            Image,
            'bottom_camera/image_rect',
            self.image_callback,
            10)
        
        # 控制参数
        self.linear_speed = 1.0  # 线性速度 (m/s)
        self.angular_speed = 1.0  # 角速度 (rad/s)
        
        # 爪子状态 (0: 抓取, 1: 释放)
        self.claw_state = 0
        
        # 机械臂参数
        self.arm_angle = 0.0  # 初始角度为0
        self.arm_min_angle = -20.0  # 最小角度
        self.arm_max_angle = 19.0   # 最大角度
        self.arm_step = 1.0  # 步进值
        self.last_arm_command_time = time.time()
        self.arm_command_interval = 0.2  # 连续按下时的命令间隔(秒)
        
        # 按键状态
        self.button_states = {}
        self.hat_state = (0, 0)
        
        # 图像保存路径
        self.image_save_dir = '/home/HwHiAiUser/ros/datasets/rawimg/'
        # 确保保存目录存在
        os.makedirs(self.image_save_dir, exist_ok=True)
        
        # 欢迎信息
        self.print_usage()
        
        # 创建手柄监听线程
        self.running = True
        self.gamepad_thread = threading.Thread(target=self.read_gamepad_input)
        self.gamepad_thread.daemon = True
        self.gamepad_thread.start()
        
        # 为了保持节点运行，创建一个定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('手柄控制节点已启动')
    
    def print_usage(self):
        """打印使用说明"""
        usage_msg = """
手柄控制说明:
---------------------------
移动控制:
  方向键 上/下/左/右 及其组合: 控制小车平移

旋转控制:
  按钮 3 : 逆时针旋转
  按钮 1 : 顺时针旋转

机械臂控制:
  按钮 0 : 机械臂角度 +1 度
  按钮 4 : 机械臂角度 -1 度
  (长按持续增加/减少，范围 -20 至 +20 度)

爪子控制:
  按钮 7 : 抓取
  按钮 9 : 释放

图像控制:
  按钮 6 : 拍摄并保存当前图像
---------------------------
        """
        print(usage_msg)

    def image_callback(self, msg):
        """图像话题回调函数"""
        try:
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'图像转换错误: {str(e)}')
    
    def timer_callback(self):
        """定时器回调，保持节点运行"""
        pass
    
    def read_gamepad_input(self):
        """读取手柄输入的线程函数"""
        try:
            while self.running:
                # 处理pygame事件
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        self.button_states[event.button] = True
                        self.handle_button_press(event.button)
                    elif event.type == pygame.JOYBUTTONUP:
                        # 检测按钮6释放事件
                        if event.button == 9 and self.button_states.get(event.button, False):
                            self.capture_and_save_image()
                        self.button_states[event.button] = False
                    elif event.type == pygame.JOYHATMOTION:
                        self.hat_state = event.value
                        self.handle_hat_motion()
                
                # 处理长按按钮的情况
                self.handle_held_buttons()
                
                # 处理多个控制的组合
                self.handle_combined_controls()
                
                time.sleep(0.05)  # 适当的循环间隔
        except Exception as e:
            self.get_logger().error(f'手柄输入错误: {str(e)}')
        finally:
            pygame.quit()
            print("\n手柄控制已退出。")
    
    def handle_button_press(self, button):
        """处理按钮按下事件"""
        if button == 7:  # 抓取
            self.claw_state = 0
            self.send_claw_goal(self.claw_state)
        elif button == 6:  # 释放
            self.claw_state = 1
            self.send_claw_goal(self.claw_state)
        elif button == 0 or button == 4:  # 机械臂控制
            self.handle_arm_button(button)
    
    def handle_arm_button(self, button):
        """处理机械臂按钮，改变角度并发送命令"""
        now = time.time()
        if now - self.last_arm_command_time < self.arm_command_interval:
            return
        
        if button == 0:  # 增加角度
            if self.arm_angle < self.arm_max_angle:
                self.arm_angle += self.arm_step
                self.send_arm_goal(self.arm_angle)
                self.last_arm_command_time = now
        
        elif button == 4:  # 减少角度
            if self.arm_angle > self.arm_min_angle:
                self.arm_angle -= self.arm_step
                self.send_arm_goal(self.arm_angle)
                self.last_arm_command_time = now
    
    def handle_held_buttons(self):
        """处理持续按下的按钮"""
        now = time.time()
        if now - self.last_arm_command_time >= self.arm_command_interval:
            # 机械臂连续控制
            if self.button_states.get(0, False):  # 增加角度按钮被按住
                if self.arm_angle < self.arm_max_angle:
                    self.arm_angle += self.arm_step
                    self.send_arm_goal(self.arm_angle)
                    self.last_arm_command_time = now
            
            elif self.button_states.get(4, False):  # 减少角度按钮被按住
                if self.arm_angle > self.arm_min_angle:
                    self.arm_angle -= self.arm_step
                    self.send_arm_goal(self.arm_angle)
                    self.last_arm_command_time = now
    
    def handle_hat_motion(self):
        """处理方向键移动"""
        # 方向键状态已更新到 self.hat_state
        # 在 handle_combined_controls 中处理组合控制
        pass
    
    def handle_combined_controls(self):
        """处理多个控制的组合"""
        # 准备速度命令
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        
        # 处理方向键输入
        x_hat, y_hat = self.hat_state
        
        if x_hat != 0 or y_hat != 0:
            # 对角线移动时进行归一化
            if x_hat != 0 and y_hat != 0:
                # 斜向移动时，使用二分之根号2作为速度
                normalize_factor = 1.0 / math.sqrt(2.0)
                linear_x = y_hat * self.linear_speed * normalize_factor  # y_hat 1 是向上，-1 是向下
                linear_y = -x_hat * self.linear_speed * normalize_factor  # x_hat -1 是向左，1 是向右
            else:
                # 单向移动
                linear_x = y_hat * self.linear_speed  # y_hat 1 是向上，-1 是向下
                linear_y = -x_hat * self.linear_speed  # x_hat -1 是向左，1 是向右
        
        # 处理旋转输入
        if self.button_states.get(3, False):  # 按钮3: 逆时针旋转
            angular_z = self.angular_speed
        if self.button_states.get(1, False):  # 按钮1: 顺时针旋转
            angular_z = -self.angular_speed
        
        # 发送组合的速度命令
        if linear_x != 0 or linear_y != 0 or angular_z != 0:
            self.publish_velocity(linear_x, linear_y, angular_z)
        else:
            # 如果没有任何移动指令，发送停止命令
            self.publish_stop()
    
    def publish_velocity(self, x_vel, y_vel, z_ang):
        """发布速度命令"""
        twist = Twist()
        twist.linear.x = x_vel
        twist.linear.y = y_vel
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = z_ang
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().debug(f'发布速度: x={x_vel}, y={y_vel}, z_ang={z_ang}')
    
    def publish_stop(self):
        """发布停止命令"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().debug('发布停止命令')
    
    def send_arm_goal(self, angle):
        """发送机械臂动作目标"""
        self.get_logger().info(f'发送机械臂角度: {angle}')
        
        # 等待动作服务器可用
        if not self.arm_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('机械臂动作服务器不可用')
            return
        
        # 创建并发送目标
        goal_msg = MoveArm.Goal()
        goal_msg.pose = angle
        
        self.arm_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
    
    def arm_feedback_callback(self, feedback_msg):
        """处理机械臂动作反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'收到机械臂反馈: {str(feedback)}')
    
    def send_claw_goal(self, command):
        """发送爪子动作目标"""
        self.get_logger().info(f'发送爪子命令: {command} ({"释放" if command else "抓取"})')
        
        # 等待动作服务器可用
        if not self.claw_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('爪子动作服务器不可用')
            return
        
        # 创建并发送目标
        goal_msg = MoveClaw.Goal()
        goal_msg.command = command
        
        self.claw_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.claw_feedback_callback
        )
    
    def claw_feedback_callback(self, feedback_msg):
        """处理爪子动作反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'收到爪子反馈: {feedback.status}')
    
    def capture_and_save_image(self):
        """捕获并保存当前图像"""
        if self.latest_image is None:
            self.get_logger().warn('没有可用的图像')
            return
        
        # 获取下一个可用的文件序号
        next_index = self.get_next_file_index()
        filename = f"{next_index:04d}.jpg"
        filepath = os.path.join(self.image_save_dir, filename)
        
        try:
            # 保存图像
            cv2.imwrite(filepath, self.latest_image)
            self.get_logger().info(f'图像已保存: {filepath}')
        except Exception as e:
            self.get_logger().error(f'保存图像错误: {str(e)}')
    
    def get_next_file_index(self):
        """获取下一个可用的文件序号"""
        # 获取现有的jpg文件
        existing_files = glob.glob(os.path.join(self.image_save_dir, '*.jpg'))
        
        if not existing_files:
            return 1
        
        # 提取现有文件的序号
        indices = []
        for file in existing_files:
            basename = os.path.basename(file)
            try:
                index = int(os.path.splitext(basename)[0])
                indices.append(index)
            except ValueError:
                # 如果文件名不是纯数字，忽略
                continue
        
        if indices:
            # 返回最大序号 + 1
            return max(indices) + 1
        else:
            return 1
    
    def __del__(self):
        """析构函数，确保资源释放"""
        self.running = False
        if hasattr(self, 'gamepad_thread') and self.gamepad_thread.is_alive():
            self.gamepad_thread.join(timeout=1.0)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = HandleControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        # 确保停止机器人
        node.publish_stop()
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
