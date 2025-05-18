#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from arm_control_interfaces.action import MoveArm
from claw_control_interfaces.action import MoveClaw
import sys
import tty
import termios
import select
import threading
import time


class KeyboardControlNode(Node):
    """
    键盘控制节点，用于通过键盘控制机器人移动、机械臂和爪子。
    """
    def __init__(self):
        super().__init__('keyboard_control_node')
        
        # 创建发布器和动作客户端
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_action_client = ActionClient(self, MoveArm, 'move_arm')
        self.claw_action_client = ActionClient(self, MoveClaw, 'move_claw')
        
        # 控制参数
        self.linear_speed = 0.3  # 线性速度 (m/s)
        self.angular_speed = 1.0  # 角速度 (rad/s)
        
        # 爪子状态 (0: 抓取, 1: 释放)
        self.claw_state = 0
        
        # 欢迎信息
        self.print_usage()
        
        # 创建键盘监听线程
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.read_keyboard_input)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # 为了保持节点运行，创建一个定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('键盘控制节点已启动')
    
    def print_usage(self):
        """打印使用说明"""
        usage_msg = """
键盘控制说明:
---------------------------
移动控制:
  i : 前进
  k : 后退
  j : 左移
  l : 右移
  空格 : 停止

旋转控制:
  a : 逆时针旋转
  d : 顺时针旋转

机械臂控制:
  w : 机械臂位置 -2
  s : 机械臂位置 18

爪子控制:
  回车 : 切换爪子状态 (抓取/释放)

退出:
  q : 退出程序
---------------------------
        """
        print(usage_msg)
    
    def timer_callback(self):
        """定时器回调，保持节点运行"""
        pass
    
    def read_keyboard_input(self):
        """读取键盘输入的线程函数"""
        # 保存终端设置
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            # 设置终端为raw模式
            tty.setraw(sys.stdin.fileno())
            
            while self.running:
                # 检查是否有输入可读取
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    
                    # 处理WASD按键
                    if key == 'w':  # 上 - 机械臂位置 -2
                        self.send_arm_goal(-2.0)
                    elif key == 's':  # 下 - 机械臂位置 18
                        self.send_arm_goal(18.0)
                    elif key == 'a':  # 左 - 逆时针旋转
                        self.publish_angular_velocity(self.angular_speed)
                    elif key == 'd':  # 右 - 顺时针旋转
                        self.publish_angular_velocity(-self.angular_speed)
                    # 处理其他按键
                    elif key == 'i':  # 前进
                        self.publish_linear_velocity(self.linear_speed, 0.0)
                    elif key == ',':  # 后退
                        self.publish_linear_velocity(-self.linear_speed, 0.0)
                    elif key == 'j':  # 左移
                        self.publish_linear_velocity(0.0, self.linear_speed)
                    elif key == 'l':  # 右移
                        self.publish_linear_velocity(0.0, -self.linear_speed)
                    elif key == 'k':  # 空格键，停止
                        self.publish_stop()
                    elif key == '\r':  # 回车键
                        self.toggle_and_send_claw_goal()
                    elif key == 'q':  # 退出
                        self.running = False
                        
                time.sleep(0.01)  # 避免CPU过载
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\n键盘控制已退出。")
    
    def publish_linear_velocity(self, x_vel, y_vel):
        """发布线性速度命令"""
        twist = Twist()
        twist.linear.x = x_vel
        twist.linear.y = y_vel
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().debug(f'发布线性速度: x={x_vel}, y={y_vel}')
    
    def publish_angular_velocity(self, z_vel):
        """发布角速度命令"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = z_vel
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().debug(f'发布角速度: z={z_vel}')
    
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
    
    def send_arm_goal(self, pose):
        """发送机械臂动作目标"""
        self.get_logger().info(f'发送机械臂位置: {pose}')
        
        # 等待动作服务器可用
        if not self.arm_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('机械臂动作服务器不可用')
            return
        
        # 创建并发送目标
        goal_msg = MoveArm.Goal()
        goal_msg.pose = pose
        
        self.arm_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
    
    def arm_feedback_callback(self, feedback_msg):
        """处理机械臂动作反馈"""
        feedback = feedback_msg.feedback
        # 使用更通用的方法记录反馈，而不假设特定的属性名
        self.get_logger().debug(f'收到机械臂反馈: {str(feedback)}')
    
    def toggle_and_send_claw_goal(self):
        """切换爪子状态并发送目标"""
        # 切换状态: 0 (抓取) <-> 1 (释放)
        self.claw_state = 1 - self.claw_state
        
        self.get_logger().info(f'发送爪子命令: {self.claw_state} ({"释放" if self.claw_state else "抓取"})')
        
        # 等待动作服务器可用
        if not self.claw_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('爪子动作服务器不可用')
            return
        
        # 创建并发送目标
        goal_msg = MoveClaw.Goal()
        goal_msg.command = self.claw_state
        
        self.claw_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.claw_feedback_callback
        )
    
    def claw_feedback_callback(self, feedback_msg):
        """处理爪子动作反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'收到爪子反馈: {feedback.status}')
    
    def __del__(self):
        """析构函数，确保终端恢复正常"""
        self.running = False
        if hasattr(self, 'keyboard_thread') and self.keyboard_thread.is_alive():
            self.keyboard_thread.join(timeout=1.0)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = KeyboardControlNode()
    
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
