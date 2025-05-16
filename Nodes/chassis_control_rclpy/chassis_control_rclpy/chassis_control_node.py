#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import signal

from chassis_control_rclpy.chassis import ChassisController

class ChassisControlNode(Node):
    """控制麦轮底盘的ROS2节点。"""
    
    def __init__(self):
        super().__init__('chassis_control_node')
        
        # 声明并获取串口设备参数，默认为 ttyAMA2
        self.declare_parameter('serial_device', '/dev/ttyAMA2')
        serial_device = self.get_parameter('serial_device').get_parameter_value().string_value
        
        # 初始化底盘控制器，使用指定的串口设备
        self.chassis = ChassisController(device=serial_device)
        
        # 创建cmd_vel的订阅
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info(f'麦轮底盘控制节点已启动，使用串口: {serial_device}')
        
        # 参数,限速
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_angular_speed', 1.0)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        
        # 看门狗计时器，如果长时间没有收到命令则停止
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.5, self.watchdog_callback)
        
    def cmd_vel_callback(self, msg):
        """
        cmd_vel消息的回调函数。
        
        参数:
            msg: 包含速度命令的Twist消息
        """
        # 更新最后命令时间
        self.last_cmd_time = self.get_clock().now()
        
        # 提取linear.x（前进/后退）、linear.y（左右平移）和angular.z（旋转）
        linear_x = msg.linear.x
        linear_y = msg.linear.y  # 添加Y方向速度支持
        angular_z = msg.angular.z
        
        # 应用速度限制
        linear_x = max(min(linear_x, self.max_linear_speed), -self.max_linear_speed)
        linear_y = max(min(linear_y, self.max_linear_speed), -self.max_linear_speed)  # 限制Y方向速度
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)
        
        self.get_logger().debug(f'收到cmd_vel - linear_x: {linear_x}, linear_y: {linear_y}, angular_z: {angular_z}')
        
        # 发送速度命令到底盘控制器
        try:
            self.chassis.set_velocity(linear_x, linear_y, angular_z)
        except Exception as e:
            self.get_logger().error(f'发送速度命令失败: {e}')
    
    def watchdog_callback(self):
        """
        看门狗计时器回调，如果一段时间内没有收到命令则停止机器人。
        """
        current_time = self.get_clock().now()
        if (current_time - self.last_cmd_time).nanoseconds > 1000000000:  # 1秒
            # 1秒内没有收到命令，停止机器人
            try:
                self.chassis.stop()
                self.get_logger().debug('看门狗触发: 停止机器人')
            except Exception as e:
                self.get_logger().error(f'停止机器人失败: {e}')
    
    def cleanup(self):
        """关闭前清理资源。"""
        self.get_logger().info('正在关闭，停止底盘...')
        try:
            self.chassis.stop()
            self.chassis.close()
        except Exception as e:
            self.get_logger().error(f'清理过程中出错: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    chassis_control = ChassisControlNode()
    
    # 优雅地处理关闭信号
    def signal_handler(sig, frame):
        chassis_control.get_logger().info('收到关闭信号')
        chassis_control.cleanup()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(chassis_control)
    except Exception as e:
        chassis_control.get_logger().error(f'发生错误: {e}')
    finally:
        chassis_control.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()