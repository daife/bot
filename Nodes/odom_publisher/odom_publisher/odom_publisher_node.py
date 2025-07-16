#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
import time
from nav_msgs.msg import Odometry
import math

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # 声明参数
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('serial_port', '/dev/ttyAMA1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_rate', 100.0)
        
        # 获取参数
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # 初始化发布器
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        
        # 初始化串口
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Serial port {self.serial_port} opened successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return
        
        # 数据缓冲区
        self.buffer = bytearray()
        
        # 创建定时器读取串口数据
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.read_serial_data)
        
        self.get_logger().info('Odom Publisher Node started')
    
    def read_serial_data(self):
        """读取并解析串口数据"""
        try:
            # 读取可用数据
            if self.serial_conn.in_waiting > 0:
                data = self.serial_conn.read(self.serial_conn.in_waiting)
                self.buffer.extend(data)
            
            # 查找完整的数据包
            while len(self.buffer) >= 26:  # 数据包长度：1(包头) + 24(6个float) + 1(包尾)
                # 查找包头 0xAA
                header_index = self.buffer.find(0xAA)
                if header_index == -1:
                    # 没有找到包头，清空缓冲区
                    self.buffer.clear()
                    break
                
                # 移除包头之前的数据
                if header_index > 0:
                    self.buffer = self.buffer[header_index:]
                
                # 检查是否有完整的数据包
                if len(self.buffer) < 26:
                    break
                
                # 检查包尾
                if self.buffer[25] == 0x0A:
                    # 找到完整数据包，解析数据
                    self.parse_odom_data(self.buffer[1:25])  # 提取6个float数据
                    # 移除已处理的数据包
                    self.buffer = self.buffer[26:]
                else:
                    # 包尾不匹配，移除当前包头，继续搜索
                    self.buffer = self.buffer[1:]
                    
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {e}')
    
    def parse_odom_data(self, data_bytes):
        """解析里程计数据并发布"""
        try:
            # 解析6个float值
            odom_data = struct.unpack('<6f', data_bytes)  # 小端字节序
            
            # 提取数据
            x_pos = odom_data[0]      # 全局X位置
            y_pos = odom_data[1]      # Y位置
            x_vel = odom_data[2]      # X轴速度
            y_vel = odom_data[3]      # Y轴速度
            angular_vel = odom_data[4] # 角速度
            theta = odom_data[5]      # 当前角度
            
            # 创建时间戳
            current_time = self.get_clock().now()
            
            # 创建四元数
            quat = quaternion_from_euler(0, 0, theta)
            
            # 发布里程计消息
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = self.frame_id
            odom_msg.child_frame_id = self.child_frame_id
            
            # 位置信息
            odom_msg.pose.pose.position.x = x_pos
            odom_msg.pose.pose.position.y = y_pos
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            
            # 速度信息
            odom_msg.twist.twist.linear.x = x_vel
            odom_msg.twist.twist.linear.y = y_vel
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = angular_vel
            
            # 设置协方差矩阵（可根据实际情况调整）
            odom_msg.pose.covariance[0] = 0.1   # x
            odom_msg.pose.covariance[7] = 0.1   # y
            odom_msg.pose.covariance[35] = 0.1  # yaw
            odom_msg.twist.covariance[0] = 0.1  # vx
            odom_msg.twist.covariance[7] = 0.1  # vy
            odom_msg.twist.covariance[35] = 0.1 # vyaw
            
            self.odom_pub.publish(odom_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error parsing odom data: {e}')
    
    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        odom_publisher = OdomPublisher()
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'odom_publisher' in locals():
            odom_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
