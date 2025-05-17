#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Point
# from tf2_ros import TransformBroadcaster
import math
import time
import wiringpi
import sys
import threading
import collections
import struct
import numpy as np  # Add NumPy for proper array handling

class OdomImuPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_imu_publisher')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyAMA1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        # 获取参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 创建循环缓冲区和线程同步对象
        self.buffer = collections.deque(maxlen=8192)  # 循环缓冲区，存储最多8192个字节
        self.buffer_lock = threading.Lock()  # 用于线程安全访问缓冲区
        self.running = False  # 控制后台线程运行
        
        # 创建发布器
        self.odom_publisher = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        
        # 初始化串口
        self.serial_fd = -1
        if self.serial_port is not None:
            try:
                wiringpi.wiringPiSetup()
                self.serial_fd = wiringpi.serialOpen(self.serial_port, self.baud_rate)
                if self.serial_fd < 0:
                    self.get_logger().error(f"无法打开指定串口设备: {self.serial_port}")
                    rclpy.shutdown()
                    sys.exit(-1)
                else:
                    self.get_logger().info(f"已连接到 {self.serial_port}，波特率 {self.baud_rate}")
            except Exception as e:
                self.get_logger().error(f"串口打开失败: {e}")
                rclpy.shutdown()
                sys.exit(-1)
        else:
            self.get_logger().error("未指定串口设备")
            rclpy.shutdown()
            sys.exit(-1)
        
        # 启动后台线程，持续读取串口数据
        self.running = True
        self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader_thread.start()
            
        # 创建定时器，定时处理和发布数据
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        self.get_logger().info('OdomImuPublisherNode 初始化完成')
    
    def _serial_reader(self):
        """后台线程，持续从串口读取数据并存入缓冲区"""
        while self.running and self.serial_fd >= 0:
            try:
                if wiringpi.serialDataAvail(self.serial_fd) > 0:
                    byte_value = wiringpi.serialGetchar(self.serial_fd)
                    with self.buffer_lock:
                        self.buffer.append(byte_value)
                else:
                    time.sleep(0.001)  # 短暂休眠，避免CPU占用过高
            except Exception as e:
                self.get_logger().error(f"串口读取错误: {e}")
                time.sleep(0.1)  # 错误后短暂休眠
    
    def _get_latest_message(self):
        """从缓冲区获取最新的完整消息（93字节，包含23个float和一个换行符）"""
        with self.buffer_lock:
            buffer_list = list(self.buffer)
            buffer_len = len(buffer_list)
            
            # 查找最后一个换行符
            last_newline_index = -1
            for i in range(buffer_len - 1, -1, -1):
                if buffer_list[i] == 10:  # ASCII for '\n'
                    last_newline_index = i
                    break
            
            if last_newline_index == -1:
                return None  # 没有找到换行符
            
            # 检查是否有足够的数据（93字节）
            if last_newline_index < 92:
                return None  # 数据不足93字节
            
            # 提取最新的完整消息
            start_index = last_newline_index - 92
            message_bytes = bytes(buffer_list[start_index:last_newline_index+1])
            
            return message_bytes
    
    def timer_callback(self):
        """定时器回调函数，从缓冲区读取最新数据并发布消息"""
        try:
            message_bytes = self._get_latest_message()
            if message_bytes is not None:
                self.process_data(message_bytes)
        except Exception as e:
            self.get_logger().error(f"处理数据出错: {e}")
    
    def process_data(self, message_bytes):
        """处理二进制格式的数据（92个字节的float数据 + 1个换行符）"""
        try:
            # 确保数据长度为93字节（23个float值 + 换行符）
            if len(message_bytes) != 93:
                self.get_logger().warning(f"收到不完整数据，长度为 {len(message_bytes)}，应为 93")
                return
            
            # 去掉末尾的换行符，解析23个float值
            float_data = message_bytes[:-1]  # 去掉末尾的\n
            
            if len(float_data) != 92:  # 23 floats * 4 bytes = 92
                self.get_logger().warning(f"浮点数据长度不正确: {len(float_data)}，应为 92")
                return
            
            # 解析23个float值
            floats = struct.unpack('<23f', float_data)
            
            # 提取里程计数据
            odom_x, odom_y, odom_z = floats[0], floats[1], floats[2]
            odom_vx, odom_vy, odom_vz = floats[3], floats[4], floats[5]
            odom_wx, odom_wy, odom_wz = floats[6], floats[7], floats[8]
            odom_qx, odom_qy, odom_qz, odom_qw = floats[9], floats[10], floats[11], floats[12]
            
            # 提取IMU数据
            imu_wx, imu_wy, imu_wz = floats[13], floats[14], floats[15]
            imu_ax, imu_ay, imu_az = floats[16], floats[17], floats[18]
            imu_qx, imu_qy, imu_qz, imu_qw = floats[19], floats[20], floats[21], floats[22]
            
            # 当前时间戳
            now = self.get_clock().now().to_msg()
            
            # 发布里程计消息
            self.publish_odometry(now, odom_x, odom_y, odom_z, odom_qx, odom_qy, odom_qz, odom_qw, 
                                 odom_vx, odom_vy, odom_vz, odom_wx, odom_wy, odom_wz)
            
            # 发布IMU消息
            self.publish_imu(now, imu_qx, imu_qy, imu_qz, imu_qw, 
                           imu_wx, imu_wy, imu_wz, imu_ax, imu_ay, imu_az)
            
        except struct.error as e:
            self.get_logger().warning(f"二进制数据解析出错: {e}")
        except Exception as e:
            self.get_logger().error(f"数据处理出错: {e}")
    
    def publish_odometry(self, timestamp, x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz):
        """发布里程计消息和tf变换"""
        # 创建里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id
        
        # 设置位置
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        
        # 设置姿态
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # 设置线速度
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = vz

        # 设置角速度（直接用里程计角速度）
        odom_msg.twist.twist.angular.x = wx
        odom_msg.twist.twist.angular.y = wy
        odom_msg.twist.twist.angular.z = wz
        
        # 发布里程计消息
        self.odom_publisher.publish(odom_msg)
        
        # # 不发布tf变换，应当融合后通过robot_localization发布
        # t = TransformStamped()
        # t.header.stamp = timestamp
        # t.header.frame_id = self.frame_id
        # t.child_frame_id = self.child_frame_id
        
        # t.transform.translation.x = x
        # t.transform.translation.y = y
        # t.transform.translation.z = z
        
        # t.transform.rotation.x = qx
        # t.transform.rotation.y = qy
        # t.transform.rotation.z = qz
        # t.transform.rotation.w = qw
        
        # self.tf_broadcaster.sendTransform(t)
    
    def publish_imu(self, timestamp, qx, qy, qz, qw, wx, wy, wz, ax, ay, az):
        """发布IMU消息"""
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp
        imu_msg.header.frame_id = self.imu_frame_id
        
        # 设置姿态
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        
        # 设置角速度
        imu_msg.angular_velocity.x = wx
        imu_msg.angular_velocity.y = wy
        imu_msg.angular_velocity.z = wz
        
        # 设置线加速度
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        
        # 协方差矩阵，设置为-1表示忽略这些数据
        imu_msg.orientation_covariance = np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        imu_msg.angular_velocity_covariance = np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        imu_msg.linear_acceleration_covariance = np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        
        # 发布IMU消息
        self.imu_publisher.publish(imu_msg)

    def close(self):
        """关闭资源"""
        self.running = False  # 停止读取线程
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)  # 等待线程结束，最多1秒
        if self.serial_fd >= 0:
            wiringpi.serialClose(self.serial_fd)

def main(args=None):
    rclpy.init(args=args)
    node = OdomImuPublisherNode()
    
    # 检查串口是否初始化成功
    if not hasattr(node, 'serial_fd') or node.serial_fd < 0:
        node.get_logger().error("串口初始化失败，节点退出")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        node.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
