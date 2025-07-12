#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import pyudev
import struct
import time
import math
import threading
import numpy as np

class IMUPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # 声明参数
        self.declare_parameter('device_serial_id', '1a86_USB_Serial')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)  # Hz
        
        # 获取参数
        self.device_serial_id = self.get_parameter('device_serial_id').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 创建发布器 - 使用与odom_imu_publisher相同的话题名
        self.imu_publisher = self.create_publisher(Imu, '/imu', 10)
        
        # 初始化数据存储
        self.latest_data = {
            'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'time': 0},
            'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'time': 0},
            'angle': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'time': 0}
        }
        
        # 串口相关
        self.serial_port = None
        self.device_path = None
        self.running = False
        
        # 初始化连接
        if self.find_and_connect_imu():
            # 启动数据读取线程
            self.running = True
            self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
            self.reader_thread.start()
            
            # 创建定时器发布IMU数据
            self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu_data)
            
            self.get_logger().info('IMU Publisher Node 初始化完成')
        else:
            self.get_logger().error('无法连接到IMU设备，节点初始化失败')
    
    def find_imu_device(self):
        """查找指定ID_SERIAL的设备"""
        self.get_logger().info("开始搜索IMU设备...")
        context = pyudev.Context()
        
        for device in context.list_devices(subsystem='tty'):
            if device.get('ID_SERIAL') == self.device_serial_id:
                self.device_path = device.device_node
                self.get_logger().info(f"找到IMU设备: {self.device_path}")
                return True
        
        self.get_logger().error("未找到指定的IMU设备")
        return False
    
    def connect_serial(self):
        """连接串口"""
        if not self.device_path:
            return False
        
        try:
            self.serial_port = serial.Serial(
                port=self.device_path,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.get_logger().info(f"串口连接成功: {self.device_path}")
            # 清空缓冲区
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            return True
        except Exception as e:
            self.get_logger().error(f"串口连接失败: {e}")
            return False
    
    def find_and_connect_imu(self):
        """查找并连接IMU设备"""
        if self.find_imu_device():
            return self.connect_serial()
        return False
    
    def verify_checksum(self, data_packet):
        """验证校验和"""
        if len(data_packet) != 11:
            return False
        
        sum_crc = 0
        for i in range(10):  # 前10个字节参与校验
            sum_crc += data_packet[i]
        
        calculated_crc = sum_crc & 0xFF
        received_crc = data_packet[10]
        return calculated_crc == received_crc
    
    def parse_data(self, data_type, data_bytes):
        """解析数据包中的三轴数据"""
        if len(data_bytes) < 6:
            return None, None, None
        
        # 按照协议：DATA1=(short)((short)DATA1H<<8|DATA1L)
        # 数据顺序是低字节在前，高字节在后
        x_raw = (data_bytes[1] << 8) | data_bytes[0]
        y_raw = (data_bytes[3] << 8) | data_bytes[2]
        z_raw = (data_bytes[5] << 8) | data_bytes[4]
        
        # 转换为有符号short
        if x_raw > 32767: x_raw -= 65536
        if y_raw > 32767: y_raw -= 65536
        if z_raw > 32767: z_raw -= 65536
        
        # 根据数据类型转换为标准单位
        if data_type == 0x51:  # 加速度
            # 单位转换: 32768*16g -> m/s²
            scale = 16 * 9.8 / 32768
            return x_raw * scale, y_raw * scale, z_raw * scale
        elif data_type == 0x52:  # 角速度
            # 单位转换: 32768*2000°/s -> rad/s
            scale = 2000 * math.pi / (180 * 32768)
            return x_raw * scale, y_raw * scale, z_raw * scale
        elif data_type == 0x53:  # 角度
            # 单位转换: 32768*180° -> rad
            scale = math.pi / 32768
            return x_raw * scale, y_raw * scale, z_raw * scale
        
        return None, None, None
    
    def _serial_reader(self):
        """后台线程，持续从串口读取并解析IMU数据"""
        buffer = bytearray()
        
        while self.running and self.serial_port:
            try:
                # 读取数据
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(data)
                
                # 解析数据包
                while len(buffer) >= 11:
                    # 查找协议头 0x55
                    header_index = -1
                    for i in range(len(buffer) - 10):
                        if buffer[i] == 0x55 and buffer[i + 1] in [0x51, 0x52, 0x53, 0x54]:
                            header_index = i
                            break
                    
                    if header_index == -1:
                        buffer = buffer[-10:]
                        break
                    
                    if header_index > 0:
                        buffer = buffer[header_index:]
                    
                    if len(buffer) < 11:
                        break
                    
                    packet = buffer[:11]
                    buffer = buffer[11:]
                    
                    # 验证校验和
                    if not self.verify_checksum(packet):
                        continue
                    
                    # 解析数据
                    data_type = packet[1]
                    data_bytes = packet[2:8]
                    
                    x, y, z = self.parse_data(data_type, data_bytes)
                    
                    if x is not None:
                        current_time = time.time()
                        
                        # 更新对应的数据
                        if data_type == 0x51:  # 加速度
                            self.latest_data['accel'] = {'x': x, 'y': y, 'z': z, 'time': current_time}
                        elif data_type == 0x52:  # 角速度
                            self.latest_data['gyro'] = {'x': x, 'y': y, 'z': z, 'time': current_time}
                        elif data_type == 0x53:  # 角度
                            self.latest_data['angle'] = {'x': x, 'y': y, 'z': z, 'time': current_time}
                
                time.sleep(0.001)  # 短暂休眠
                
            except Exception as e:
                self.get_logger().error(f"串口读取错误: {e}")
                time.sleep(0.1)
    
    def publish_imu_data(self):
        """发布IMU数据"""
        try:
            # 创建IMU消息
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame_id
            
            # 设置角速度（来自gyro数据）
            imu_msg.angular_velocity.x = self.latest_data['gyro']['x']
            imu_msg.angular_velocity.y = self.latest_data['gyro']['y']
            imu_msg.angular_velocity.z = self.latest_data['gyro']['z']
            
            # 设置线加速度（来自accel数据）
            imu_msg.linear_acceleration.x = self.latest_data['accel']['x']
            imu_msg.linear_acceleration.y = self.latest_data['accel']['y']
            imu_msg.linear_acceleration.z = self.latest_data['accel']['z']
            
            # 从角度数据计算四元数（假设角度数据是欧拉角）
            roll = self.latest_data['angle']['x']
            pitch = self.latest_data['angle']['y']
            yaw = self.latest_data['angle']['z']
            
            # 欧拉角转四元数
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            
            imu_msg.orientation.w = cy * cp * cr + sy * sp * sr
            imu_msg.orientation.x = cy * cp * sr - sy * sp * cr
            imu_msg.orientation.y = sy * cp * sr + cy * sp * cr
            imu_msg.orientation.z = sy * cp * cr - cy * sp * sr
            
            # 设置协方差矩阵（设置为-1表示未知）
            imu_msg.orientation_covariance = np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
            imu_msg.angular_velocity_covariance = np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
            imu_msg.linear_acceleration_covariance = np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
            
            # 发布IMU消息
            self.imu_publisher.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f"发布IMU数据时出错: {e}")
    
    def close(self):
        """关闭资源"""
        self.running = False
        if hasattr(self, 'reader_thread') and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        if self.serial_port:
            self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisherNode()
    
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
