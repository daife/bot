#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
import time
import threading
import signal
import sys
import os
import yaml
import select
import termios
import tty

from .kalman_filter import KalmanFilter2D
from .pid_controller import PIDController, CascadePIDController
from .servo_control import YawServoController, PitchServoController

class CameraTrackingNode(Node):
    """
    摄像头跟踪节点 - 使用卡尔曼滤波和PID控制云台跟踪纸条
    支持调参模式，可实时重载参数
    """
    
    def __init__(self):
        super().__init__('camera_tracking_node')
        
        # 声明调参模式相关参数
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('debug_config_path', '/home/HwHiAiUser/ros/src/Nodes/camera_tracking/config/tracking_params.yaml')
        
        # 获取调参模式设置
        self.debug_mode = self.get_parameter('debug_mode').value
        self.debug_config_path = self.get_parameter('debug_config_path').value
        
        # 初始化debug_params为空字典
        self.debug_params = {}
        
        if self.debug_mode:
            self.get_logger().info(f'启用调参模式，监听配置文件: {self.debug_config_path}')
            self.get_logger().info('按 "r" 键重载参数，按 "q" 键退出')
            
            # 检查调参配置文件是否存在
            if not os.path.exists(self.debug_config_path):
                self.get_logger().warn(f'调参配置文件不存在: {self.debug_config_path}')
                self.get_logger().warn('将使用编译时参数作为默认值')
                # 声明编译时参数作为后备
                self.declare_compile_time_parameters()
            
            self.load_debug_parameters()
        else:
            self.get_logger().info('使用编译时参数')
            self.declare_compile_time_parameters()
        
        # 初始化组件
        self.init_components()
        
        # 跟踪状态
        self.target_lost_count = 0
        self.max_lost_frames = self.get_param_value('tracking.max_lost_frames', 30)
        self.tracking_active = False
        self.last_detection_time = time.time()
        
        # 死区参数
        self.deadzone_x = self.get_param_value('tracking.deadzone_x', 5.0)
        self.deadzone_y = self.get_param_value('tracking.deadzone_y', 5.0)
        
        # 订阅纸条位置话题
        self.pose_subscription = self.create_subscription(
            Pose,
            'paper_center_pose',
            self.pose_callback,
            10
        )
        
        # 创建控制定时器 (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_callback)
        
        # 启动舵机PWM
        self.yaw_servo.start_pwm()
        self.pitch_servo.start_pwm()
        
        # 如果是调参模式，启动键盘监听线程
        if self.debug_mode:
            self.keyboard_running = True
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
            self.keyboard_thread.start()
        
        # 信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.get_logger().info('摄像头跟踪节点已启动')
        if self.debug_mode:
            self.get_logger().info(f'死区设置: x={self.deadzone_x}, y={self.deadzone_y}')
    
    def load_debug_parameters(self):
        """加载调参模式参数文件"""
        try:
            if os.path.exists(self.debug_config_path):
                with open(self.debug_config_path, 'r') as file:
                    config = yaml.safe_load(file)
                    self.debug_params = config.get('camera_tracking_node', {}).get('ros__parameters', {})
                    self.get_logger().info('成功加载调参配置文件')
            else:
                self.get_logger().warn('调参配置文件不存在，使用空参数字典')
                self.debug_params = {}
                # 仍然声明编译时参数作为后备
                self.declare_compile_time_parameters()
        except Exception as e:
            self.get_logger().error(f'加载调参配置文件失败: {e}')
            self.get_logger().warn('将使用编译时参数')
            self.debug_params = {}
            self.declare_compile_time_parameters()
    
    def declare_compile_time_parameters(self):
        """声明ROS参数（重命名原来的declare_parameters方法）"""
        # 卡尔曼滤波参数
        self.declare_parameter('kalman.process_variance', 1.0)
        self.declare_parameter('kalman.measurement_variance', 25.0)
        
        # Yaw轴串级PID参数
        self.declare_parameter('yaw.position_pid.kp', 0.01)
        self.declare_parameter('yaw.position_pid.ki', 0.001)
        self.declare_parameter('yaw.position_pid.kd', 0.005)
        self.declare_parameter('yaw.velocity_pid.kp', 0.8)
        self.declare_parameter('yaw.velocity_pid.ki', 0.1)
        self.declare_parameter('yaw.velocity_pid.kd', 0.05)
        
        # Pitch轴PID参数
        self.declare_parameter('pitch.pid.kp', 0.15)
        self.declare_parameter('pitch.pid.ki', 0.01)
        self.declare_parameter('pitch.pid.kd', 0.08)
        
        # 舵机参数
        self.declare_parameter('servo.yaw_pin', 6)
        self.declare_parameter('servo.pitch_pin', 19)  # 修改为19
        self.declare_parameter('servo.pitch_center_angle', 90)
        self.declare_parameter('servo.pitch_min_angle', 30)
        self.declare_parameter('servo.pitch_max_angle', 150)
        
        # 跟踪参数
        self.declare_parameter('tracking.deadzone_x', 5.0)
        self.declare_parameter('tracking.deadzone_y', 5.0)
        self.declare_parameter('tracking.max_lost_frames', 30)
        self.declare_parameter('tracking.speed_limit', 0.8)
        self.declare_parameter('tracking.angle_change_limit', 2.0)
    
    def get_param_value(self, key_path, default_value):
        """统一获取参数值的方法"""
        if self.debug_mode and self.debug_params:
            keys = key_path.split('.')
            value = self.debug_params
            for key in keys:
                if isinstance(value, dict):
                    value = value.get(key, {})
                else:
                    break
            
            # 如果从调参文件中找到了有效值，使用它
            if value != {} and not isinstance(value, dict):
                return value
            
            # 否则尝试从ROS参数获取（如果已声明）
            try:
                return self.get_parameter(key_path).value
            except:
                return default_value
        else:
            # 非调参模式或调参参数为空，使用ROS参数
            try:
                return self.get_parameter(key_path).value
            except:
                return default_value
    
    def init_components(self):
        """初始化组件"""
        # 初始化卡尔曼滤波器
        process_var = self.get_param_value('kalman.process_variance', 1.0)
        measurement_var = self.get_param_value('kalman.measurement_variance', 25.0)
        self.kalman_filter = KalmanFilter2D(process_var, measurement_var)
        
        # 初始化PID控制器
        self.init_controllers()
        
        # 初始化舵机控制器
        self.init_servos()
    
    def init_controllers(self):
        """初始化控制器"""
        # Yaw轴串级PID参数
        yaw_pos_params = {
            'kp': self.get_param_value('yaw.position_pid.kp', 0.01),
            'ki': self.get_param_value('yaw.position_pid.ki', 0.001),
            'kd': self.get_param_value('yaw.position_pid.kd', 0.005),
            'output_limits': (-50.0, 50.0),
            'windup_limit': 100.0
        }
        
        speed_limit = self.get_param_value('tracking.speed_limit', 0.8)
        yaw_vel_params = {
            'kp': self.get_param_value('yaw.velocity_pid.kp', 0.8),
            'ki': self.get_param_value('yaw.velocity_pid.ki', 0.1),
            'kd': self.get_param_value('yaw.velocity_pid.kd', 0.05),
            'output_limits': (-speed_limit, speed_limit),
            'windup_limit': 10.0
        }
        
        self.yaw_controller = CascadePIDController(yaw_pos_params, yaw_vel_params)
        
        # Pitch轴PID参数
        angle_limit = self.get_param_value('tracking.angle_change_limit', 2.0)
        pitch_params = {
            'kp': self.get_param_value('pitch.pid.kp', 0.15),
            'ki': self.get_param_value('pitch.pid.ki', 0.01),
            'kd': self.get_param_value('pitch.pid.kd', 0.08),
            'output_limits': (-angle_limit, angle_limit),
            'windup_limit': 50.0
        }
        
        self.pitch_controller = PIDController(**pitch_params)
    
    def init_servos(self):
        """初始化舵机"""
        yaw_pin = self.get_param_value('servo.yaw_pin', 6)
        pitch_pin = self.get_param_value('servo.pitch_pin', 19)
        
        self.yaw_servo = YawServoController(pin=yaw_pin)
        
        self.pitch_servo = PitchServoController(
            pin=pitch_pin,
            min_angle=self.get_param_value('servo.pitch_min_angle', 30),
            max_angle=self.get_param_value('servo.pitch_max_angle', 150)
        )
        
        # 设置Pitch初始角度为中心位置
        center_angle = self.get_param_value('servo.pitch_center_angle', 90)
        self.pitch_servo.set_angle(center_angle)
    
    def keyboard_listener(self):
        """键盘监听线程（调参模式）"""
        # 保存原始终端设置
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            # 设置终端为非阻塞模式
            tty.setraw(sys.stdin.fileno())
            
            while self.keyboard_running:
                # 检查是否有输入
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    if key == 'r' or key == 'R':
                        self.reload_parameters()
                    elif key == 'q' or key == 'Q':
                        self.get_logger().info('用户请求退出')
                        rclpy.shutdown()
                        break
                        
        except Exception as e:
            self.get_logger().error(f'键盘监听错误: {e}')
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def reload_parameters(self):
        """重新加载参数"""
        if not self.debug_mode:
            self.get_logger().warn('非调参模式，无法重载参数')
            return
        
        self.get_logger().info('重新加载参数...')
        
        # 停止当前跟踪
        self.stop_tracking()
        
        try:
            # 重新加载配置文件
            self.load_debug_parameters()
            
            # 重新初始化组件
            self.init_components()
            
            # 更新死区参数
            self.deadzone_x = self.get_param_value('tracking.deadzone_x', 5.0)
            self.deadzone_y = self.get_param_value('tracking.deadzone_y', 5.0)
            self.max_lost_frames = self.get_param_value('tracking.max_lost_frames', 30)
            
            # 重新启动舵机
            self.yaw_servo.stop_pwm()
            self.pitch_servo.stop_pwm()
            
            self.yaw_servo.start_pwm()
            self.pitch_servo.start_pwm()
            
            # 重置Pitch到中心位置
            center_angle = self.get_param_value('servo.pitch_center_angle', 90)
            self.pitch_servo.set_angle(center_angle)
            
            self.get_logger().info('参数重载完成！')
            self.get_logger().info(f'新的死区设置: x={self.deadzone_x}, y={self.deadzone_y}')
            
        except Exception as e:
            self.get_logger().error(f'参数重载失败: {e}')
    
    def pose_callback(self, msg):
        """处理纸条位置消息"""
        current_time = time.time()
        
        if msg.position.z == -1.0:
            # 未检测到目标
            self.target_lost_count += 1
            
            if self.target_lost_count > self.max_lost_frames:
                if self.tracking_active:
                    self.get_logger().warn('目标丢失，停止跟踪')
                    self.stop_tracking()
            return
        
        # 检测到目标
        self.target_lost_count = 0
        self.last_detection_time = current_time
        
        if not self.tracking_active:
            self.get_logger().info('开始跟踪目标')
            self.start_tracking()
        
        # 提取像素偏差
        pixel_error_x = msg.position.x
        pixel_error_y = msg.position.y
        
        # 卡尔曼滤波
        self.kalman_filter.predict()
        self.kalman_filter.update([pixel_error_x, pixel_error_y])
        
        # 获取滤波后的位置和速度
        filtered_x, filtered_y = self.kalman_filter.get_position()
        velocity_x, velocity_y = self.kalman_filter.get_velocity()
        
        self.get_logger().debug(
            f'原始偏差: ({pixel_error_x:.1f}, {pixel_error_y:.1f}), '
            f'滤波后: ({filtered_x:.1f}, {filtered_y:.1f}), '
            f'速度: ({velocity_x:.1f}, {velocity_y:.1f})'
        )
        
        # 存储滤波后的误差用于控制
        self.filtered_error_x = filtered_x
        self.filtered_error_y = filtered_y
        self.velocity_x = velocity_x
        self.velocity_y = velocity_y
    
    def control_callback(self):
        """控制回调函数 - 50Hz"""
        if not self.tracking_active:
            return
        
        if not hasattr(self, 'filtered_error_x'):
            return
        
        # 应用死区
        error_x = self.filtered_error_x if abs(self.filtered_error_x) > self.deadzone_x else 0.0
        error_y = self.filtered_error_y if abs(self.filtered_error_y) > self.deadzone_y else 0.0
        
        # Yaw轴控制 (串级PID)
        yaw_output = self.yaw_controller.update(error_x, self.velocity_x)
        self.yaw_servo.set_speed(yaw_output)
        
        # Pitch轴控制 (位置PID)
        pitch_change = self.pitch_controller.update(error_y)
        current_pitch = self.pitch_servo.get_angle()
        new_pitch = current_pitch + pitch_change
        self.pitch_servo.set_angle(new_pitch)
        
        self.get_logger().debug(
            f'控制输出 - Yaw速度: {yaw_output:.3f}, '
            f'Pitch角度: {current_pitch:.1f} -> {new_pitch:.1f} (变化: {pitch_change:.2f}), '
            f'误差: ({error_x:.1f}, {error_y:.1f})'
        )
    
    def start_tracking(self):
        """开始跟踪"""
        self.tracking_active = True
        self.target_lost_count = 0
        
        # 重置控制器
        self.yaw_controller.reset()
        self.pitch_controller.reset()
        
        self.get_logger().info('跟踪已激活')
    
    def stop_tracking(self):
        """停止跟踪"""
        self.tracking_active = False
        
        # 停止Yaw轴运动
        if hasattr(self, 'yaw_servo'):
            self.yaw_servo.set_speed(0.0)
        
        # 重置滤波器和控制器
        if hasattr(self, 'kalman_filter'):
            self.kalman_filter.reset()
        if hasattr(self, 'yaw_controller'):
            self.yaw_controller.reset()
        if hasattr(self, 'pitch_controller'):
            self.pitch_controller.reset()
        
        self.get_logger().info('跟踪已停止')
    
    def signal_handler(self, signum, frame):
        """信号处理函数"""
        self.get_logger().info('收到退出信号，正在清理资源...')
        self.cleanup()
        rclpy.shutdown()
        sys.exit(0)
    
    def cleanup(self):
        """清理资源"""
        self.stop_tracking()
        
        # 停止键盘监听
        if self.debug_mode:
            self.keyboard_running = False
        
        # 停止舵机PWM
        if hasattr(self, 'yaw_servo'):
            self.yaw_servo.stop_pwm()
        if hasattr(self, 'pitch_servo'):
            self.pitch_servo.stop_pwm()
        
        self.get_logger().info('资源清理完成')
    
    def __del__(self):
        """析构函数"""
        try:
            self.cleanup()
        except:
            pass  # 忽略析构时的错误


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraTrackingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行出错: {e}")
    finally:
        if 'node' in locals():
            node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
