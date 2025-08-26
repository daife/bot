#!/usr/bin/env python3

import time
import math
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError

class CarStatusManager:
    """
    小车状态管理器，负责获取和管理小车的各种状态信息
    """
    def __init__(self, node):
        self.node = node
        
        # CV Bridge用于图像转换
        self.cv_bridge = CvBridge()
        
        # 状态变量
        self.global_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # 全局坐标
        self.pose = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}    # 小车姿态
        self.velocity = {'vx': 0.0, 'vy': 0.0, 'vyaw': 0.0}   # 小车速度
        self.target_detected = False                           # 是否识别到指定物体
        self.target_position = {'x': 0.0, 'y': 0.0}          # 目标在画面中的位置
        self.latest_image = None                               # 最新的识别画面
        self.last_image_time = 0.0                            # 最后一次收到图像的时间
        
        # 设置订阅器
        self._setup_subscriptions()
        
        self.node.get_logger().info('小车状态管理器已初始化')
    
    def _setup_subscriptions(self):
        """设置状态监听的订阅器"""
        # 订阅里程计话题（获取全局坐标、姿态和速度）
        self.odom_subscription = self.node.create_subscription(
            Odometry,
            '/odom',  # 根据chassis_controlerAndpublisher的发布话题
            self._odom_callback,
            10
        )
        
        # 订阅目标检测结果（从YOLO节点获取）
        self.target_pose_subscription = self.node.create_subscription(
            Pose,
            '/onecam_yolo/target_pose',  # 从yolo_node.py获取目标位置
            self._target_pose_callback,
            10
        )
        
        # 订阅识别画面（从YOLO节点获取处理后的图像）
        self.image_subscription = self.node.create_subscription(
            Image,
            '/onecam_yolo/image',  # 从yolo_node.py获取标注后的图像
            self._image_callback,
            10
        )
        
        self.node.get_logger().info('状态订阅器已设置完成')
    
    def _odom_callback(self, msg):
        """里程计回调函数 - 获取全局坐标、姿态和速度"""
        try:
            # 提取全局坐标
            self.global_position['x'] = msg.pose.pose.position.x
            self.global_position['y'] = msg.pose.pose.position.y
            self.global_position['z'] = msg.pose.pose.position.z
            
            # 提取姿态（四元数转欧拉角）
            orientation = msg.pose.pose.orientation
            self.pose['roll'], self.pose['pitch'], self.pose['yaw'] = self._quaternion_to_euler(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
            
            # 提取速度信息
            self.velocity['vx'] = msg.twist.twist.linear.x
            self.velocity['vy'] = msg.twist.twist.linear.y
            self.velocity['vyaw'] = msg.twist.twist.angular.z
            
        except Exception as e:
            self.node.get_logger().error(f'里程计数据解析错误: {str(e)}')
    
    def _target_pose_callback(self, msg):
        """目标检测结果回调函数"""
        try:
            # 根据yolo_node.py的逻辑：
            # position.x = dx (目标相对于图像中心的x偏移)
            # position.y = dy (目标相对于图像中心的y偏移)  
            # position.z = 0.0 表示未检测到
            
            self.target_detected = (msg.position.z > 0.0)  # z > 0.0 表示检测到目标
            
            if self.target_detected:
                self.target_position['x'] = msg.position.x
                self.target_position['y'] = msg.position.y
            else:
                self.target_position['x'] = 0.0
                self.target_position['y'] = 0.0
            
        except Exception as e:
            self.node.get_logger().error(f'目标检测数据解析错误: {str(e)}')
    
    def _image_callback(self, msg):
        """识别画面回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_image_time = time.time()
            
        except CvBridgeError as e:
            self.node.get_logger().error(f'图像转换错误: {str(e)}')
        except Exception as e:
            self.node.get_logger().error(f'图像数据处理错误: {str(e)}')
    
    def _quaternion_to_euler(self, x, y, z, w):
        """四元数转欧拉角"""
        try:
            # Roll (x-axis rotation)
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            
            # Pitch (y-axis rotation)
            sinp = 2 * (w * y - z * x)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
            else:
                pitch = math.asin(sinp)
            
            # Yaw (z-axis rotation)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return roll, pitch, yaw
            
        except Exception as e:
            self.node.get_logger().error(f'四元数转欧拉角错误: {str(e)}')
            return 0.0, 0.0, 0.0
    
    def get_arm_angle(self):
        """
        获取机械臂角度 - 通过car_controller获取
        因为机械臂状态需要通过Action客户端查询，这里返回controller中的状态
        """
        from .car_control import CarController
        # 如果controller已经初始化，获取其arm_angle
        if hasattr(self.node, 'car_controller'):
            return self.node.car_controller.arm_angle
        return 0.0
    
    def get_claw_state(self):
        """
        获取爪子状态 - 通过car_controller获取
        因为爪子状态需要通过Action客户端查询，这里返回controller中的状态
        """
        from .car_control import CarController
        # 如果controller已经初始化，获取其claw_state
        if hasattr(self.node, 'car_controller'):
            claw_state_num = self.node.car_controller.claw_state
            return 'grasped' if claw_state_num == 0 else 'released'
        return 'released'
    
    def get_all_status(self):
        """获取所有状态信息"""
        return {
            'global_position': self.global_position.copy(),
            'pose': self.pose.copy(),
            'velocity': self.velocity.copy(),
            'arm_angle': self.get_arm_angle(),
            'claw_state': self.get_claw_state(),
            'target_detected': self.target_detected,
            'target_position': self.target_position.copy(),
            'last_image_time': self.last_image_time,
            'timestamp': time.time()
        }
    
    def get_position(self):
        """获取当前全局坐标"""
        return self.global_position.copy()
    
    def get_pose(self):
        """获取当前姿态"""
        return self.pose.copy()
    
    def get_velocity(self):
        """获取当前速度"""
        return self.velocity.copy()
    
    def get_target_info(self):
        """获取目标检测信息"""
        return {
            'detected': self.target_detected,
            'position': self.target_position.copy()
        }
    
    def get_image_status(self):
        """获取图像状态"""
        return {
            'available': self.latest_image is not None,
            'last_update_time': self.last_image_time,
            'age': time.time() - self.last_image_time if self.last_image_time > 0 else float('inf')
        }
    
    def is_moving(self):
        """检查小车是否在移动"""
        vel = self.velocity
        threshold = 0.01
        return (abs(vel['vx']) > threshold or 
                abs(vel['vy']) > threshold or 
                abs(vel['vyaw']) > threshold)
    
    def get_latest_image(self):
        """获取最新的识别画面（OpenCV格式）"""
        return self.latest_image