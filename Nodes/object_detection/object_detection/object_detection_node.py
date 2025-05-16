#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from object_detection_interfaces.msg import DetectionResult
import cv2
import numpy as np
import tf2_ros
import time
import threading
from geometry_msgs.msg import Point, TransformStamped
import tf2_geometry_msgs
from tf2_ros import TransformException
import os
from typing import List, Tuple, Optional, Dict

class ObjectDetectionNode(Node):
    """
    物体检测节点，负责从摄像头获取图像并检测物体
    """
    
    def __init__(self):
        super().__init__('object_detection_node')
        self.get_logger().info('Object Detection Node 启动中...')
        
        # 创建回调组，允许并发执行回调
        self.callback_group = ReentrantCallbackGroup()
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 检测使能标志
        self.detection_enabled = False
        
        # 当前图像
        self.bottom_image = None
        self.top_image = None
        
        # 图像时间戳
        self.bottom_timestamp = None
        self.top_timestamp = None
        
        # 相机信息
        self.bottom_camera_info = None
        self.top_camera_info = None
        
        # 相机内参和畸变系数
        # 顶部相机参数 (320x240)
        self.top_camera_matrix = np.array([
            [240.70762, 0.0, 153.81297],
            [0.0, 241.42526, 119.14773],
            [0.0, 0.0, 1.0]
        ])
        self.top_dist_coeffs = np.array([-0.414435, 0.176258, 0.002182, -0.000329, 0.0])
        
        # 底部相机参数 (640x480)
        self.bottom_camera_matrix = np.array([
            [465.13093, 0.0, 324.81802],
            [0.0, 466.33628, 242.54136],
            [0.0, 0.0, 1.0]
        ])
        self.bottom_dist_coeffs = np.array([-0.374992, 0.133505, 0.002906, -0.002975, 0.0])
        
        # 相机相对位置 (top相对于bottom的位置)
        self.camera_translation = np.array([5.0, 0.0, 11.0])  # 单位：cm
        
        # 特征匹配参数
        self.feature_detector = cv2.SIFT_create()
        self.feature_matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        
        # 加载YOLO模型
        self.declare_parameter('yolo_model_path', 'yolov5s.onnx')
        yolo_model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value
        
        # 检查模型文件是否存在
        if not os.path.exists(yolo_model_path):
            self.get_logger().error(f"YOLO模型文件不存在: {yolo_model_path}")
            self.get_logger().warning("将使用颜色检测作为备选方法")
            self.yolo_net = None
        else:
            try:
                self.yolo_net = cv2.dnn.readNetFromONNX(yolo_model_path)
                self.get_logger().info(f"YOLO模型加载成功: {yolo_model_path}")
            except Exception as e:
                self.get_logger().error(f"YOLO模型加载失败: {str(e)}")
                self.yolo_net = None
        
        # 检测阈值
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('distance_threshold', 1.0)  # 成功检测阈值距离(m)
        
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.nms_threshold = self.get_parameter('nms_threshold').get_parameter_value().double_value
        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        
        # 类别标签
        self.classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
                       'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse',
                       'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
                       'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
                       'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon',
                       'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut',
                       'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
                       'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book',
                       'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
        
        # TF相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 线程锁
        self.lock = threading.RLock()
        
        # 发布者
        self.detection_pub = self.create_publisher(
            DetectionResult,
            '/object_detection/result',
            10
        )
        
        # 可视化发布者
        self.bottom_detection_pub = self.create_publisher(
            Image,
            '/object_detection/bottom_image',
            10
        )
        
        self.top_detection_pub = self.create_publisher(
            Image,
            '/object_detection/top_image',
            10
        )
        
        # 订阅者
        # 订阅底部摄像头图像
        self.bottom_image_sub = self.create_subscription(
            Image,
            '/bottom_camera/image_raw',
            self.bottom_image_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 订阅顶部摄像头图像
        self.top_image_sub = self.create_subscription(
            Image,
            '/top_camera/image_raw',
            self.top_image_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 订阅底部摄像头信息
        self.bottom_camera_info_sub = self.create_subscription(
            CameraInfo,
            '/bottom_camera/camera_info',
            self.bottom_camera_info_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 订阅顶部摄像头信息
        self.top_camera_info_sub = self.create_subscription(
            CameraInfo,
            '/top_camera/camera_info',
            self.top_camera_info_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 订阅检测使能控制
        self.enable_detection_sub = self.create_subscription(
            Bool,
            '/enable_detection',
            self.enable_detection_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 定时器，执行定期检测
        self.detection_timer = self.create_timer(
            0.1,  # 10Hz
            self.detection_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Object Detection Node 初始化完成')
    
    def bottom_image_callback(self, msg):
        """处理底部摄像头图像"""
        if not self.detection_enabled:
            return
        
        try:
            with self.lock:
                self.bottom_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.bottom_timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'转换底部图像失败: {str(e)}')
    
    def top_image_callback(self, msg):
        """处理顶部摄像头图像"""
        if not self.detection_enabled:
            return
        
        try:
            with self.lock:
                self.top_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.top_timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'转换顶部图像失败: {str(e)}')
    
    def bottom_camera_info_callback(self, msg):
        """处理底部摄像头信息"""
        self.bottom_camera_info = msg
    
    def top_camera_info_callback(self, msg):
        """处理顶部摄像头信息"""
        self.top_camera_info = msg
    
    def enable_detection_callback(self, msg):
        """处理检测使能控制"""
        enabled = msg.data
        if enabled != self.detection_enabled:
            self.get_logger().info(f'{"启用" if enabled else "禁用"}物体检测')
            self.detection_enabled = enabled
            
            # 重置图像缓存
            if not enabled:
                with self.lock:
                    self.bottom_image = None
                    self.top_image = None
    
    def detection_callback(self):
        """周期性执行物体检测"""
        if not self.detection_enabled:
            return
        
        with self.lock:
            bottom_image = self.bottom_image
            top_image = self.top_image
        
        if bottom_image is None or top_image is None:
            return
        
        # 执行物体检测
        detected_objects, object_position = self.detect_objects(bottom_image, top_image)
        
        # 创建并发布检测结果
        result_msg = DetectionResult()
        result_msg.header.stamp = self.get_clock().now().to_msg()
        result_msg.header.frame_id = 'bottom_camera_frame'
        result_msg.detected_objects = detected_objects
        
        if detected_objects > 0 and object_position is not None:
            result_msg.object_position = object_position
        
        self.detection_pub.publish(result_msg)
        
        if detected_objects > 0:
            self.get_logger().debug(f'检测到{detected_objects}个物体，位置: [{object_position.x:.2f}, {object_position.y:.2f}, {object_position.z:.2f}]')
    
    def detect_objects(self, bottom_image, top_image):
        """
        检测物体并估计位置
        
        Args:
            bottom_image: 底部摄像头图像
            top_image: 顶部摄像头图像
            
        Returns:
            tuple: (检测到的物体数量, 物体位置)
        """
        # 这里实现物体检测的具体算法
        # 可以使用OpenCV、深度学习框架等
        
        # 示例：简单的颜色阈值分割检测红色物体
        try:
            # 转换到HSV色彩空间
            bottom_hsv = cv2.cvtColor(bottom_image, cv2.COLOR_BGR2HSV)
            
            # 定义红色的HSV范围
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            
            # 创建掩码
            mask = cv2.inRange(bottom_hsv, lower_red, upper_red)
            
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 过滤小的轮廓
            contours = [c for c in contours if cv2.contourArea(c) > 100]
            
            if len(contours) > 0:
                # 找到最大的轮廓
                largest_contour = max(contours, key=cv2.contourArea)
                
                # 计算质心
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # 这里应该使用相机参数将图像坐标转换为3D世界坐标
                    # 简化处理：假设物体在相机前方50cm处
                    from geometry_msgs.msg import Point
                    object_position = Point()
                    
                    # 这部分应该使用相机内参和立体视觉算法计算实际位置
                    # 简化实现，仅演示框架
                    if self.bottom_camera_info:
                        fx = self.bottom_camera_info.k[0]  # 焦距 x
                        fy = self.bottom_camera_info.k[4]  # 焦距 y
                        cx = self.bottom_camera_info.k[2]  # 主点 x
                        cy = self.bottom_camera_info.k[5]  # 主点 y
                        
                        # 假设物体距离为0.5米
                        depth = 0.5
                        
                        # 将像素坐标转换为相机坐标系下的3D点
                        object_position.x = depth
                        object_position.y = (cx - self.bottom_camera_info.width / 2) * depth / fx
                        object_position.z = (cy - self.bottom_camera_info.height / 2) * depth / fy
                    else:
                        # 没有相机参数，使用简化的估计
                        object_position.x = 0.5  # 假设前方50cm
                        object_position.y = (cx - bottom_image.shape[1] / 2) / 100.0  # 简化的横向位置估计
                        object_position.z = (cy - bottom_image.shape[0] / 2) / 100.0  # 简化的纵向位置估计
                    
                    return len(contours), object_position
            
            # 没有检测到物体
            return 0, None
            
        except Exception as e:
            self.get_logger().error(f'物体检测失败: {str(e)}')
            return 0, None

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
