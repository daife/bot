#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image
from yolo_detection_interfaces.msg import YoloDetection
from object_position_interfaces.msg import ObjectPosition
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import math
from geometry_msgs.msg import PoseStamped, Point, PointStamped

class ObjectLocalizerNode(Node):
    """
    物体定位节点：
    订阅两个摄像头的图像和YOLO检测结果，
    通过特征匹配和三角测量计算物体的3D坐标
    """

    def __init__(self):
        super().__init__('object_localizer_node')
        
        # 声明参数
        self.declare_parameter('confidence_threshold', 0.65)
        self.declare_parameter('feature_match_ratio', 0.7)
        self.declare_parameter('max_feature_points', 100)
        self.declare_parameter('publish_debug_images', True)
        
        # 获取参数
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.feature_match_ratio = self.get_parameter('feature_match_ratio').value
        self.max_feature_points = self.get_parameter('max_feature_points').value
        self.publish_debug_images = self.get_parameter('publish_debug_images').value
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # TF变换相关
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 存储摄像头内参
        self.bottom_camera_matrix = np.array([
            [465.13093, 0.0, 324.81802],
            [0.0, 466.33628, 242.54136],
            [0.0, 0.0, 1.0]
        ])
        
        self.top_camera_matrix = np.array([
            [240.70762, 0.0, 153.81297],
            [0.0, 241.42526, 119.14773],
            [0.0, 0.0, 1.0]
        ])
        
        # 特征检测器和描述子计算器
        self.feature_detector = cv2.SIFT_create(nfeatures=self.max_feature_points)
        self.feature_matcher = cv2.BFMatcher()
        
        # 存储最新的图像和检测结果
        self.top_image = None
        self.bottom_image = None
        self.current_detection = None
        
        # 标志位
        self.bottom_image_updated = False
        self.top_image_updated = False
        self.detection_updated = False
        
        # 发布器
        self.position_publisher = self.create_publisher(
            ObjectPosition, 'object_position', 10)
        
        # 调试图像发布器
        if self.publish_debug_images:
            self.debug_image_publisher = self.create_publisher(
                Image, 'object_localizer/debug_image', 1)
        
        # 订阅器
        self.bottom_image_subscription = self.create_subscription(
            Image, 'bottom_camera/image_rect', self.bottom_image_callback, 10)
        
        self.top_image_subscription = self.create_subscription(
            Image, 'top_camera/image_rect', self.top_image_callback, 10)
        
        self.detection_subscription = self.create_subscription(
            YoloDetection, 'bottom_camera/detections', self.detection_callback, 10)
        
        # 创建定时器，定期进行物体位置计算
        self.timer = self.create_timer(0.1, self.localize_object)
        
        self.get_logger().info('物体定位器节点已初始化')
    
    def bottom_image_callback(self, msg):
        """处理底部摄像头图像"""
        try:
            self.bottom_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.bottom_image_updated = True
        except Exception as e:
            self.get_logger().error(f'处理底部摄像头图像错误: {e}')
    
    def top_image_callback(self, msg):
        """处理顶部摄像头图像"""
        try:
            self.top_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.top_image_updated = True
        except Exception as e:
            self.get_logger().error(f'处理顶部摄像头图像错误: {e}')
    
    def detection_callback(self, msg):
        """处理YOLO检测结果"""
        self.current_detection = msg
        self.detection_updated = True
    
    def localize_object(self):
        """主要的物体定位算法，结合两个相机图像和检测结果进行三角测量"""
        # 检查所有需要的数据是否可用
        if (not self.bottom_image_updated or 
            not self.top_image_updated or 
            not self.detection_updated or
            self.bottom_image is None or
            self.top_image is None or
            self.current_detection is None):
            return
        
        # 创建并初始化物体位置消息
        position_msg = ObjectPosition()
        position_msg.detected = False
        position_msg.x = 0.0
        position_msg.y = 0.0
        position_msg.z = 0.0
        position_msg.confidence = 0.0
        
        # 检查是否检测到物体
        if self.current_detection.num_objects == 0:
            self.position_publisher.publish(position_msg)
            return
        
        try:
            # 提取检测框区域
            x = self.current_detection.x
            y = self.current_detection.y
            width = self.current_detection.width
            height = self.current_detection.height
            
            # 确保检测框不超出图像边界
            if (x < 0 or y < 0 or 
                x + width > self.bottom_image.shape[1] or 
                y + height > self.bottom_image.shape[0]):
                self.get_logger().warn('检测框超出图像边界')
                self.position_publisher.publish(position_msg)
                return
            
            # 从底部摄像头检测框中提取ROI
            bottom_roi = self.bottom_image[y:y+height, x:x+width]
            
            # 检测特征点
            bottom_keypoints, bottom_descriptors = self.feature_detector.detectAndCompute(bottom_roi, None)
            
            if bottom_descriptors is None or len(bottom_keypoints) < 4:
                self.get_logger().warn('底部图像中找不到足够的特征点')
                self.position_publisher.publish(position_msg)
                return
            
            # 在顶部摄像头图像中检测特征
            top_keypoints, top_descriptors = self.feature_detector.detectAndCompute(self.top_image, None)
            
            if top_descriptors is None or len(top_keypoints) < 4:
                self.get_logger().warn('顶部图像中找不到足够的特征点')
                self.position_publisher.publish(position_msg)
                return
            
            # 特征匹配
            matches = self.feature_matcher.knnMatch(bottom_descriptors, top_descriptors, k=2)
            
            # 筛选好的匹配
            good_matches = []
            for m, n in matches:
                if m.distance < self.feature_match_ratio * n.distance:
                    good_matches.append(m)
            
            if len(good_matches) < 4:
                self.get_logger().warn(f'找不到足够的特征匹配点: {len(good_matches)}')
                self.position_publisher.publish(position_msg)
                return
            
            # 提取匹配点坐标
            bottom_pts = []
            top_pts = []
            
            for match in good_matches:
                # 注意需要加上ROI的偏移
                bottom_pt = bottom_keypoints[match.queryIdx].pt
                bottom_pts.append((bottom_pt[0] + x, bottom_pt[1] + y))
                
                top_pt = top_keypoints[match.trainIdx].pt
                top_pts.append(top_pt)
            
            # 使用几何变换找到一个大致的物体位置
            bottom_pts = np.array(bottom_pts)
            top_pts = np.array(top_pts)
            
            # 计算两组点的中心
            bottom_center = np.mean(bottom_pts, axis=0)
            top_center = np.mean(top_pts, axis=0)
            
            # 三角测量 - 首先需要获取摄像头的相对位置
            try:
                # 获取两个摄像头到base_link的变换
                bottom_to_base = self.tf_buffer.lookup_transform(
                    'base_link', 'bottom_camera_link', rclpy.time.Time())
                    
                top_to_base = self.tf_buffer.lookup_transform(
                    'base_link', 'top_camera_link', rclpy.time.Time())
                
                # 计算相对于摄像头的射线
                bottom_ray = self.pixel_to_ray(
                    bottom_center[0], bottom_center[1], 
                    self.bottom_camera_matrix)
                    
                top_ray = self.pixel_to_ray(
                    top_center[0], top_center[1], 
                    self.top_camera_matrix)
                
                # 将射线表示为相对于base_link的点
                bottom_point_msg = PointStamped()
                bottom_point_msg.header.frame_id = 'bottom_camera_link'
                bottom_point_msg.point.x = bottom_ray[0]
                bottom_point_msg.point.y = bottom_ray[1]
                bottom_point_msg.point.z = bottom_ray[2]
                
                top_point_msg = PointStamped()
                top_point_msg.header.frame_id = 'top_camera_link'
                top_point_msg.point.x = top_ray[0]
                top_point_msg.point.y = top_ray[1]
                top_point_msg.point.z = top_ray[2]
                
                # 转换到base_link坐标系
                bottom_ray_base = self.tf_buffer.transform(bottom_point_msg, 'base_link')
                top_ray_base = self.tf_buffer.transform(top_point_msg, 'base_link')
                
                # 提取射线向量
                bottom_ray_vector = np.array([
                    bottom_ray_base.point.x,
                    bottom_ray_base.point.y,
                    bottom_ray_base.point.z
                ])
                
                top_ray_vector = np.array([
                    top_ray_base.point.x,
                    top_ray_base.point.y,
                    top_ray_base.point.z
                ])
                
                # 提取摄像头位置
                bottom_pos = np.array([
                    bottom_to_base.transform.translation.x,
                    bottom_to_base.transform.translation.y,
                    bottom_to_base.transform.translation.z
                ])
                
                top_pos = np.array([
                    top_to_base.transform.translation.x,
                    top_to_base.transform.translation.y,
                    top_to_base.transform.translation.z
                ])
                
                # 计算两条射线的最近点 (三角测量)
                object_position, confidence = self.find_closest_point_between_rays(
                    bottom_pos, bottom_ray_vector, 
                    top_pos, top_ray_vector)
                
                # 更新位置消息
                if confidence > self.confidence_threshold:
                    position_msg.detected = True
                    position_msg.x = float(object_position[0])
                    position_msg.y = float(object_position[1])
                    position_msg.z = float(object_position[2])
                    position_msg.confidence = float(confidence)
                else:
                    self.get_logger().warn(f'三角测量置信度低: {confidence}')
                
                # 发布物体位置
                self.position_publisher.publish(position_msg)
                
                # 发布调试图像
                if self.publish_debug_images:
                    self.publish_debug_image(bottom_pts, top_pts, good_matches)
                
            except Exception as e:
                self.get_logger().error(f'计算物体位置时出错: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                self.position_publisher.publish(position_msg)
                
        except Exception as e:
            self.get_logger().error(f'定位物体时出错: {e}')
            self.position_publisher.publish(position_msg)
    
    def pixel_to_ray(self, x, y, camera_matrix):
        """将像素坐标转换为相机坐标系中的单位方向向量"""
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        # 计算归一化的方向向量
        direction = np.array([(x - cx) / fx, (y - cy) / fy, 1.0])
        norm = np.linalg.norm(direction)
        
        # 返回单位长度的方向向量
        return direction / norm
    
    def find_closest_point_between_rays(self, p1, v1, p2, v2):
        """
        找到两条射线的最近点（最小二乘解）
        p1, p2: 射线起点
        v1, v2: 射线方向（单位向量）
        返回: (最近点坐标, 置信度)
        """
        # 归一化方向向量
        v1 = v1 / np.linalg.norm(v1)
        v2 = v2 / np.linalg.norm(v2)
        
        # 计算点-向量方程的系数
        A = np.array([[np.dot(v1, v1), -np.dot(v1, v2)],
                      [np.dot(v1, v2), -np.dot(v2, v2)]])
                      
        b = np.array([np.dot(v1, p2-p1),
                      np.dot(v2, p2-p1)])
        
        # 解线性方程以得到最佳的缩放因子
        try:
            t = np.linalg.solve(A, b)
            
            # 使用缩放因子计算两条线上的点
            point1 = p1 + t[0] * v1
            point2 = p2 + t[1] * v2
            
            # 计算两条线最近点的中点
            closest_point = (point1 + point2) / 2
            
            # 计算最近点之间的距离作为置信度指标
            distance = np.linalg.norm(point2 - point1)
            confidence = 1.0 / (1.0 + distance)  # 距离越小，置信度越高
            
            return closest_point, confidence
            
        except np.linalg.LinAlgError:
            # 处理平行线的情况
            self.get_logger().warn('射线几乎平行，无法可靠地计算交点')
            return np.zeros(3), 0.0
    
    def publish_debug_image(self, bottom_pts, top_pts, matches):
        """生成并发布调试图像，显示特征匹配结果"""
        if self.bottom_image is None or self.top_image is None:
            return
            
        try:
            # 调整两个图像为相同大小
            h1, w1 = self.bottom_image.shape[:2]
            h2, w2 = self.top_image.shape[:2]
            
            # 确定拼接图像尺寸
            height = max(h1, h2)
            width = w1 + w2
            
            # 创建拼接图像
            vis = np.zeros((height, width, 3), np.uint8)
            vis[:h1, :w1] = self.bottom_image
            vis[:h2, w1:w1+w2] = self.top_image
            
            # 绘制检测框
            if self.current_detection.num_objects > 0:
                cv2.rectangle(
                    vis, 
                    (self.current_detection.x, self.current_detection.y),
                    (self.current_detection.x + self.current_detection.width, 
                     self.current_detection.y + self.current_detection.height),
                    (0, 255, 0), 2)
            
            # 绘制特征匹配线
            for i, (bp, tp) in enumerate(zip(bottom_pts, top_pts)):
                if i > 20:  # 限制显示的线数量
                    break
                    
                # 添加w1偏移到顶部摄像头坐标
                tp_offset = (int(tp[0]) + w1, int(tp[1]))
                
                # 画线
                cv2.line(vis, (int(bp[0]), int(bp[1])), tp_offset, (0, 255, 0), 1)
                
                # 给特征点标记
                cv2.circle(vis, (int(bp[0]), int(bp[1])), 4, (255, 0, 0), -1)
                cv2.circle(vis, tp_offset, 4, (0, 0, 255), -1)
            
            # 添加文本说明
            cv2.putText(vis, "Bottom Camera", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(vis, "Top Camera", (w1 + 10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            
            # 发布调试图像
            debug_msg = self.bridge.cv2_to_imgmsg(vis, "bgr8")
            self.debug_image_publisher.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布调试图像出错: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，关闭节点')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
