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
        
        # ----------------------- 参数声明及说明 -----------------------
        
        # 基本参数
        # confidence_threshold: 三角测量的置信度阈值，越高要求精度越高，但可能导致有些物体无法定位
        # 范围: 0.0-1.0，建议值: 0.5-0.7
        self.declare_parameter('confidence_threshold', 0.65)
        
        # feature_match_ratio: 特征点匹配距离比率，用于kNN匹配过滤
        # 值越小，匹配越严格（更少但更可靠的匹配点）
        # 值越大，匹配越宽松（更多但可能更不准确的匹配点）
        # 范围: 0.0-1.0，建议值: 0.6-0.8
        self.declare_parameter('feature_match_ratio', 0.9)
        
        # max_feature_points: 最大特征点数量，限制用于匹配的特征点数量
        # 值越大，计算量越大但精度可能更高
        # 值越小，计算更快但可能降低精度
        # 范围: 10-1000，建议值: 50-200
        self.declare_parameter('max_feature_points', 100)
        
        # show_debug_visualization: 是否显示调试可视化窗口
        # True: 显示特征匹配可视化窗口（有助于调试）
        # False: 不显示可视化窗口（适合部署环境）
        self.declare_parameter('show_debug_visualization', True)
        
        # SIFT参数（影响特征点检测的质量和数量）
        # sift_n_features: SIFT算法检测的最大特征点数
        # 值越大，检测更多特征点，增加匹配可能，但计算量更大
        # 范围: 10-2000，建议值: 100-500
        self.declare_parameter('sift_n_features', 300)
        
        # sift_n_octave_layers: SIFT算法中每个八度层的数量
        # 值越大，检测到的特征点尺度多样性越高，但计算量更大
        # 范围: 1-5，建议值: 3
        self.declare_parameter('sift_n_octave_layers', 3)
        
        # sift_contrast_threshold: 特征点对比度阈值
        # 值越小，检测低对比度区域的特征点，但可能引入噪声
        # 值越大，只检测高对比度区域的特征点，减少但更可靠
        # 范围: 0.01-0.1，建议值: 0.04
        self.declare_parameter('sift_contrast_threshold', 0.04)
        
        # sift_edge_threshold: 边缘响应阈值，用于过滤边缘上的点
        # 值越小，边缘过滤越严格，检测特征点会偏向于角点
        # 值越大，边缘过滤越宽松，会在边缘上检测到更多特征点
        # 范围: 5-20，建议值: 10
        self.declare_parameter('sift_edge_threshold', 15)
        
        # sift_sigma: SIFT高斯滤波器的sigma值
        # 控制特征点检测的空间尺度，影响特征点的大小敏感性
        # 范围: 1.0-2.0，建议值: 1.6
        self.declare_parameter('sift_sigma', 1.6)
        
        # 特征匹配参数
        # match_k: kNN匹配时查找的近邻数量
        # 通常设为2，用于比率测试；若设为1则变为最近邻匹配
        # 范围: 1-3，建议值: 2
        self.declare_parameter('match_k', 2)
        
        # match_cross_check: 是否使用交叉检查进行匹配
        # True: 使用交叉检查（更可靠但匹配点更少）
        # False: 不使用交叉检查（匹配点较多但可能有误匹配）
        # 注意: 启用交叉检查时，feature_match_ratio参数将被忽略
        self.declare_parameter('match_cross_check', False)
        
        # ----------------------- 参数获取 -----------------------
        # 获取参数
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.feature_match_ratio = self.get_parameter('feature_match_ratio').value
        self.max_feature_points = self.get_parameter('max_feature_points').value
        self.show_debug_visualization = self.get_parameter('show_debug_visualization').value
        
        # 获取SIFT参数
        self.sift_n_features = self.get_parameter('sift_n_features').value
        self.sift_n_octave_layers = self.get_parameter('sift_n_octave_layers').value
        self.sift_contrast_threshold = self.get_parameter('sift_contrast_threshold').value
        self.sift_edge_threshold = self.get_parameter('sift_edge_threshold').value
        self.sift_sigma = self.get_parameter('sift_sigma').value
        
        # 获取匹配参数
        self.match_k = self.get_parameter('match_k').value
        self.match_cross_check = self.get_parameter('match_cross_check').value
        
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
        
        # 调整内参矩阵适应缩放后的图像
        self.scale_factor = 0.5  # 640x480 -> 320x240
        self.bottom_camera_matrix_scaled = self.bottom_camera_matrix.copy()
        self.bottom_camera_matrix_scaled[:2, :] *= self.scale_factor
        
        self.top_camera_matrix_scaled = self.top_camera_matrix.copy()
        self.top_camera_matrix_scaled[:2, :] *= self.scale_factor
        
        # 特征检测器和描述子计算器
        self.feature_detector = cv2.SIFT_create(
            nfeatures=self.sift_n_features,
            nOctaveLayers=self.sift_n_octave_layers,
            contrastThreshold=self.sift_contrast_threshold,
            edgeThreshold=self.sift_edge_threshold,
            sigma=self.sift_sigma
        )
        
        # 根据参数设置特征匹配器
        if self.match_cross_check:
            self.feature_matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        else:
            self.feature_matcher = cv2.BFMatcher(cv2.NORM_L2)
        
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
        
        # 移除调试图像发布器，改用cv2.imshow()直接显示
        
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
        
        # 如果启用了可视化，创建一个命名窗口
        if self.show_debug_visualization:
            cv2.namedWindow('Object Localizer Visualization', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Object Localizer Visualization', 640, 480)
    
    def bottom_image_callback(self, msg):
        """处理底部摄像头图像"""
        try:
            # 将图像转换为OpenCV格式
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 将图像调整为320x240
            self.bottom_image = cv2.resize(img, (320, 240))
            self.bottom_image_updated = True
        except Exception as e:
            self.get_logger().error(f'处理底部摄像头图像错误: {e}')
    
    def top_image_callback(self, msg):
        """处理顶部摄像头图像"""
        try:
            # 将图像转换为OpenCV格式
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 将图像调整为320x240
            self.top_image = cv2.resize(img, (320, 240))
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
            # 即使没有检测到物体也可以显示空白的调试图像
            if self.show_debug_visualization:
                self.publish_debug_image([], [], [], 0, 0, 0, 0)
            self.position_publisher.publish(position_msg)
            return
        
        try:
            # 提取检测框区域 (需要考虑图像已被缩放)
            x = int(self.current_detection.x * self.scale_factor)
            y = int(self.current_detection.y * self.scale_factor)
            width = int(self.current_detection.width * self.scale_factor)
            height = int(self.current_detection.height * self.scale_factor)
            
            # 确保检测框不超出图像边界
            if (x < 0 or y < 0 or 
                x + width > self.bottom_image.shape[1] or 
                y + height > self.bottom_image.shape[0]):
                self.get_logger().warn('检测框超出图像边界')
                if self.show_debug_visualization:
                    self.publish_debug_image([], [], [], x, y, width, height)
                self.position_publisher.publish(position_msg)
                return
            
            # 从底部摄像头检测框中提取ROI
            bottom_roi = self.bottom_image[y:y+height, x:x+width]
            
            # 检测特征点
            bottom_keypoints, bottom_descriptors = self.feature_detector.detectAndCompute(bottom_roi, None)
            
            # 初始化点和匹配为空列表，以便在任何错误情况下仍能显示调试图像
            bottom_pts = []
            top_pts = []
            good_matches = []
            
            # 检查是否有足够的特征点
            if bottom_descriptors is None or len(bottom_keypoints) < 1:
                self.get_logger().warn('底部图像中找不到足够的特征点')
                if self.show_debug_visualization:
                    self.publish_debug_image(bottom_pts, top_pts, good_matches, x, y, width, height)
                self.position_publisher.publish(position_msg)
                return
            
            # 在顶部摄像头图像中检测特征
            top_keypoints, top_descriptors = self.feature_detector.detectAndCompute(self.top_image, None)
            
            if top_descriptors is None or len(top_keypoints) < 1:
                self.get_logger().warn('顶部图像中找不到足够的特征点')
                if self.show_debug_visualization:
                    self.publish_debug_image(bottom_pts, top_pts, good_matches, x, y, width, height)
                self.position_publisher.publish(position_msg)
                return
            
            # 特征匹配
            if self.match_cross_check:
                matches = self.feature_matcher.match(bottom_descriptors, top_descriptors)
                # 按距离排序
                matches = sorted(matches, key=lambda x: x.distance)
                # 选取前N个最佳匹配
                good_matches = matches[:min(len(matches), self.max_feature_points)]
            else:
                matches = self.feature_matcher.knnMatch(bottom_descriptors, top_descriptors, k=self.match_k)
                # 筛选好的匹配
                good_matches = []
                for m, n in matches:
                    if m.distance < self.feature_match_ratio * n.distance:
                        good_matches.append(m)
            
            # 提取匹配点坐标 (无论匹配点数量如何，都提取坐标用于显示)
            bottom_pts = []
            top_pts = []
            
            for match in good_matches:
                # 注意需要加上ROI的偏移
                bottom_pt = bottom_keypoints[match.queryIdx].pt
                bottom_pts.append((bottom_pt[0] + x, bottom_pt[1] + y))
                
                top_pt = top_keypoints[match.trainIdx].pt
                top_pts.append(top_pt)
            
            # 检查是否有足够的匹配点进行三角测量
            if len(good_matches) < 4:
                self.get_logger().warn(f'找不到足够的特征匹配点: {len(good_matches)}')
                # 显示目前已经匹配的点
                if self.show_debug_visualization:
                    self.publish_debug_image(bottom_pts, top_pts, good_matches, x, y, width, height)
                self.position_publisher.publish(position_msg)
                return
            
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
                
                # 计算相对于摄像头的射线 - 使用缩放后的内参矩阵
                bottom_ray = self.pixel_to_ray(
                    bottom_center[0], bottom_center[1], 
                    self.bottom_camera_matrix_scaled)
                    
                top_ray = self.pixel_to_ray(
                    top_center[0], top_center[1], 
                    self.top_camera_matrix_scaled)
                
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
                if self.show_debug_visualization:
                    self.publish_debug_image(bottom_pts, top_pts, good_matches, x, y, width, height)
                
            except Exception as e:
                self.get_logger().error(f'计算物体位置时出错: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                # 尽管出错，仍然显示已匹配的点
                if self.show_debug_visualization:
                    self.publish_debug_image(bottom_pts, top_pts, good_matches, x, y, width, height)
                self.position_publisher.publish(position_msg)
                
        except Exception as e:
            self.get_logger().error(f'定位物体时出错: {e}')
            # 在任何错误情况下，尝试显示调试信息
            if self.show_debug_visualization:
                self.publish_debug_image([], [], [], x, y, width, height)
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
    
    def publish_debug_image(self, bottom_pts, top_pts, matches, box_x, box_y, box_width, box_height):
        """生成并显示调试图像，显示特征匹配结果"""
        if not self.show_debug_visualization or self.bottom_image is None or self.top_image is None:
            return
            
        try:
            # 获取图像尺寸
            h1, w1 = self.bottom_image.shape[:2]
            h2, w2 = self.top_image.shape[:2]
            
            # 垂直拼接图像（上下），如用户要求
            width = max(w1, w2)
            height = h1 + h2
            
            # 创建拼接图像
            vis = np.zeros((height, width, 3), np.uint8)
            vis[:h1, :w1] = self.bottom_image
            vis[h1:h1+h2, :w2] = self.top_image
            
            # 绘制底部摄像头的检测框
            cv2.rectangle(
                vis, 
                (box_x, box_y),
                (box_x + box_width, box_y + box_height),
                (0, 255, 0), 2)
            
            # 绘制特征匹配线，即使没有足够的匹配点也绘制
            for i, (bp, tp) in enumerate(zip(bottom_pts, top_pts)):
                # 顶部摄像头的点需要加上h1的偏移
                tp_offset = (int(tp[0]), int(tp[1]) + h1)
                
                # 画线
                cv2.line(vis, (int(bp[0]), int(bp[1])), tp_offset, (0, 255, 0), 1)
                
                # 给特征点标记
                cv2.circle(vis, (int(bp[0]), int(bp[1])), 4, (255, 0, 0), -1)
                cv2.circle(vis, tp_offset, 4, (0, 0, 255), -1)
            
            # 添加文本说明
            cv2.putText(vis, "Bottom Camera", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(vis, "Top Camera", (10, h1 + 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            
            # 显示匹配数量信息
            cv2.putText(vis, f"Matches: {len(matches)}", (10, h1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 使用cv2.imshow()显示结果，而不是发布到ROS话题
            cv2.imshow('Object Localizer Visualization', vis)
            cv2.waitKey(1)  # 非阻塞更新
            
        except Exception as e:
            self.get_logger().error(f'显示调试图像出错: {e}')

    def __del__(self):
        """析构函数，确保在节点销毁时关闭OpenCV窗口"""
        if self.show_debug_visualization:
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，关闭节点')
    finally:
        # 确保在退出时关闭所有OpenCV窗口
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
