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
    通过模板匹配和三角测量计算物体的3D坐标
    """

    def __init__(self):
        super().__init__('object_localizer_node')
        
        # ----------------------- 参数声明及说明 -----------------------
        
        # 基本参数
        # confidence_threshold: 三角测量的置信度阈值，越高要求精度越高，但可能导致有些物体无法定位
        # 范围: 0.0-1.0，建议值: 0.5-0.7
        self.declare_parameter('confidence_threshold', 0.65)
        
        # show_debug_visualization: 是否显示调试可视化窗口
        # True: 显示特征匹配可视化窗口（有助于调试）
        # False: 不显示可视化窗口（适合部署环境）
        self.declare_parameter('show_debug_visualization', True)
        
        # 模板匹配参数
        # template_match_method：模板匹配方法，可选值包括：
        # 0: TM_SQDIFF - 平方差匹配法，值越小越相似
        # 1: TM_SQDIFF_NORMED - 归一化平方差匹配法，值越小越相似
        # 2: TM_CCORR - 相关匹配法，值越大越相似
        # 3: TM_CCORR_NORMED - 归一化相关匹配法，值越大越相似
        # 4: TM_CCOEFF - 相关系数匹配法，值越大越相似
        # 5: TM_CCOEFF_NORMED - 归一化相关系数匹配法，值越大越相似 (推荐)
        # 范围: 0-5，建议值: 5
        self.declare_parameter('template_match_method', 5)
        
        # template_match_threshold：模板匹配阈值，用于筛选匹配结果
        # 对于TM_SQDIFF和TM_SQDIFF_NORMED，值越小越好，阈值应为上限
        # 对于其他方法，值越大越好，阈值应为下限
        # 范围: 0.0-1.0，建议值: 0.5-0.8
        self.declare_parameter('template_match_threshold', 0.61)
        
        # template_search_scale：模板匹配搜索比例，用于限制搜索区域
        # 值为1.0表示在整个顶部图像中搜索
        # 值为0.5表示只在顶部图像中间的一半区域搜索
        # 较小的值可提高匹配速度，但可能错过正确匹配
        # 范围: 0.1-1.0，建议值: 0.7-1.0
        self.declare_parameter('template_search_scale', 1.0)
        
        # 确保这两行在使用之前就添加
        self.declare_parameter('template_scale_range', 0.2)
        self.declare_parameter('template_scale_steps', 3)
        
        # 垂直位置约束参数
        # vertical_position_check: 是否启用垂直位置检查
        # 启用后，上方摄像头匹配框的中心y坐标必须比下方摄像头检测框的中心y坐标大
        # 这符合物理直觉，因为上方摄像头看到的物体通常会在图像中偏下方
        # True: 启用位置约束 (推荐)
        # False: 禁用位置约束
        self.declare_parameter('vertical_position_check', True)
        
        # 色彩相似度检查参数
        # color_similarity_check: 是否启用颜色相似度检查
        # 启用后，会比较两个检测框中心区域的颜色相似度，过滤掉不匹配的物体
        # True: 启用颜色相似度检查
        # False: 禁用颜色相似度检查
        self.declare_parameter('color_similarity_check', True)
        
        # color_similarity_threshold: 颜色相似度阈值
        # 两个区域的颜色相似度必须高于此阈值才被视为匹配
        # 范围: 0.0-1.0，建议值: 0.7-0.85
        self.declare_parameter('color_similarity_threshold', 0.85)
        
        # color_sample_size: 颜色采样大小
        # 从检测框中心提取多大区域的颜色样本进行比较
        # 较大的值考虑更多像素，较小的值更聚焦于中心
        # 范围: 3-21，建议值: 5-11 (应为奇数)
        self.declare_parameter('color_sample_size', 7)
        
        # top_match_count: 顶部搜索结果数量
        # 在满足阈值的情况下，考虑多少个可能的匹配结果
        # 较大的值允许尝试更多可能的匹配，但可能增加错误匹配风险
        # 范围: 1-10，建议值: 3-5
        self.declare_parameter('top_match_count', 3)
        
        # ----------------------- 参数获取 -----------------------
        # 获取参数
        # 先得到基本参数，然后再获取可能导致问题的参数
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.show_debug_visualization = self.get_parameter('show_debug_visualization').value
        
        # 获取模板匹配参数
        self.template_match_method = self.get_parameter('template_match_method').value
        self.template_match_threshold = self.get_parameter('template_match_threshold').value
        self.template_search_scale = self.get_parameter('template_search_scale').value
        self.template_scale_range = self.get_parameter('template_scale_range').value
        self.template_scale_steps = self.get_parameter('template_scale_steps').value
        
        # 获取过滤参数
        self.vertical_position_check = self.get_parameter('vertical_position_check').value
        self.color_similarity_check = self.get_parameter('color_similarity_check').value
        self.color_similarity_threshold = self.get_parameter('color_similarity_threshold').value
        self.color_sample_size = self.get_parameter('color_sample_size').value
        self.top_match_count = self.get_parameter('top_match_count').value
        
        # 将匹配方法参数转换为OpenCV常量
        self.match_method_map = {
            0: cv2.TM_SQDIFF,
            1: cv2.TM_SQDIFF_NORMED,
            2: cv2.TM_CCORR,
            3: cv2.TM_CCORR_NORMED,
            4: cv2.TM_CCOEFF,
            5: cv2.TM_CCOEFF_NORMED
        }
        self.match_method = self.match_method_map.get(self.template_match_method, cv2.TM_CCOEFF_NORMED)
        
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
    
    def do_template_matching(self, template, search_image, template_w, template_h):
        """执行模板匹配，返回多个可能的匹配位置和置信度"""
        all_matches = []  # 存储所有可能的匹配
        
        # 计算缩放范围
        if self.template_scale_range <= 0:
            # 如果没有设置缩放范围，只使用原始尺寸
            scales = [1.0]
        else:
            # 根据缩放范围和步数计算不同的缩放尺度
            min_scale = 1.0 - self.template_scale_range
            max_scale = 1.0 + self.template_scale_range
            scale_step = (max_scale - min_scale) / max(1, self.template_scale_steps - 1)
            scales = [min_scale + i * scale_step for i in range(self.template_scale_steps)]
        
        # 计算搜索区域（可能只使用图像的一部分）
        h, w = search_image.shape[:2]
        center_x, center_y = w // 2, h // 2
        search_w = int(w * self.template_search_scale)
        search_h = int(h * self.template_search_scale)
        x1 = max(0, center_x - search_w // 2)
        y1 = max(0, center_y - search_h // 2)
        x2 = min(w, center_x + search_w // 2)
        y2 = min(h, center_y + search_h // 2)
        
        # 提取搜索区域
        search_roi = search_image[y1:y2, x1:x2]
        
        # 在不同尺度上进行模板匹配
        for scale in scales:
            # 缩放模板
            if scale != 1.0:
                scaled_w = int(template_w * scale)
                scaled_h = int(template_h * scale)
                if scaled_w <= 0 or scaled_h <= 0:
                    continue
                scaled_template = cv2.resize(template, (scaled_w, scaled_h))
            else:
                scaled_template = template
                scaled_w = template_w
                scaled_h = template_h
            
            # 如果模板大于搜索区域，跳过
            if scaled_template.shape[0] > search_roi.shape[0] or scaled_template.shape[1] > search_roi.shape[1]:
                continue
            
            # 执行模板匹配
            result = cv2.matchTemplate(search_roi, scaled_template, self.match_method)
            
            # 获取所有可能的匹配位置
            # 创建掩码用于屏蔽已找到的匹配区域
            mask = np.ones_like(result, dtype=bool)
            
            for _ in range(self.top_match_count):
                # 在掩码区域内找到最佳匹配
                if self.match_method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                    # 越小越好的匹配方法
                    masked_result = np.where(mask, result, np.inf)
                    match_val = np.min(masked_result)
                    if match_val == np.inf:
                        break
                    y_match, x_match = np.unravel_index(np.argmin(masked_result), result.shape)
                    # 将匹配值转换为相似度（1-差异）
                    match_confidence = 1.0 - match_val
                else:
                    # 越大越好的匹配方法
                    masked_result = np.where(mask, result, -np.inf)
                    match_val = np.max(masked_result)
                    if match_val == -np.inf:
                        break
                    y_match, x_match = np.unravel_index(np.argmax(masked_result), result.shape)
                    match_confidence = match_val
                
                # 调整到原始图像坐标系
                match_x = x_match + x1
                match_y = y_match + y1
                
                # 将结果添加到列表
                all_matches.append((match_x, match_y, scaled_w, scaled_h, match_confidence, scale))
                
                # 更新掩码，屏蔽当前匹配位置及其周围区域
                y_start = max(0, y_match - scaled_h // 2)
                y_end = min(result.shape[0], y_match + scaled_h // 2)
                x_start = max(0, x_match - scaled_w // 2)
                x_end = min(result.shape[1], x_match + scaled_w // 2)
                mask[y_start:y_end, x_start:x_end] = False
        
        # 根据匹配值排序（从高到低）
        if self.match_method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            # 低值更好的方法，但我们已经转换为置信度，所以是高值更好
            all_matches.sort(key=lambda x: x[4], reverse=True)
        else:
            # 高值更好的方法
            all_matches.sort(key=lambda x: x[4], reverse=True)
        
        return all_matches
    
    def calculate_color_similarity(self, img1, center1, img2, center2, size):
        """计算两个图像中心区域的颜色相似度（忽略亮度）"""
        try:
            # 确保size是奇数
            if size % 2 == 0:
                size += 1
            
            half = size // 2
            
            # 提取中心区域
            x1, y1 = int(center1[0]), int(center1[1])
            x2, y2 = int(center2[0]), int(center2[1])
            
            # 确保区域在图像内部
            if (x1 - half < 0 or x1 + half + 1 > img1.shape[1] or
                y1 - half < 0 or y1 + half + 1 > img1.shape[0] or
                x2 - half < 0 or x2 + half + 1 > img2.shape[1] or
                y2 - half < 0 or y2 + half + 1 > img2.shape[0]):
                return 0.0
            
            # 提取区域
            region1 = img1[y1-half:y1+half+1, x1-half:x1+half+1]
            region2 = img2[y2-half:y2+half+1, x2-half:x2+half+1]
            
            # 转换为HSV颜色空间（分离色彩和亮度）
            hsv1 = cv2.cvtColor(region1, cv2.COLOR_BGR2HSV)
            hsv2 = cv2.cvtColor(region2, cv2.COLOR_BGR2HSV)
            
            # 只比较H和S通道（忽略V亮度通道）
            h1, s1, _ = cv2.split(hsv1)
            h2, s2, _ = cv2.split(hsv2)
            
            # 计算H和S通道的直方图
            hist1_h = cv2.calcHist([h1], [0], None, [30], [0, 180])
            hist1_s = cv2.calcHist([s1], [0], None, [32], [0, 256])
            hist2_h = cv2.calcHist([h2], [0], None, [30], [0, 180])
            hist2_s = cv2.calcHist([s2], [0], None, [32], [0, 256])
            
            # 归一化直方图
            cv2.normalize(hist1_h, hist1_h, 0, 1, cv2.NORM_MINMAX)
            cv2.normalize(hist1_s, hist1_s, 0, 1, cv2.NORM_MINMAX)
            cv2.normalize(hist2_h, hist2_h, 0, 1, cv2.NORM_MINMAX)
            cv2.normalize(hist2_s, hist2_s, 0, 1, cv2.NORM_MINMAX)
            
            # 比较直方图相似度
            h_correlation = cv2.compareHist(hist1_h, hist2_h, cv2.HISTCMP_CORREL)
            s_correlation = cv2.compareHist(hist1_s, hist2_s, cv2.HISTCMP_CORREL)
            
            # 计算综合相似度（H通道权重更高）
            similarity = 0.7 * h_correlation + 0.3 * s_correlation
            
            return similarity
            
        except Exception as e:
            self.get_logger().error(f'计算颜色相似度出错: {e}')
            return 0.0
    
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
        position_msg.z = 0.0  # 没有检测到时为0
        position_msg.confidence = 0.0
        
        # 检查是否检测到物体
        if self.current_detection.num_objects == 0:
            # 即使没有检测到物体也可以显示空白的调试图像
            if self.show_debug_visualization:
                self.publish_debug_image(0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0)
            self.position_publisher.publish(position_msg)
            return
        
        try:
            # 提取检测框区域 (需要考虑图像已被缩放)
            x_bottom = int(self.current_detection.x * self.scale_factor)
            y_bottom = int(self.current_detection.y * self.scale_factor)
            width_bottom = int(self.current_detection.width * self.scale_factor)
            height_bottom = int(self.current_detection.height * self.scale_factor)
            
            # 计算检测框周长
            detection_perimeter = 2 * (width_bottom + height_bottom)
            
            # 计算底部检测框中心点
            bottom_center_x = x_bottom + width_bottom / 2
            bottom_center_y = y_bottom + height_bottom / 2
            
            # 确保检测框不超出图像边界
            if (x_bottom < 0 or y_bottom < 0 or 
                x_bottom + width_bottom > self.bottom_image.shape[1] or 
                y_bottom + height_bottom > self.bottom_image.shape[0]):
                self.get_logger().warn('检测框超出图像边界')
                if self.show_debug_visualization:
                    self.publish_debug_image(x_bottom, y_bottom, width_bottom, height_bottom, 0, 0, 0, 0, 0, 0.0)
                # 检测框无效时z=0
                position_msg.z = 0.0
                self.position_publisher.publish(position_msg)
                return
            
            # 从底部摄像头检测框中提取ROI作为模板
            template = self.bottom_image[y_bottom:y_bottom+height_bottom, x_bottom:x_bottom+width_bottom]
            
            # 如果模板区域太小，可能无法进行有效的模板匹配
            if template.shape[0] < 10 or template.shape[1] < 10:
                self.get_logger().warn(f'检测框太小: {template.shape}')
                if self.show_debug_visualization:
                    self.publish_debug_image(x_bottom, y_bottom, width_bottom, height_bottom, 0, 0, 0, 0, 0, 0.0)
                # 检测框太小时z=0
                position_msg.z = 0.0
                self.position_publisher.publish(position_msg)
                return
            
            # 在顶部摄像头图像中执行模板匹配，获取多个可能的匹配
            matches = self.do_template_matching(template, self.top_image, width_bottom, height_bottom)
            
            if not matches:
                self.get_logger().warn('未找到可能的模板匹配')
                if self.show_debug_visualization:
                    self.publish_debug_image(x_bottom, y_bottom, width_bottom, height_bottom, 0, 0, 0, 0, 0, 0.0)
                # 未找到匹配时z=0
                position_msg.z = 0.0
                self.position_publisher.publish(position_msg)
                return
            
            # 遍历匹配结果，找到满足过滤条件的第一个结果
            valid_match_found = False
            valid_match = None
            color_similarity = 0.0
            
            for match in matches:
                match_x, match_y, width_top, height_top, match_confidence, scale = match
                
                # 检查是否满足匹配阈值
                threshold_check = match_confidence < self.template_match_threshold if self.match_method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED] else match_confidence > self.template_match_threshold
                if not threshold_check:
                    continue
                
                # 计算顶部匹配框的中心点
                top_center_x = match_x + width_top / 2
                top_center_y = match_y + height_top / 2
                
                # 垂直位置检查：顶部摄像头的匹配框中心应该在图像中更低的位置
                vertical_check = True
                if self.vertical_position_check:
                    # 检查顶部匹配的y坐标是否大于底部检测的y坐标
                    # (y坐标越大，在图像中位置越低)
                    vertical_check = top_center_y >= bottom_center_y
                    if not vertical_check:
                        self.get_logger().debug(f'垂直位置检查失败: 上 {top_center_y} vs 下 {bottom_center_y}')
                        continue
                
                # 颜色相似度检查
                color_check = True
                if self.color_similarity_check:
                    # 计算两个区域中心的颜色相似度
                    color_similarity = self.calculate_color_similarity(
                        self.top_image, (top_center_x, top_center_y),
                        self.bottom_image, (bottom_center_x, bottom_center_y),
                        self.color_sample_size
                    )
                    color_check = color_similarity >= self.color_similarity_threshold
                    if not color_check:
                        self.get_logger().debug(f'颜色相似度检查失败: {color_similarity:.4f} < {self.color_similarity_threshold}')
                        continue
                
                # 如果通过所有检查，保存这个匹配并退出循环
                valid_match = match
                valid_match_found = True
                break
            
            # 如果没有找到有效匹配，输出警告
            if not valid_match_found:
                self.get_logger().warn('没有找到满足过滤条件的匹配')
                # 显示第一个匹配结果，但标记为无效
                if self.show_debug_visualization and matches:
                    match_x, match_y, width_top, height_top, match_confidence, _ = matches[0]
                    self.publish_debug_image(
                        x_bottom, y_bottom, width_bottom, height_bottom,
                        match_x, match_y, width_top, height_top, 
                        match_confidence, color_similarity, False)
                # 没有找到有效匹配时z=0
                position_msg.z = 0.0
                self.position_publisher.publish(position_msg)
                return
            
            # 提取有效匹配结果
            match_x, match_y, width_top, height_top, match_confidence, _ = valid_match
            
            # 计算检测框的中心点（已经在上面计算过底部的中心点）
            top_center_x = match_x + width_top / 2
            top_center_y = match_y + height_top / 2
            
            # 三角测量 - 首先需要获取摄像头的相对位置
            try:
                # 获取两个摄像头到base_link的变换
                bottom_to_base = self.tf_buffer.lookup_transform(
                    'base_link', 'bottom_camera_link', rclpy.time.Time())
                    
                top_to_base = self.tf_buffer.lookup_transform(
                    'base_link', 'top_camera_link', rclpy.time.Time())
                
                # 计算相对于摄像头的射线 - 使用缩放后的内参矩阵
                bottom_ray = self.pixel_to_ray(
                    bottom_center_x, bottom_center_y, 
                    self.bottom_camera_matrix_scaled)
                    
                top_ray = self.pixel_to_ray(
                    top_center_x, top_center_y, 
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
                object_position, triangulation_confidence = self.find_closest_point_between_rays(
                    bottom_pos, bottom_ray_vector, 
                    top_pos, top_ray_vector)
                
                # 结合模板匹配置信度和三角测量置信度
                overall_confidence = triangulation_confidence * match_confidence
                if self.color_similarity_check:
                    # 如果启用了颜色检查，也将其纳入总体置信度
                    overall_confidence *= color_similarity
                
                # 更新位置消息
                if overall_confidence > self.confidence_threshold:
                    position_msg.detected = True
                    position_msg.x = float(object_position[0])
                    position_msg.y = float(object_position[1])
                    position_msg.z = float(detection_perimeter)  # 使用检测框周长作为z值
                    position_msg.confidence = float(overall_confidence)
                else:
                    self.get_logger().warn(f'三角测量置信度低: {overall_confidence}')
                    # 置信度低时z=0
                    position_msg.z = 0.0
                
                # 发布物体位置
                self.position_publisher.publish(position_msg)
                
                # 显示调试图像
                if self.show_debug_visualization:
                    self.publish_debug_image(
                        x_bottom, y_bottom, width_bottom, height_bottom, 
                        match_x, match_y, width_top, height_top, 
                        match_confidence, color_similarity, True)
                
            except Exception as e:
                self.get_logger().error(f'计算物体位置时出错: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                # 尽管出错，仍然显示匹配结果，但z=0
                position_msg.z = 0.0
                if self.show_debug_visualization:
                    self.publish_debug_image(
                        x_bottom, y_bottom, width_bottom, height_bottom, 
                        match_x, match_y, width_top, height_top, 
                        match_confidence, color_similarity, False)
                self.position_publisher.publish(position_msg)
                
        except Exception as e:
            self.get_logger().error(f'定位物体时出错: {e}')
            # 在任何错误情况下，尝试显示调试信息，z=0
            position_msg.z = 0.0
            if self.show_debug_visualization:
                self.publish_debug_image(0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0)
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
    
    def publish_debug_image(self, x_bottom, y_bottom, width_bottom, height_bottom, 
                           x_top, y_top, width_top, height_top, match_confidence, 
                           color_similarity=0.0, valid_match=True):
        """生成并显示调试图像，显示模板匹配结果"""
        if not self.show_debug_visualization or self.bottom_image is None or self.top_image is None:
            return
            
        try:
            # 获取图像尺寸
            h_bottom, w_bottom = self.bottom_image.shape[:2]
            h_top, w_top = self.top_image.shape[:2]
            
            # 垂直拼接图像 (先上方，后下方)
            width = max(w_top, w_bottom)
            height = h_top + h_bottom
            
            # 创建拼接图像
            vis = np.zeros((height, width, 3), np.uint8)
            vis[:h_top, :w_top] = self.top_image
            vis[h_top:h_top+h_bottom, :w_bottom] = self.bottom_image
            
            # 检测框颜色 - 有效匹配为绿色，无效匹配为红色
            box_color = (0, 255, 0) if valid_match else (0, 0, 255)
            
            # 绘制顶部摄像头的匹配框
            if width_top > 0 and height_top > 0:
                cv2.rectangle(
                    vis, 
                    (x_top, y_top),
                    (x_top + width_top, y_top + height_top),
                    box_color, 2)
                
                # 绘制顶部框中心十字线
                center_x = int(x_top + width_top / 2)
                center_y = int(y_top + height_top / 2)
                cv2.line(vis, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 2)
                cv2.line(vis, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 2)
                
                # 如果启用了颜色相似度检查，绘制中心采样区域
                if self.color_similarity_check:
                    half = self.color_sample_size // 2
                    cv2.rectangle(
                        vis, 
                        (center_x - half, center_y - half),
                        (center_x + half, center_y + half),
                        (255, 255, 0), 1)
            
            # 绘制底部摄像头的检测框
            if width_bottom > 0 and height_bottom > 0:
                cv2.rectangle(
                    vis, 
                    (x_bottom, y_bottom + h_top),  # 注意坐标偏移
                    (x_bottom + width_bottom, y_bottom + height_bottom + h_top),
                    box_color, 2)
                
                # 绘制底部框中心十字线
                center_x = int(x_bottom + width_bottom / 2)
                center_y = int(y_bottom + height_bottom / 2 + h_top)
                cv2.line(vis, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 2)
                cv2.line(vis, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 2)
                
                # 如果启用了颜色相似度检查，绘制中心采样区域
                if self.color_similarity_check:
                    half = self.color_sample_size // 2
                    cv2.rectangle(
                        vis, 
                        (center_x - half, center_y - half),
                        (center_x + half, center_y + half),
                        (255, 255, 0), 1)
            
            # 添加文本说明
            cv2.putText(vis, "Top Camera", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(vis, "Bottom Camera", (10, h_top + 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            
            # 显示匹配置信度
            match_text = f"Match: {match_confidence:.3f}"
            cv2.putText(vis, match_text, (10, h_top - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
            
            # 如果启用了颜色相似度检查，显示颜色相似度
            if self.color_similarity_check:
                color_text = f"Color Sim: {color_similarity:.3f}"
                cv2.putText(vis, color_text, (10, h_top - 10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
                
            # 使用cv2.imshow()显示结果
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
