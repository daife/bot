#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo_detection_interfaces.msg import YoloDetection
import cv2
import numpy as np
import acl
import os
import time
import threading
from ament_index_python.packages import get_package_share_directory

# Import ACLLite modules
from acllite_utils import *
from constants import *
from acllite_imageproc import AclLiteImageProc
from acllite_model import AclLiteModel
from acllite_resource import resource_list

# Camera intrinsic parameters
CAMERA_MATRIX = np.array([
    [465.13093,   0.     , 324.81802],
    [  0.     , 466.33628, 242.54136],
    [  0.     ,   0.     ,   1.     ]
])
DISTORTION_COEFFS = np.array([-0.374992, 0.133505, 0.002906, -0.002975, 0.000000])
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

class AclLiteResource:
    """ACL资源管理类"""
    def __init__(self, device_id=0):
        self.device_id = device_id
        self.context = None
        self.stream = None
        
    def init(self):
        ret = acl.init()
        ret = acl.rt.set_device(self.device_id)
        self.context, ret = acl.rt.create_context(self.device_id)
        self.stream, ret = acl.rt.create_stream()
        return SUCCESS

    def __del__(self):
        resource_list.destroy()
        if self.stream:
            acl.rt.destroy_stream(self.stream)
        if self.context:
            acl.rt.destroy_context(self.context)
        acl.rt.reset_device(self.device_id)
        acl.finalize()

class YOLO11s:
    """YOLO11s模型处理类"""
    def __init__(self, model_path, input_size=640, correct_distortion=True):
        self.model_path = model_path
        self.input_size = input_size
        self.model = None
        self.dvpp = None
        self.correct_distortion = correct_distortion
        self.camera_matrix = CAMERA_MATRIX
        self.dist_coeffs = DISTORTION_COEFFS

    def init(self):
        """初始化模型和图像处理器"""
        self.dvpp = AclLiteImageProc()
        self.model = AclLiteModel(self.model_path)
        return SUCCESS

    def preprocess(self, frame):
        """图像预处理"""
        # 畸变校正
        undistorted_frame = None
        if self.correct_distortion:
            undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            frame = undistorted_frame
        else:
            undistorted_frame = frame
        
        # 保持宽高比缩放
        h, w = frame.shape[:2]
        scale = min(self.input_size/w, self.input_size/h)
        nh, nw = int(h*scale), int(w*scale)
        img = cv2.resize(frame, (nw, nh))
        
        # 填充灰边
        top = (self.input_size - nh) // 2
        bottom = self.input_size - nh - top
        left = (self.input_size - nw) // 2
        right = self.input_size - nw - left
        img = cv2.copyMakeBorder(img, top, bottom, left, right, 
                                cv2.BORDER_CONSTANT, value=(114,114,114))
        
        # 归一化并转换格式
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose(2,0,1)[np.newaxis].astype(np.float32)/255
        return img, (h, w), (top, left, scale), undistorted_frame

    def postprocess(self, pred, orig_shape, pad_info):
        """后处理"""
        CONF_THRESH = 0.1
        IOU_THRESH = 0.9
        # 打印实际 shape
        #print("pred[0] shape:", pred[0].shape)
        arr = pred[0]
        # 兼容 (1, 84, 8400) 输出
        if arr.ndim == 3 and arr.shape[0] == 1:
            arr = arr.squeeze(0)
        if arr.shape[0] == 84 and arr.shape[1] == 8400:
            arr = arr.transpose(1, 0)  # (8400, 84)
        elif arr.shape[0] == 8400 and arr.shape[1] == 84:
            pass  # already correct
        else:
            raise ValueError(f"Unexpected pred shape: {arr.shape}")
        
        conf_mask = arr[:, 4] > CONF_THRESH
        detections = []
        for i in range(arr.shape[0]):
            if not conf_mask[i]:
                continue
            cx, cy, w, h = arr[i, :4]
            conf = arr[i, 4]
            cls_scores = arr[i, 5:]
            # 正确还原到原图坐标
            cx = (cx - pad_info[1]) / pad_info[2]
            cy = (cy - pad_info[0]) / pad_info[2]
            w = w / pad_info[2]
            h = h / pad_info[2]
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)
            class_id = np.argmax(cls_scores)
            detections.append([x1, y1, x2, y2, conf, class_id])
            
        # NMS处理
        if not detections:
            return []
        
        boxes = [d[:4] for d in detections]
        confs = [d[4] for d in detections]
        indices = cv2.dnn.NMSBoxes(boxes, confs, CONF_THRESH, IOU_THRESH)
        
        if len(indices) == 0:
            return []
            
        if isinstance(indices, np.ndarray):
            indices = indices.flatten()
        else:
            indices = [i[0] if isinstance(i, (list, tuple, np.ndarray)) else i for i in indices]
        
        return [detections[i] for i in indices]

class YoloDetectorNode(Node):
    """
    YOLO检测器节点，订阅底部摄像头校正图像，
    执行目标检测并发布检测结果。
    """

    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # 参数设置（硬编码在程序中）
        self.display_image = True  # 控制是否显示检测画面
        
        # 声明参数
        self.declare_parameter('model_path', 'models/yolo11s16.om')
        self.declare_parameter('device_id', 0)
        self.declare_parameter('input_size', 640)
        
        # 获取参数
        model_name = self.get_parameter('model_path').value
        self.device_id = self.get_parameter('device_id').value
        self.input_size = self.get_parameter('input_size').value
        
        # 获取模型的完整路径
        pkg_dir = get_package_share_directory('bottom_camera_yolo_detector')
        self.model_path = os.path.join(pkg_dir, model_name)
        
        self.get_logger().info(f'使用模型: {self.model_path}')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 创建发布器
        self.detection_publisher = self.create_publisher(
            YoloDetection, 'bottom_camera/detections', 10)

        # 初始化资源和模型（在主线程中完成）
        self.acl_resource = None
        self.yolo_model = None
        self.initialized = False
        
        # 在主线程中执行初始化
        self.get_logger().info('开始初始化ACL资源和YOLO模型...')
        self.initialize_model()
        
        # 只有在成功初始化模型后才创建订阅器
        if self.initialized:
            self.get_logger().info('初始化成功，创建图像订阅器')
            self.image_subscription = self.create_subscription(
                Image,
                'bottom_camera/image_rect',  # 订阅已校正的图像
                self.image_callback,
                10)
        else:
            self.get_logger().error('模型初始化失败，未创建图像订阅器')

    def initialize_model(self):
        """初始化ACL资源和YOLO模型"""
        try:
            # 初始化ACL资源
            self.get_logger().info('开始初始化ACL资源...')
            self.acl_resource = AclLiteResource(self.device_id)
            result = self.acl_resource.init()
            if result != SUCCESS:
                self.get_logger().error('初始化ACL资源失败')
                return
            
            self.get_logger().info('ACL资源初始化成功')
            
            # 检查模型文件是否存在
            if not os.path.exists(self.model_path):
                self.get_logger().error(f'模型文件不存在: {self.model_path}')
                return
            
            self.get_logger().info(f'找到模型文件: {self.model_path}')
            
            # 初始化YOLO模型
            self.get_logger().info('开始初始化YOLO模型...')
            self.yolo_model = YOLO11s(self.model_path, self.input_size, correct_distortion=False)
            result = self.yolo_model.init()
            if result != SUCCESS:
                self.get_logger().error('初始化YOLO模型失败')
                return
            
            self.initialized = True
            self.get_logger().info('成功初始化ACL资源和YOLO模型')
        except Exception as e:
            self.get_logger().error(f'初始化过程中出错: {e}')
            import traceback
            self.get_logger().error(f'错误详情: {traceback.format_exc()}')

    def image_callback(self, msg):
        """处理接收到的图像消息"""
        if not self.initialized:
            return
        
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 预处理图像
            img, orig_shape, pad_info, _ = self.yolo_model.preprocess(cv_image)
            
            # 执行推理
            predictions = self.yolo_model.model.execute([img])
            
            # 后处理获取检测结果
            detections = self.yolo_model.postprocess(predictions, orig_shape, pad_info)
            
            # 创建YoloDetection消息
            detection_msg = YoloDetection()
            detection_msg.num_objects = len(detections)
            
            # 如果有检测结果，选取最大的边界框
            if detections:
                # 按面积排序检测结果
                detections.sort(key=lambda x: (x[2] - x[0]) * (x[3] - x[1]), reverse=True)
                largest_det = detections[0]
                x1, y1, x2, y2 = largest_det[:4]
                
                detection_msg.x = x1
                detection_msg.y = y1
                detection_msg.width = x2 - x1
                detection_msg.height = y2 - y1
            
            # 发布检测结果
            self.detection_publisher.publish(detection_msg)
            
            # 显示图像（如果启用）
            if self.display_image:
                display_frame = cv_image.copy()
                
                # 绘制检测结果
                for det in detections:
                    x1, y1, x2, y2, conf, cls_id = det
                    label = f"class{int(cls_id)} {conf:.2f}"
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(display_frame, label, (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                cv2.imshow("YOLO Detection", display_frame)
                cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    yolo_detector_node = YoloDetectorNode()
    
    # 只有当初始化成功时才进行spin
    if yolo_detector_node.initialized:
        try:
            rclpy.spin(yolo_detector_node)
        except KeyboardInterrupt:
            pass
        finally:
            # 关闭所有OpenCV窗口
            cv2.destroyAllWindows()
    else:
        yolo_detector_node.get_logger().error("由于模型初始化失败，节点不会处理任何图像")
    
    # 无论如何都需要清理资源
    yolo_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
