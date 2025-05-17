#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import os
import yaml

class BottomCameraPublisher(Node):
    """
    发布下方摄像头图像的ROS 2节点。
    摄像头采集的数据经过畸变校正后作为sensor_msgs/Image类型发布。
    """

    def __init__(self):
        super().__init__('bottom_camera_publisher')
        
        # 声明参数
        self.declare_parameter('camera_device', 2)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('camera_fps', 60.0)  # 摄像头帧率
        self.declare_parameter('camera_format', 'YUYV')  # 摄像头格式
        self.declare_parameter('calibration_file', 'config/bottom_camera.yaml')  # 相机标定文件路径
        
        # 获取参数
        self.camera_device = self.get_parameter('camera_device').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.camera_fps = self.get_parameter('camera_fps').value
        self.camera_format = self.get_parameter('camera_format').value
        self.calibration_file = self.get_parameter('calibration_file').value
        
        # 加载相机标定参数
        self.load_calibration_params()
        
        # 创建发布器 - 只发布校正后的图像
        self.publisher = self.create_publisher(Image, 'bottom_camera/image_rect', 10)
        
        # 初始化摄像头
        self.get_logger().info(f'正在打开下方摄像头 (设备 {self.camera_device})')
        self.cap = cv2.VideoCapture(self.camera_device)
        
        if not self.cap.isOpened():
            self.get_logger().error('无法打开下方摄像头')
            return
        
        # 设置摄像头属性
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
        
        # 设置摄像头格式
        if self.camera_format.upper() == 'YUYV':
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        self.get_logger().info('下方摄像头发布器已初始化')

    def load_calibration_params(self):
        """加载相机标定参数"""
        package_path = os.path.join(os.path.dirname(os.path.dirname(__file__)))
        cal_file_path = os.path.join(package_path, self.calibration_file)
        
        try:
            with open(cal_file_path, 'r') as file:
                self.get_logger().info(f'加载相机标定文件: {cal_file_path}')
                calib_data = yaml.safe_load(file)
                
                # 提取相机矩阵和畸变系数
                cam_matrix = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
                dist_coeffs = np.array(calib_data['distortion_coefficients']['data'])
                
                self.camera_matrix = cam_matrix
                self.dist_coeffs = dist_coeffs
                self.get_logger().info('相机标定参数加载成功')
                
                # 计算畸变校正映射
                self.mapx, self.mapy = cv2.initUndistortRectifyMap(
                    self.camera_matrix, self.dist_coeffs, None, 
                    self.camera_matrix, (self.frame_width, self.frame_height), 
                    cv2.CV_32FC1)
                
        except Exception as e:
            self.get_logger().error(f'加载相机标定参数失败: {str(e)}')
            # 使用默认值
            self.camera_matrix = np.array([
                [465.13093, 0.0, 324.81802],
                [0.0, 466.33628, 242.54136],
                [0.0, 0.0, 1.0]
            ])
            self.dist_coeffs = np.array([-0.374992, 0.133505, 0.002906, -0.002975, 0.000000])
            
            # 计算畸变校正映射
            self.mapx, self.mapy = cv2.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs, None, 
                self.camera_matrix, (self.frame_width, self.frame_height), 
                cv2.CV_32FC1)

    def publish_loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('未能从摄像头读取帧')
                rclpy.spin_once(self, timeout_sec=0.01)
                continue
            try:
                # 校正图像畸变
                undistorted_frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
                
                # 发布校正后的图像
                rect_img_msg = self.bridge.cv2_to_imgmsg(undistorted_frame, encoding="bgr8")
                rect_img_msg.header.stamp = self.get_clock().now().to_msg()
                rect_img_msg.header.frame_id = "bottom_camera_link_rect"
                self.publisher.publish(rect_img_msg)
                
            except Exception as e:
                self.get_logger().error(f'处理图像时出错: {str(e)}')
            rclpy.spin_once(self, timeout_sec=0.0)
    
    def __del__(self):
        """销毁对象时释放摄像头资源"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = BottomCameraPublisher()
    
    try:
        node.publish_loop()
    except KeyboardInterrupt:
        node.get_logger().info('通过键盘中断停止节点')
    except Exception as e:
        node.get_logger().error(f'发生异常: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()