#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class TopCameraPublisher(Node):
    """
    发布上方摄像头图像的ROS 2节点。
    摄像头采集的数据会作为sensor_msgs/Image类型发布。
    """

    def __init__(self):
        super().__init__('top_camera_publisher')
        
        # 声明参数
        self.declare_parameter('camera_device', 0)  # 默认使用摄像头设备0
        self.declare_parameter('frame_width', 320)  # 图像宽度
        self.declare_parameter('frame_height', 240) # 图像高度
        self.declare_parameter('camera_fps', 30.0)  # 摄像头帧率
        self.declare_parameter('camera_format', 'YUYV')  # 摄像头格式
        
        # 获取参数值
        self.camera_device = self.get_parameter('camera_device').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.camera_fps = self.get_parameter('camera_fps').value
        self.camera_format = self.get_parameter('camera_format').value
        
        # 创建发布器
        self.publisher = self.create_publisher(Image, 'top_camera/image_raw', 10)
        
        # 初始化摄像头
        self.get_logger().info(f'正在打开上方摄像头 (设备 {self.camera_device})')
        self.cap = cv2.VideoCapture(self.camera_device)
        
        if not self.cap.isOpened():
            self.get_logger().error('无法打开上方摄像头')
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
        
        self.get_logger().info('上方摄像头发布器已初始化')

    def publish_loop(self):
        """发布循环，读取并发布摄像头图像"""
        while rclpy.ok():
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().warn('未能从摄像头读取帧')
                rclpy.spin_once(self, timeout_sec=0.01)
                continue
            
            # 将OpenCV图像转换为ROS图像消息
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "top_camera_link"
                
                # 发布图像消息
                self.publisher.publish(img_msg)
                
            except Exception as e:
                self.get_logger().error(f'转换图像时出错: {str(e)}')
            
            rclpy.spin_once(self, timeout_sec=0.0)
    
    def __del__(self):
        """销毁对象时释放摄像头资源"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = TopCameraPublisher()
    
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