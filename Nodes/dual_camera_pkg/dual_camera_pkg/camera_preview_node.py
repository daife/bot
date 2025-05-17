#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np


class CameraPreviewNode(Node):
    """
    摄像头预览节点，用于显示上方和下方摄像头的校正后图像。
    """

    def __init__(self):
        super().__init__('camera_preview_node')
        
        # 创建CV桥接器用于转换ROS图像消息和OpenCV图像
        self.bridge = CvBridge()
        
        # 存储最新的图像帧
        self.top_frame = None
        self.bottom_frame = None
        
        # 创建订阅器 - 只订阅校正后的图像话题
        self.top_camera_subscription = self.create_subscription(
            Image,
            'top_camera/image_rect',
            self.top_camera_callback,
            10
        )
        self.bottom_camera_subscription = self.create_subscription(
            Image,
            'bottom_camera/image_rect',
            self.bottom_camera_callback,
            10
        )
        
        self.get_logger().info('摄像头预览节点已初始化，等待摄像头校正图像数据...')
        
        # 创建定时器以固定频率更新显示
        self.timer = self.create_timer(0.033, self.update_display)  # 约30Hz

    def top_camera_callback(self, msg):
        """处理上方摄像头校正图像回调"""
        try:
            self.top_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'处理上方摄像头图像时出错: {str(e)}')

    def bottom_camera_callback(self, msg):
        """处理下方摄像头校正图像回调"""
        try:
            self.bottom_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'处理下方摄像头图像时出错: {str(e)}')

    def update_display(self):
        """更新显示两个摄像头的校正图像"""
        if self.top_frame is not None:
            cv2.imshow('上方摄像头(校正)', self.top_frame)
        
        if self.bottom_frame is not None:
            cv2.imshow('下方摄像头(校正)', self.bottom_frame)
        
        # 短时间等待键盘输入，这样窗口才能更新
        key = cv2.waitKey(1)
        if key == 27:  # ESC键
            self.get_logger().info('用户按下ESC键，关闭预览')
            rclpy.shutdown()

    def __del__(self):
        """清理资源"""
        cv2.destroyAllWindows()
        self.get_logger().info('摄像头预览节点已关闭')


def main(args=None):
    rclpy.init(args=args)
    camera_preview_node = CameraPreviewNode()
    
    try:
        rclpy.spin(camera_preview_node)
    except KeyboardInterrupt:
        camera_preview_node.get_logger().info('用户中断，关闭预览')
    finally:
        # 清理资源
        cv2.destroyAllWindows()
        camera_preview_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()