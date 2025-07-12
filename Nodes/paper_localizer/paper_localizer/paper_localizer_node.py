#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
import acl
import time
import ctypes
import pyudev
from scipy import ndimage
from skimage.morphology import skeletonize

# 导入ACL相关模块
try:
    from acllite_utils import *
    from constants import *
    from acllite_model import AclLiteModel
    from acllite_resource import resource_list
except ImportError as e:
    print(f"Warning: ACL modules not available: {e}")

# DVPP constants
HI_PT_JPEG = 26
HI_VDEC_SEND_MODE_FRAME = 0
HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420 = 1
HI_TRUE = 1
HI_FALSE = 0

# ACL constants
ACL_MEMCPY_HOST_TO_DEVICE = 1
ACL_MEMCPY_DEVICE_TO_HOST = 2

# Camera settings
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 120

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
        return const.SUCCESS

    def __del__(self):
        resource_list.destroy()
        if self.stream:
            acl.rt.destroy_stream(self.stream)
        if self.context:
            acl.rt.destroy_context(self.context)
        acl.rt.reset_device(self.device_id)
        acl.finalize()

class PaperSegmentationDVPP:
    """纸条分割处理类 - 基于DVPP和AIPP"""
    def __init__(self, model_path):
        self.model_path = model_path
        self.model = None
        self.channel_id = 0
        
        # 预分配的内存缓冲区
        self.jpeg_input_buffer = None
        self.jpeg_input_size = 0
        self.yuv_output_buffer = None
        self.yuv_output_size = 0
        self.padded_yuv_buffer = None
        self.padded_yuv_size = 640 * 640 * 3 // 2
        self.host_buffer = None
        self.host_buffer_size = 0
        self.padded_host_buffer = None
        
        # 预分配的numpy数组
        self.yuv_numpy_array = None
        self.yuv_input_view = None

    def init(self):
        """初始化模型和DVPP，预分配内存缓冲区"""
        # Initialize model
        self.model = AclLiteModel(self.model_path)
        
        # Initialize DVPP
        if not self.init_dvpp():
            return const.FAILED
        
        # 预分配内存缓冲区
        if not self.init_memory_buffers():
            return const.FAILED
            
        return const.SUCCESS

    def init_memory_buffers(self):
        """初始化预分配的内存缓冲区"""
        try:
            # 1. 预分配JPEG输入缓冲区
            self.jpeg_input_size = 640 * 480 * 2
            self.jpeg_input_buffer, ret = acl.himpi.dvpp_malloc(0, self.jpeg_input_size)
            if ret != 0:
                print(f"Failed to allocate JPEG input buffer: {ret}")
                return False
            
            # 2. 预分配YUV输出缓冲区
            self.yuv_output_size = 640 * 480 * 3 // 2
            self.yuv_output_buffer, ret = acl.himpi.dvpp_malloc(0, self.yuv_output_size)
            if ret != 0:
                print(f"Failed to allocate YUV output buffer: {ret}")
                return False
            
            # 3. 预分配填充后的YUV缓冲区并初始化填充区域
            self.padded_yuv_buffer, ret = acl.himpi.dvpp_malloc(0, self.padded_yuv_size)
            if ret != 0:
                print(f"Failed to allocate padded YUV buffer: {ret}")
                return False
            
            # 初始化填充区域为黑色
            if not self.init_padding_area():
                return False
            
            # 4. 预分配主机内存缓冲区
            self.host_buffer_size = max(self.yuv_output_size, self.padded_yuv_size)
            self.host_buffer, ret = acl.rt.malloc_host(self.host_buffer_size)
            if ret != 0:
                print(f"Failed to allocate host buffer: {ret}")
                return False
            
            self.padded_host_buffer, ret = acl.rt.malloc_host(self.padded_yuv_size)
            if ret != 0:
                print(f"Failed to allocate padded host buffer: {ret}")
                return False
            
            # 5. 预分配numpy数组
            self.yuv_numpy_array = np.zeros((self.padded_yuv_size,), dtype=np.uint8)
            self.yuv_input_view = self.yuv_numpy_array.reshape((640 * 3 // 2, 640))
            
            return True
            
        except Exception as e:
            print(f"Failed to initialize memory buffers: {e}")
            return False

    def init_padding_area(self):
        """初始化640x640填充区域为黑色背景"""
        try:
            temp_host_buffer, ret = acl.rt.malloc_host(self.padded_yuv_size)
            if ret != 0:
                return False
            
            # 设置Y平面为0 (黑色的亮度值)
            y_size = 640 * 640
            ctypes.memset(temp_host_buffer, 0, y_size)
            
            # 设置UV平面为128 (黑色的色度值)
            uv_size = 640 * 640 // 2
            ctypes.memset(temp_host_buffer + y_size, 128, uv_size)
            
            # 将初始化的数据复制到设备内存
            ret = acl.rt.memcpy(self.padded_yuv_buffer, self.padded_yuv_size,
                              temp_host_buffer, self.padded_yuv_size, ACL_MEMCPY_HOST_TO_DEVICE)
            
            acl.rt.free_host(temp_host_buffer)
            
            if ret != 0:
                print(f"Failed to initialize padding area: {ret}")
                return False
            
            return True
            
        except Exception as e:
            print(f"Exception initializing padding area: {e}")
            return False

    def init_dvpp(self):
        """初始化DVPP系统"""
        ret = acl.himpi.sys_init()
        if ret != 0:
            print(f"Failed to init media system: {ret}")
            return False
        
        return self.create_vdec_channel()

    def create_vdec_channel(self):
        """创建VDEC通道用于JPEG解码"""
        attr = {'type': HI_PT_JPEG, 'mode': HI_VDEC_SEND_MODE_FRAME,
                'pic_width': 640, 'pic_height': 480,
                'stream_buf_size': 640 * 480, 'frame_buf_size': 0,
                'frame_buf_cnt': 9}

        ret = acl.himpi.vdec_create_chn(self.channel_id, attr)
        if ret != 0:
            print(f"Failed to create VDEC channel: {ret}")
            return False
            
        # Set channel parameters
        jpegd_param_dict, ret = acl.himpi.vdec_get_chn_param(self.channel_id)
        if ret != 0:
            print(f"Failed to get channel param: {ret}")
            return False
        
        try:
            if 'pic_param' in jpegd_param_dict:
                jpegd_param_dict["pic_param"]["pixel_format"] = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420
                jpegd_param_dict["pic_param"]["alpha"] = 255
            
            ret = acl.himpi.vdec_set_chn_param(self.channel_id, jpegd_param_dict)
            if ret != 0:
                print(f"Warning: Failed to set channel param: {ret}")
                
        except Exception as e:
            print(f"Exception setting channel params: {e}")

        ret = acl.himpi.vdec_start_recv_stream(self.channel_id)
        if ret != 0:
            print(f"Failed to start recv stream: {ret}")
            return False
            
        return True

    def trim_jpeg_data(self, jpeg_data):
        """修剪JPEG数据，移除填充"""
        jpeg_end_marker = b'\xff\xd9'
        
        try:
            end_pos = jpeg_data.find(jpeg_end_marker)
            if end_pos != -1:
                trimmed_data = jpeg_data[:end_pos + 2]
                return trimmed_data
            else:
                return jpeg_data
        except Exception as e:
            print(f"Error trimming JPEG data: {e}")
            return jpeg_data

    def validate_jpeg_format(self, jpeg_data):
        """验证JPEG格式"""
        if len(jpeg_data) < 10:
            return False
            
        if jpeg_data[:2] != b'\xff\xd8' or jpeg_data[-2:] != b'\xff\xd9':
            return False
            
        return True

    def process_jpeg_to_yuv_optimized(self, jpeg_data):
        """优化的JPEG到YUV处理"""
        trimmed_jpeg_data = self.trim_jpeg_data(jpeg_data)
        
        if not self.validate_jpeg_format(trimmed_jpeg_data):
            print("JPEG format validation failed")
            return None
        
        jpeg_data = trimmed_jpeg_data
        input_size = len(jpeg_data)
        
        if input_size > self.jpeg_input_size:
            print(f"JPEG size {input_size} exceeds buffer size {self.jpeg_input_size}")
            return None
            
        try:
            vdec_file = np.frombuffer(jpeg_data, dtype=np.uint8)
            bytes_data = vdec_file.tobytes()
            vdec_file_ptr = acl.util.bytes_to_ptr(bytes_data)
            
            ret = acl.rt.memcpy(self.jpeg_input_buffer, input_size, vdec_file_ptr, 
                              vdec_file.itemsize * vdec_file.size, ACL_MEMCPY_HOST_TO_DEVICE)
            if ret != 0:
                print(f"Failed to copy data to device: {ret}")
                return None
            
            # ...existing code for stream creation and processing...
            stream = {'end_of_frame': HI_TRUE, 'end_of_stream': HI_FALSE,
                      'need_display': HI_TRUE, 'pts': 0,
                      'len': input_size, 'addr': self.jpeg_input_buffer}
            
            temp_stream = {'end_of_frame': HI_TRUE, 'end_of_stream': HI_FALSE,
                          'need_display': HI_TRUE, 'pts': 0,
                          'len': input_size, 'addr': vdec_file_ptr}
            
            img_info, ret = acl.himpi.dvpp_get_image_info(HI_PT_JPEG, temp_stream)
            if ret != 0:
                print(f"Failed to get image info: {ret}")
                return None
            
            out_pic_info = {"width": img_info['width'],
                            "height": img_info['height'],
                            "width_stride": img_info['width_stride'],
                            "height_stride": img_info['height_stride'],
                            "pixel_format": HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420,
                            "buffer_size": self.yuv_output_size,
                            "vir_addr": self.yuv_output_buffer}

            ret = acl.himpi.vdec_send_stream(self.channel_id, stream, out_pic_info, -1)
            if ret != 0:
                ret = acl.himpi.vdec_send_stream(self.channel_id, stream, out_pic_info, 0)
                if ret != 0:
                    print(f"Failed to send stream: {ret}")
                    return None

            frame_info, supplement, ret_stream, ret = acl.himpi.vdec_get_frame(self.channel_id, 1000)
            if ret != 0:
                print(f"Failed to get frame: {ret}")
                return None
                
            dec_result = frame_info['v_frame']['frame_flag']
            if dec_result != 0:
                print(f"Decode failed: {dec_result}")
                acl.himpi.vdec_release_frame(self.channel_id, frame_info)
                return None

            success = self.pad_yuv420sp_to_square_optimized(frame_info['v_frame']['virt_addr'][0], 640, 480)
            
            acl.himpi.vdec_release_frame(self.channel_id, frame_info)
            
            return self.padded_yuv_buffer if success else None
            
        except Exception as e:
            print(f"Exception in optimized DVPP processing: {e}")
            return None

    def pad_yuv420sp_to_square_optimized(self, yuv_addr, width, height):
        """优化的YUV填充"""
        try:
            ret = acl.rt.memcpy(self.host_buffer, self.yuv_output_size, 
                              yuv_addr, self.yuv_output_size, ACL_MEMCPY_DEVICE_TO_HOST)
            if ret != 0:
                print(f"Failed to copy YUV to host: {ret}")
                return False
            
            ret = acl.rt.memcpy(self.padded_host_buffer, self.padded_yuv_size,
                              self.padded_yuv_buffer, self.padded_yuv_size, ACL_MEMCPY_DEVICE_TO_HOST)
            if ret != 0:
                print(f"Failed to copy padded template to host: {ret}")
                return False
            
            # 复制Y平面 (居中放置)
            y_offset = (640 - 480) // 2
            for row in range(480):
                src_offset = row * 640
                dst_offset = (row + y_offset) * 640
                ctypes.memmove(self.padded_host_buffer + dst_offset, 
                             self.host_buffer + src_offset, 640)
            
            # 复制UV平面 (居中放置)
            y_size_orig = 640 * 480
            y_size_new = 640 * 640
            uv_offset_rows = (640 - 480) // 4
            for row in range(240):
                src_offset = y_size_orig + row * 640
                dst_offset = y_size_new + (row + uv_offset_rows) * 640
                ctypes.memmove(self.padded_host_buffer + dst_offset,
                             self.host_buffer + src_offset, 640)
                             
            ret = acl.rt.memcpy(self.padded_yuv_buffer, self.padded_yuv_size,
                              self.padded_host_buffer, self.padded_yuv_size, ACL_MEMCPY_HOST_TO_DEVICE)
            if ret != 0:
                print(f"Failed to copy padded YUV back to device: {ret}")
                return False
                
            return True
            
        except Exception as e:
            print(f"Exception in optimized YUV padding: {e}")
            return False

    def create_yuv_input_buffer_optimized(self, padded_yuv_addr):
        """优化的YUV输入缓冲区创建"""
        try:
            ret = acl.rt.memcpy(self.host_buffer, self.padded_yuv_size, 
                              padded_yuv_addr, self.padded_yuv_size, ACL_MEMCPY_DEVICE_TO_HOST)
            if ret != 0:
                print(f"Failed to copy YUV from device to host: {ret}")
                return None
            
            ctypes.memmove(self.yuv_numpy_array.ctypes.data, self.host_buffer, self.padded_yuv_size)
            
            return self.yuv_input_view
            
        except Exception as e:
            print(f"Exception creating optimized YUV input buffer: {e}")
            return None

    def postprocess_paper_center(self, pred, orig_shape=(480, 640)):
        """简化的后处理 - 计算纸条的真正中心点（中心骨架点）"""
        CONF_THRESH = 0.3
        
        if pred is None:
            return None
        
        output = pred[0]
        
        if len(pred) > 1:
            proto = pred[1]
        else:
            proto = None
        
        if output.ndim == 3 and output.shape[0] == 1:
            output = output.squeeze(0)
        
        if output.shape[0] < output.shape[1]:
            output = output.T
        
        # 解析输出
        boxes = output[:, :4]
        scores = output[:, 4]
        
        if output.shape[1] > 5:
            mask_coeffs = output[:, 5:]
        else:
            mask_coeffs = None
        
        # 过滤低置信度的检测
        valid_mask = scores > CONF_THRESH
        if not np.any(valid_mask):
            return None
        
        boxes = boxes[valid_mask]
        scores = scores[valid_mask]
        if mask_coeffs is not None:
            mask_coeffs = mask_coeffs[valid_mask]
        
        # 选择最高置信度的检测
        best_idx = np.argmax(scores)
        best_score = scores[best_idx]
        
        # 生成掩膜
        if mask_coeffs is not None and proto is not None:
            best_mask_coeffs = mask_coeffs[best_idx]
            mask = self.generate_mask(best_mask_coeffs, proto, boxes[best_idx], orig_shape)
        else:
            # 使用边界框创建简单掩膜
            mask = self.create_bbox_mask(boxes[best_idx], orig_shape)
        
        if mask is None:
            return None
        
        # 计算纸条的真正中心点（中心骨架点）
        center_point = self.calculate_paper_center(mask)
        
        if center_point is not None:
            return {
                'center_x': center_point[0],
                'center_y': center_point[1],
                'confidence': best_score,
                'mask': mask
            }
        
        return None
    
    def calculate_paper_center(self, mask):
        """计算纸条的真正中心点 - 使用骨架化找到中心线上的点"""
        try:
            # 确保掩膜是二值的
            binary_mask = (mask > 0).astype(np.uint8)
            
            if np.sum(binary_mask) < 10:  # 如果掩膜太小，直接返回质心
                return self.calculate_centroid(binary_mask)
            
            # 方法1：使用形态学骨架化找到中心线
            skeleton = skeletonize(binary_mask)
            skeleton_points = np.where(skeleton > 0)
            
            if len(skeleton_points[0]) > 0:
                # 在骨架点中找到最接近质心的点
                centroid = self.calculate_centroid(binary_mask)
                if centroid is None:
                    return None
                
                # 计算每个骨架点到质心的距离
                distances = ((skeleton_points[1] - centroid[0]) ** 2 + 
                           (skeleton_points[0] - centroid[1]) ** 2)
                closest_idx = np.argmin(distances)
                
                center_x = skeleton_points[1][closest_idx]
                center_y = skeleton_points[0][closest_idx]
                
                return (int(center_x), int(center_y))
            
            # 方法2：如果骨架化失败，使用距离变换找到中心
            dist_transform = cv2.distanceTransform(binary_mask, cv2.DIST_L2, 5)
            
            # 找到距离变换的最大值点（最远离边缘的点）
            max_val = np.max(dist_transform)
            if max_val > 0:
                max_points = np.where(dist_transform == max_val)
                if len(max_points[0]) > 0:
                    # 如果有多个最大值点，取中间的那个
                    center_idx = len(max_points[0]) // 2
                    center_x = max_points[1][center_idx]
                    center_y = max_points[0][center_idx]
                    return (int(center_x), int(center_y))
            
            # 方法3：如果以上都失败，返回质心
            return self.calculate_centroid(binary_mask)
            
        except Exception as e:
            print(f"Error calculating paper center: {e}")
            # 出错时返回质心
            return self.calculate_centroid(mask)
    
    def calculate_centroid(self, mask):
        """计算掩膜的质心"""
        try:
            y_coords, x_coords = np.where(mask > 0)
            
            if len(x_coords) == 0 or len(y_coords) == 0:
                return None
            
            center_x = int(np.mean(x_coords))
            center_y = int(np.mean(y_coords))
            
            return (center_x, center_y)
        except:
            return None
    
    def generate_mask(self, mask_coeffs, proto, box, orig_shape):
        """从掩膜系数和原型生成最终掩膜"""
        if proto.ndim == 4 and proto.shape[0] == 1:
            proto = proto.squeeze(0)
        
        mask = np.sum(mask_coeffs[:, None, None] * proto, axis=0)
        mask = 1 / (1 + np.exp(-mask))
        
        mask = cv2.resize(mask, (640, 640))
        
        # 应用边界框裁剪
        x1, y1, x2, y2 = box.astype(int)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(640, x2), min(640, y2)
        
        bbox_mask = np.zeros((640, 640), dtype=np.float32)
        bbox_mask[y1:y2, x1:x2] = 1.0
        mask = mask * bbox_mask
        
        # 还原到原图尺寸
        mask = self.remove_padding_and_scale_mask(mask, orig_shape)
        
        # 二值化
        mask = (mask > 0.5).astype(np.uint8)
        
        return mask
    
    def remove_padding_and_scale_mask(self, mask, orig_shape):
        """移除填充并缩放掩膜到原图尺寸"""
        orig_h, orig_w = orig_shape
        
        y_offset = (640 - orig_h) // 2
        x_offset = (640 - orig_w) // 2
        
        mask_cropped = mask[y_offset:y_offset+orig_h, x_offset:x_offset+orig_w]
        
        return mask_cropped
    
    def create_bbox_mask(self, box, orig_shape):
        """基于边界框创建简单掩膜"""
        orig_h, orig_w = orig_shape
        
        y_offset = (640 - orig_h) // 2
        x_offset = (640 - orig_w) // 2
        
        x1, y1, x2, y2 = box.astype(int)
        x1 = max(0, min(orig_w, x1 - x_offset))
        y1 = max(0, min(orig_h, y1 - y_offset))
        x2 = max(0, min(orig_w, x2 - x_offset))
        y2 = max(0, min(orig_h, y2 - y_offset))
        
        mask = np.zeros((orig_h, orig_w), dtype=np.uint8)
        mask[y1:y2, x1:x2] = 1
        
        return mask

    def cleanup(self):
        """清理资源"""
        try:
            if self.jpeg_input_buffer:
                acl.himpi.dvpp_free(self.jpeg_input_buffer)
                self.jpeg_input_buffer = None
            
            if self.yuv_output_buffer:
                acl.himpi.dvpp_free(self.yuv_output_buffer)
                self.yuv_output_buffer = None
            
            if self.padded_yuv_buffer:
                acl.himpi.dvpp_free(self.padded_yuv_buffer)
                self.padded_yuv_buffer = None
            
            if self.host_buffer:
                acl.rt.free_host(self.host_buffer)
                self.host_buffer = None
            
            if self.padded_host_buffer:
                acl.rt.free_host(self.padded_host_buffer)
                self.padded_host_buffer = None
            
            if hasattr(self, 'channel_id'):
                acl.himpi.vdec_stop_recv_stream(self.channel_id)
                acl.himpi.vdec_destroy_chn(self.channel_id)
            
            acl.himpi.sys_exit()
        except Exception as e:
            print(f"Exception during cleanup: {e}")

class PaperLocalizerNode(Node):
    """纸条定位节点"""
    
    def __init__(self):
        super().__init__('paper_localizer_node')
        
        # 声明参数
        self.declare_parameter('model_path', 'yolo11s-seg-self-aipp.om')
        self.declare_parameter('publish_rate', 10.0)
        
        # 获取参数
        self.model_path = self.get_parameter('model_path').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 初始化ACL资源
        self.acl_resource = AclLiteResource()
        self.acl_resource.init()
        
        # 初始化分割模型
        self.paper_seg = PaperSegmentationDVPP(self.model_path)
        if self.paper_seg.init() != const.SUCCESS:
            self.get_logger().error("模型或DVPP初始化失败")
            return
        
        # 创建发布器
        self.pose_publisher = self.create_publisher(Pose, 'paper_center_pose', 10)
        
        # 创建定时器控制发布频率
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # 存储最新的检测结果
        self.latest_center = None
        self.latest_confidence = 0.0
        self.latest_mask = None
        
        # 初始化摄像头
        self.cap = None
        self.init_camera()
        
        self.get_logger().info(f'纸条定位节点已启动')

    def find_target_camera(self):
        """使用pyudev查找目标摄像头设备"""
        context = pyudev.Context()
        target_serial = "HD_Camera_Manufacturer_USB_2.0_Camera"
        
        video_devices = []
        for device in context.list_devices(subsystem='video4linux'):
            if device.get('ID_SERIAL') == target_serial:
                device_node = device.device_node
                if device_node and '/dev/video' in device_node:
                    video_devices.append(device_node)
        
        if video_devices:
            video_devices.sort()
            self.get_logger().info(f"找到目标摄像头设备: {video_devices}")
            return video_devices[0]
        else:
            self.get_logger().warn(f"未找到ID_SERIAL为 {target_serial} 的摄像头设备")
            return None

    def init_camera(self):
        """初始化摄像头"""
        # 查找目标摄像头
        camera_device = self.find_target_camera()
        if camera_device is None:
            self.get_logger().warn("未找到目标摄像头，使用默认摄像头 /dev/video0")
            camera_device = "/dev/video0"
        
        device_index = int(camera_device.replace('/dev/video', ''))
        self.get_logger().info(f"使用摄像头设备: {camera_device} (索引: {device_index})")

        # 打开摄像头并设置参数（使用v4l2获取原始MJPG数据）
        self.cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            return
            
        # 设置摄像头参数 - 与main-seg-dvaipp.py完全相同
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # 获取原始MJPG数据
        
        # 验证摄像头设置
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        fourcc = self.cap.get(cv2.CAP_PROP_FOURCC)
        convert_rgb = self.cap.get(cv2.CAP_PROP_CONVERT_RGB)
        
        self.get_logger().info(f"摄像头设置: {actual_width}x{actual_height} @ {actual_fps}fps")
        self.get_logger().info(f"FOURCC: {int(fourcc)}, Convert RGB: {int(convert_rgb)}")
        
        # 预热摄像头
        for i in range(5):
            ret, _ = self.cap.read()
            if ret:
                self.get_logger().info(f"摄像头预热 {i+1}/5")
            time.sleep(0.1)

    def process_camera_frame(self):
        """处理摄像头帧"""
        if self.cap is None or not self.cap.isOpened():
            return
            
        ret, frame_data = self.cap.read()
        if not ret or frame_data is None:
            return
        
        try:
            # 检查是否为原始MJPG数据
            if len(frame_data.shape) == 2 and frame_data.shape[0] == 1:
                # 转换为字节数据
                jpeg_data = frame_data.flatten().tobytes()
                
                # 使用优化的DVPP处理
                yuv_addr = self.paper_seg.process_jpeg_to_yuv_optimized(jpeg_data)
                if yuv_addr is not None:
                    yuv_input = self.paper_seg.create_yuv_input_buffer_optimized(yuv_addr)
                    
                    if yuv_input is not None:
                        # 模型推理 - 使用与main-seg-dvaipp相同的方式
                        try:
                            # 确保使用AclLiteModel.execute方法，传入列表格式的输入
                            pred = self.paper_seg.model.execute([yuv_input])
                            
                            # 后处理获取中心点
                            result = self.paper_seg.postprocess_paper_center(pred, orig_shape=(480, 640))
                            
                            if result is not None:
                                self.latest_center = (result['center_x'], result['center_y'])
                                self.latest_confidence = result['confidence']
                                self.latest_mask = result['mask']
                                self.get_logger().debug(f"检测到纸条: 中心({result['center_x']}, {result['center_y']}), 置信度{result['confidence']:.3f}")
                            else:
                                self.latest_center = None
                                self.latest_confidence = 0.0
                                self.latest_mask = None
                                
                        except Exception as e:
                            self.get_logger().error(f"模型推理失败: {e}")
                            # 打印更详细的错误信息
                            import traceback
                            self.get_logger().error(f"详细错误: {traceback.format_exc()}")
            else:
                # 如果不是原始MJPG数据，尝试重新编码
                if len(frame_data.shape) == 3:
                    # BGR图像，重新编码为JPEG
                    _, jpeg_encoded = cv2.imencode('.jpg', frame_data)
                    jpeg_data = jpeg_encoded.tobytes()
                    
                    # 使用优化的DVPP处理
                    yuv_addr = self.paper_seg.process_jpeg_to_yuv_optimized(jpeg_data)
                    if yuv_addr is not None:
                        yuv_input = self.paper_seg.create_yuv_input_buffer_optimized(yuv_addr)
                        
                        if yuv_input is not None:
                            try:
                                pred = self.paper_seg.model.execute([yuv_input])
                                result = self.paper_seg.postprocess_paper_center(pred, orig_shape=(480, 640))
                                
                                if result is not None:
                                    self.latest_center = (result['center_x'], result['center_y'])
                                    self.latest_confidence = result['confidence']
                                    self.latest_mask = result['mask']
                                else:
                                    self.latest_center = None
                                    self.latest_confidence = 0.0
                                    self.latest_mask = None
                                    
                            except Exception as e:
                                self.get_logger().error(f"BGR->JPEG模型推理失败: {e}")
                                
        except Exception as e:
            self.get_logger().error(f"图像处理出错: {e}")

    def timer_callback(self):
        """定时器回调，处理摄像头数据并发布纸条中心点位置"""
        # 处理摄像头帧
        self.process_camera_frame()
        
        pose_msg = Pose()
        
        # 图像中心点
        image_center_x = 320
        image_center_y = 240
        
        if self.latest_center is not None:
            # 计算相对于图像中心的坐标（有符号）
            center_x_pixel, center_y_pixel = self.latest_center
            
            # 相对于图像中心的坐标
            relative_x = float(center_x_pixel - image_center_x)
            relative_y = float(center_y_pixel - image_center_y)
            
            pose_msg.position.x = relative_x
            pose_msg.position.y = relative_y
            
            # 检查图像中心点是否在掩膜范围内
            if (self.latest_mask is not None and 
                0 <= image_center_y < self.latest_mask.shape[0] and 
                0 <= image_center_x < self.latest_mask.shape[1]):
                
                if self.latest_mask[image_center_y, image_center_x] > 0:
                    # 图像中心在掩膜内
                    pose_msg.position.z = 1.0
                else:
                    # 图像中心不在掩膜内
                    pose_msg.position.z = 0.0
            else:
                # 掩膜无效或坐标越界
                pose_msg.position.z = 0.0
            
            self.get_logger().debug(
                f"发布纸条中心位置: x={relative_x:.1f}, y={relative_y:.1f}, z={pose_msg.position.z}, "
                f"置信度={self.latest_confidence:.3f}, "
                f"像素坐标=({center_x_pixel}, {center_y_pixel})"
            )
        else:
            # 没有检测到目标
            pose_msg.position.x = 0.0
            pose_msg.position.y = 0.0
            pose_msg.position.z = -1.0
        
        # 设置方向（无旋转）
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0
        
        # 发布
        self.pose_publisher.publish(pose_msg)

    def __del__(self):
        """析构函数"""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        if hasattr(self, 'paper_seg'):
            self.paper_seg.cleanup()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PaperLocalizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("用户中断")
    except Exception as e:
        print(f"节点运行出错: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
