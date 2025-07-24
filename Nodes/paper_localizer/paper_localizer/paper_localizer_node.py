#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import cv2
import acl
import numpy as np
import ctypes
import time
from skimage.morphology import skeletonize
import queue
import pyudev

MODEL_INPUT_SIZE = 1056  # 与测试脚本一致
WIDTH, HEIGHT = 1920, 1080
CHANNEL_ID = 0
HI_PT_JPEG = 26
HI_VDEC_SEND_MODE_FRAME = 0
HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420 = 1

# 参考点(proto空间264*264)参数定义
REFERENCE_X = 139
REFERENCE_Y = 141

CONF_THRESH = 0.3

def yuv420sp_device_to_numpy(yuv_addr, img_info):
    width = img_info['width']
    height = img_info['height']
    yuv_size = width * height * 3 // 2
    host_buf, ret = acl.rt.malloc_host(yuv_size)
    if ret != 0:
        return None
    ret = acl.rt.memcpy(host_buf, yuv_size, yuv_addr, yuv_size, 2)
    if ret != 0:
        acl.rt.free_host(host_buf)
        return None
    buf_ptr = ctypes.cast(host_buf, ctypes.POINTER(ctypes.c_ubyte * yuv_size))
    yuv_np = np.frombuffer(buf_ptr.contents, dtype=np.uint8).copy()
    acl.rt.free_host(host_buf)
    yuv_np = yuv_np.reshape((height * 3 // 2, width))
    return yuv_np

def postprocess_mask_center(pred, conf_thresh=CONF_THRESH):
    """
    只取置信度最大且大于阈值的掩膜区域，骨架中心（掩膜空间bbox裁剪，不还原到原图）
    返回: (center_x, center_y), mask_shape, confidence, mask_bbox
    """
    if pred is None or len(pred) == 0:
        return None, None, None, None

    output = pred[0]
    if output.ndim == 3 and output.shape[0] == 1:
        output = output.squeeze(0)
    if output.shape[0] < output.shape[1]:
        output = output.T

    boxes = output[:, :4]
    scores = output[:, 4]
    mask_coeffs = output[:, 5:] if output.shape[1] > 5 else None

    valid_mask = scores > conf_thresh
    if not np.any(valid_mask):
        return None, None, None, None

    idx = np.argmax(scores * valid_mask)
    best_score = scores[idx]

    if mask_coeffs is not None and len(pred) > 1:
        proto = pred[1]
        if proto.ndim == 4 and proto.shape[0] == 1:
            proto = proto.squeeze(0)
        best_mask_coeffs = mask_coeffs[idx]
        mask = np.sum(best_mask_coeffs[:, None, None] * proto, axis=0)
        mask = 1 / (1 + np.exp(-mask))
        mask = (mask > 0.5).astype(np.uint8)
        h_mask, w_mask = mask.shape
        box = boxes[idx]
        x_c, y_c, w, h = box
        scale_x = w_mask / MODEL_INPUT_SIZE
        scale_y = h_mask / MODEL_INPUT_SIZE
        x_c_p = x_c * scale_x
        y_c_p = y_c * scale_y
        w_p = w * scale_x
        h_p = h * scale_y
        x1 = int(max(0, x_c_p - w_p / 2))
        y1 = int(max(0, y_c_p - h_p / 2))
        x2 = int(min(w_mask, x_c_p + w_p / 2))
        y2 = int(min(h_mask, y_c_p + h_p / 2))
        mask_bbox = np.zeros_like(mask)
        mask_bbox[y1:y2, x1:x2] = mask[y1:y2, x1:x2]
    else:
        return None, None, None, None

    skeleton = skeletonize(mask_bbox)
    skeleton_points = np.where(skeleton > 0)
    if len(skeleton_points[0]) == 0:
        return None, mask_bbox.shape, best_score, mask_bbox

    center_x = int(np.mean(skeleton_points[1]))
    center_y = int(np.mean(skeleton_points[0]))
    return (center_x, center_y), mask_bbox.shape, best_score, mask_bbox

def find_target_camera():
    """使用pyudev查找目标摄像头设备"""
    context = pyudev.Context()
    target_vendor_id = "05a3"
    target_model_id = "9230"
    video_devices = []
    for device in context.list_devices(subsystem='video4linux'):
        vendor_id = device.get('ID_VENDOR_ID')
        model_id = device.get('ID_MODEL_ID')
        if vendor_id == target_vendor_id and model_id == target_model_id:
            device_node = device.device_node
            if device_node and '/dev/video' in device_node:
                video_devices.append(device_node)
    if video_devices:
        video_devices.sort()
        return video_devices[0]
    else:
        return None

class PaperLocalizerNode(Node):
    def __init__(self):
        super().__init__('paper_localizer_node')
        self.declare_parameter('model_path', '/home/HwHiAiUser/yolo_test/yolo11n-seg-self-12-nodrop-125epo-sourceinput.om')
        self.model_path = self.get_parameter('model_path').value

        # 初始化ACL
        acl.init()
        acl.rt.set_device(0)
        self.ctx, ret = acl.rt.create_context(0)
        acl.himpi.sys_init()
        self.stream, ret = acl.rt.create_stream()

        # 初始化模型
        from acllite_model import AclLiteModel
        self.model = AclLiteModel(self.model_path)

        # 创建VDEC通道
        attr = {
            'type': HI_PT_JPEG,
            'mode': HI_VDEC_SEND_MODE_FRAME,
            'pic_width': WIDTH,
            'pic_height': HEIGHT,
            'stream_buf_size': WIDTH * HEIGHT,
            'frame_buf_size': 0,
            'frame_buf_cnt': 9
        }
        ret = acl.himpi.vdec_create_chn(CHANNEL_ID, attr)
        jpegd_param_dict, ret = acl.himpi.vdec_get_chn_param(CHANNEL_ID)
        if 'pic_param' in jpegd_param_dict:
            jpegd_param_dict["pic_param"]["pixel_format"] = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420
            jpegd_param_dict["pic_param"]["alpha"] = 255
        elif 'pixel_format' in jpegd_param_dict:
            jpegd_param_dict["pixel_format"] = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420
            if 'alpha' in jpegd_param_dict:
                jpegd_param_dict["alpha"] = 255
        acl.himpi.vdec_set_chn_param(CHANNEL_ID, jpegd_param_dict)
        acl.himpi.vdec_start_recv_stream(CHANNEL_ID)

        # 摄像头初始化（通过pyudev查找）
        camera_device = find_target_camera()
        if camera_device is None:
            self.get_logger().warn("未找到目标摄像头，使用默认摄像头 /dev/video0")
            camera_device = "/dev/video0"
        self.get_logger().info(f"使用摄像头设备: {camera_device}")
        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

        # ROS发布器
        self.pose_publisher = self.create_publisher(Pose, 'paper_center_pose', 10)

        # 后处理队列和线程
        self.postproc_queue = queue.Queue(maxsize=2)
        import threading
        self.running = True
        self.postproc_thread = threading.Thread(target=self.postproc_worker, daemon=True)
        self.postproc_thread.start()

        self.get_logger().info("PaperLocalizerNode started.")

        # 主循环
        self.main_loop()

    def main_loop(self):
        try:
            while rclpy.ok() and self.running:
                ret, frame_data = self.cap.read()
                if not ret or frame_data is None:
                    # 摄像头断开，尝试重新查找并初始化
                    self.get_logger().warn("摄像头读取失败，尝试重新连接摄像头...")
                    camera_device = find_target_camera()
                    if camera_device is None:
                        self.get_logger().warn("未找到目标摄像头，使用默认摄像头 /dev/video0")
                        camera_device = "/dev/video0"
                    self.get_logger().info(f"重新初始化摄像头设备: {camera_device}")
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
                    self.cap.set(cv2.CAP_PROP_FPS, 30)
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
                    time.sleep(0.5)  # 等待设备稳定
                    continue
                if len(frame_data.shape) == 2 and frame_data.shape[0] == 1:
                    jpeg_data = frame_data.flatten().tobytes()
                    input_size = len(jpeg_data)
                    input_addr, ret = acl.himpi.dvpp_malloc(0, input_size)
                    if ret != 0:
                        continue
                    try:
                        vdec_file = np.frombuffer(jpeg_data, dtype=np.uint8)
                        vdec_file_ptr = acl.util.bytes_to_ptr(vdec_file.tobytes())
                        ret = acl.rt.memcpy(input_addr, input_size, vdec_file_ptr, input_size, 1)
                        if ret != 0:
                            acl.himpi.dvpp_free(input_addr)
                            continue
                        stream_dict = {
                            'end_of_frame': 1,
                            'end_of_stream': 0,
                            'need_display': 1,
                            'pts': 0,
                            'len': input_size,
                            'addr': input_addr
                        }
                        img_info, ret = acl.himpi.dvpp_get_image_info(HI_PT_JPEG, {
                            'end_of_frame': 1,
                            'end_of_stream': 0,
                            'need_display': 1,
                            'pts': 0,
                            'len': input_size,
                            'addr': vdec_file_ptr
                        })
                        if ret != 0:
                            acl.himpi.dvpp_free(input_addr)
                            continue
                        out_pic_info = {
                            "width": img_info['width'],
                            "height": img_info['height'],
                            "width_stride": img_info['width_stride'],
                            "height_stride": img_info['height_stride'],
                            "pixel_format": HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420,
                            "buffer_size": img_info['img_buf_size']
                        }
                        out_buffer, ret = acl.himpi.dvpp_malloc(0, out_pic_info['buffer_size'])
                        if ret != 0:
                            acl.himpi.dvpp_free(input_addr)
                            continue
                        out_pic_info['vir_addr'] = out_buffer
                        ret = acl.himpi.vdec_send_stream(CHANNEL_ID, stream_dict, out_pic_info, -1)
                        if ret != 0:
                            acl.himpi.dvpp_free(out_buffer)
                            acl.himpi.dvpp_free(input_addr)
                            continue
                        frame_info, supplement, ret_stream, ret = acl.himpi.vdec_get_frame(CHANNEL_ID, 1000)
                        if ret != 0:
                            acl.himpi.dvpp_free(out_buffer)
                            acl.himpi.dvpp_free(input_addr)
                            continue
                        dec_result = frame_info['v_frame']['frame_flag']
                        if dec_result == 0:
                            input_np = yuv420sp_device_to_numpy(frame_info['v_frame']['virt_addr'][0], img_info)
                            acl.himpi.dvpp_free(frame_info['v_frame']['virt_addr'][0])
                            if input_np is not None:
                                result = self.model.execute([input_np])
                                # 推理结果送入后处理队列（只保留最新）
                                try:
                                    self.postproc_queue.put_nowait(result)
                                except queue.Full:
                                    try:
                                        self.postproc_queue.get_nowait()
                                    except queue.Empty:
                                        pass
                                    self.postproc_queue.put_nowait(result)
                            acl.himpi.vdec_release_frame(CHANNEL_ID, frame_info)
                            acl.himpi.dvpp_free(out_buffer)
                        else:
                            acl.himpi.vdec_release_frame(CHANNEL_ID, frame_info)
                            acl.himpi.dvpp_free(out_buffer)
                    finally:
                        acl.himpi.dvpp_free(input_addr)
                rclpy.spin_once(self, timeout_sec=0.001)
        except KeyboardInterrupt:
            self.running = False
        finally:
            self.cleanup()

    def postproc_worker(self):
        while self.running:
            try:
                pred = self.postproc_queue.get()
                if pred is None:
                    continue
                center, mask_shape, conf, mask_bbox = postprocess_mask_center(pred)
                pose_msg = Pose()
                # 使用前面定义的参考点参数
                reference_x, reference_y = REFERENCE_X, REFERENCE_Y
                if center is not None:
                    center_x, center_y = center
                    pose_msg.position.x = float(center_x - reference_x)
                    pose_msg.position.y = float(center_y - reference_y)
                    # 检查参考点是否在掩膜内
                    if mask_bbox is not None and 0 <= reference_y < mask_bbox.shape[0] and 0 <= reference_x < mask_bbox.shape[1]:
                        pose_msg.position.z = 1.0 if mask_bbox[reference_y, reference_x] > 0 else 0.0
                    else:
                        pose_msg.position.z = 0.0
                else:
                    pose_msg.position.x = 0.0
                    pose_msg.position.y = 0.0
                    pose_msg.position.z = -1.0
                pose_msg.orientation.x = 0.0
                pose_msg.orientation.y = 0.0
                pose_msg.orientation.z = 0.0
                pose_msg.orientation.w = 1.0
                self.pose_publisher.publish(pose_msg)
            except Exception as e:
                pass

    def cleanup(self):
        self.running = False
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        acl.himpi.vdec_stop_recv_stream(CHANNEL_ID)
        acl.himpi.vdec_destroy_chn(CHANNEL_ID)
        acl.rt.destroy_stream(self.stream)
        acl.himpi.sys_exit()
        acl.finalize()
        try:
            self.postproc_queue.put(None)
            self.postproc_thread.join(timeout=1.0)
        except Exception:
            pass

def main(args=None):
    # 设置最高优先级
    try:
        os.nice(-20)
    except Exception as e:
        pass
    rclpy.init(args=args)
    node = PaperLocalizerNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()