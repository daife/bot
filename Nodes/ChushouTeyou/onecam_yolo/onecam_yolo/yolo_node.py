import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import acl
import os
import pyudev
from acllite_utils import *
from constants import *
from acllite_imageproc import AclLiteImageProc
from acllite_model import AclLiteModel
from acllite_resource import resource_list
from geometry_msgs.msg import Pose

# ==== 宏定义参数（直接修改这里即可） ====
MODEL_PATH = "/home/HwHiAiUser/wlw_yolo/yolo11n-new.om"
CAMERA_VENDOR_ID = "0c45"
CAMERA_MODEL_ID = "6368"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
INPUT_SIZE = 640
CORRECT_DISTORTION = True
PUBLISH_TOPIC = '/onecam_yolo/image'
POSE_TOPIC = '/onecam_yolo/target_pose'
TARGET_CLASS_ID = 0  # 目标物类别ID

CAMERA_MATRIX = np.array([
    [465.13093,   0.     , 324.81802],
    [  0.     , 466.33628, 242.54136],
    [  0.     ,   0.     ,   1.     ]
])
DISTORTION_COEFFS = np.array([-0.374992, 0.133505, 0.002906, -0.002975, 0.000000])

def find_camera_by_vid_pid(vendor_id, product_id):
    context = pyudev.Context()
    for device in context.list_devices(subsystem='video4linux'):
        if device.device_node and device.device_node.startswith('/dev/video'):
            vid = device.get('ID_VENDOR_ID', '').lower()
            pid = device.get('ID_MODEL_ID', '').lower()
            if vid == vendor_id.lower() and pid == product_id.lower():
                return device.device_node
    return None

class AclLiteResource:
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

class YOLO11s:
    def __init__(self, model_path, input_size=640, correct_distortion=True):
        self.model_path = model_path
        self.input_size = input_size
        self.model = None
        self.dvpp = None
        self.correct_distortion = correct_distortion
        self.camera_matrix = CAMERA_MATRIX
        self.dist_coeffs = DISTORTION_COEFFS

    def init(self):
        self.dvpp = AclLiteImageProc()
        self.model = AclLiteModel(self.model_path)
        return const.SUCCESS

    def preprocess(self, frame):
        undistorted_frame = None
        if self.correct_distortion:
            undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            frame = undistorted_frame
        h, w = frame.shape[:2]
        scale = min(self.input_size/w, self.input_size/h)
        nh, nw = int(h*scale), int(w*scale)
        img = cv2.resize(frame, (nw, nh))
        top = (self.input_size - nh) // 2
        bottom = self.input_size - nh - top
        left = (self.input_size - nw) // 2
        right = self.input_size - nw - left
        img = cv2.copyMakeBorder(img, top, bottom, left, right, 
                                cv2.BORDER_CONSTANT, value=(114,114,114))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose(2,0,1)[np.newaxis].astype(np.float32)/255
        return img, (h, w), (top, left, scale), undistorted_frame

    def postprocess(self, pred, orig_shape, pad_info):
        CONF_THRESH = 0.1
        IOU_THRESH = 0.9
        arr = pred[0]
        if arr.ndim == 3 and arr.shape[0] == 1:
            arr = arr.squeeze(0)
        # 直接处理(8, 8400)格式
        assert arr.shape == (8, 8400), f"Unexpected pred shape: {arr.shape}"
        detections = []
        for i in range(arr.shape[1]):
            conf = arr[4, i]
            if conf < CONF_THRESH:
                continue
            cx, cy, w, h = arr[0, i], arr[1, i], arr[2, i], arr[3, i]
            cls_scores = arr[5:9, i]
            class_id = np.argmax(cls_scores)
            score = cls_scores[class_id]
            if score < CONF_THRESH:
                continue
            cx = (cx - pad_info[1]) / pad_info[2]
            cy = (cy - pad_info[0]) / pad_info[2]
            w = w / pad_info[2]
            h = h / pad_info[2]
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)
            detections.append([x1, y1, x2, y2, score, class_id])
        best_per_class = {}
        for det in detections:
            x1, y1, x2, y2, score, class_id = det
            if (class_id not in best_per_class) or (score > best_per_class[class_id][4]):
                best_per_class[class_id] = det
        return list(best_per_class.values())

class OneCamYoloNode(Node):
    def __init__(self):
        super().__init__('onecam_yolo_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, PUBLISH_TOPIC, 10)
        self.pose_pub = self.create_publisher(Pose, POSE_TOPIC, 10)

        self.get_logger().info(f"查找摄像头: vendor={CAMERA_VENDOR_ID}, model={CAMERA_MODEL_ID}")
        camera_dev = find_camera_by_vid_pid(CAMERA_VENDOR_ID, CAMERA_MODEL_ID)
        if camera_dev is None:
            self.get_logger().error("未找到指定摄像头设备")
            raise RuntimeError("Camera not found")
        self.get_logger().info(f"使用摄像头设备: {camera_dev}")

        # 检查模型文件是否存在
        if os.path.exists(MODEL_PATH):
            self.get_logger().info(f"模型文件存在: {os.path.abspath(MODEL_PATH)}")
        else:
            self.get_logger().error(f"模型文件不存在: {os.path.abspath(MODEL_PATH)}")
            raise RuntimeError("Model file not found")

        self.acl_resource = AclLiteResource()
        self.acl_resource.init()
        self.yolo = YOLO11s(MODEL_PATH, input_size=INPUT_SIZE, correct_distortion=CORRECT_DISTORTION)
        self.yolo.init()

        self.cap = cv2.VideoCapture(camera_dev)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        self.timer = self.create_timer(0.05, self.process_frame)  # 20Hz

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("摄像头读取失败")
            return
        img, orig_shape, pad_info, undistorted_frame = self.yolo.preprocess(frame)
        # 推理
        pred = self.yolo.model.execute([img])
        # 后处理
        detections = self.yolo.postprocess(pred, orig_shape, pad_info)
        # 使用校正后的图像来显示
        display_frame = undistorted_frame if undistorted_frame is not None else frame
        # 绘制结果
        target_found = False
        dx = dy = 0.0
        img_h, img_w = orig_shape
        cx_img = img_w // 2
        cy_img = img_h // 2
        for det in detections:
            x1, y1, x2, y2, conf, cls_id = det
            label = f"{cls_id} {conf:.2f}"
            cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(display_frame, label, (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if int(cls_id) == TARGET_CLASS_ID and not target_found:
                # 只取第一个目标
                target_found = True
                target_cx = (x1 + x2) // 2
                target_cy = (y1 + y2) // 2
                dx = float(target_cx - cx_img)
                dy = float(target_cy - cy_img)
        # 发布pose消息
        pose_msg = Pose()
        pose_msg.position.x = dx
        pose_msg.position.y = dy
        pose_msg.position.z = 1.0 if target_found else 0.0
        self.pose_pub.publish(pose_msg)
        # 发布图像
        msg = self.bridge.cv2_to_imgmsg(display_frame, encoding='bgr8')
        self.publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OneCamYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
