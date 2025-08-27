#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time
import cv2
import hmac
import hashlib
import base64
import requests
from datetime import datetime, timezone
from pathlib import Path
import paho.mqtt.client as mqtt
from .car_status import CarStatusManager
from .car_control import CarController

class CarClientNode(Node):
    """
    小车客户端节点，响应MQTT服务器指令并控制小车
    """
    def __init__(self):
        super().__init__('car_client_node')
        
        # 初始化状态管理器和控制器
        self.status_manager = CarStatusManager(self)
        self.car_controller = CarController(self)
        
        # MQTT连接信息（从mqtt copy.py复制）
        self.MQTT_SERVER = "96c9c56aa6.st1.iotda-device.cn-north-4.myhuaweicloud.com"
        self.MQTT_PORT = 1883
        self.MQTT_USERNAME = "68a82f89d582f2001847e650_car1"
        self.CLIENT_ID = "68a82f89d582f2001847e650_car1_0_0_2025082209"
        self.MQTT_PASSWORD = "fdf6efa3d8528028fd8909380983a5ca961750c264421f22f28e7da0a8d41680"
        
        # MQTT主题
        self.ALINK_TOPIC_PROP_POST = "$oc/devices/68a82f89d582f2001847e650_car1/sys/properties/report"
        self.ALINK_TOPIC_CMD = "$oc/devices/68a82f89d582f2001847e650_car1/sys/commands/#"
        
        # 华为云OBS参数
        self.AK = "HPUAFFSKOKVIL1WRMPAN"
        self.SK = "Yl4g8Q9UhLxEgzFO7SUI2ilxCQAJfEg8j1PTXzjW"
        self.bucket_name = "rui1"
        
        # 初始化MQTT客户端
        self.mqtt_client = self.init_mqtt()
        
        # 任务锁
        self.task_lock = threading.Lock()
        
        # 启动MQTT连接
        self.mqtt_thread = threading.Thread(target=self.mqtt_loop, daemon=True)
        self.mqtt_thread.start()
        
        self.get_logger().info('小车客户端节点已启动，MQTT连接已建立')
    
    def init_mqtt(self):
        """初始化MQTT客户端"""
        client = mqtt.Client(client_id=self.CLIENT_ID, protocol=mqtt.MQTTv311)
        client.username_pw_set(self.MQTT_USERNAME, self.MQTT_PASSWORD)
        client.on_connect = self.on_mqtt_connect
        client.on_message = self.on_mqtt_message
        return client
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        if rc == 0:
            self.get_logger().info("已连接到MQTT服务器")
            client.subscribe(self.ALINK_TOPIC_CMD)
            self.get_logger().info(f"已订阅主题: {self.ALINK_TOPIC_CMD}")
        else:
            self.get_logger().error(f"MQTT连接失败，返回码: {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """MQTT消息回调"""
        try:
            command_data = json.loads(msg.payload.decode())
            self.get_logger().info(f"收到MQTT指令: {command_data}")
            
            # 处理指令
            self.process_mqtt_command(command_data)
            
        except Exception as e:
            self.get_logger().error(f"处理MQTT消息错误: {str(e)}")
    
    def process_mqtt_command(self, command_data):
        """处理MQTT指令"""
        try:
            service_id = command_data.get("service_id")
            cmd_name = command_data.get("command_name")
            paras = command_data.get("paras", {})
            
            self.get_logger().info(f"处理指令: {cmd_name}, 参数: {paras}")
            
            if cmd_name == "upload":
                # 上传状态和图像
                self.handle_upload_command()
            elif cmd_name in ["go_forward", "go_back", "go_left", "go_right", 
                             "A", "B", "X", "Y", "L", "R"]:
                # 控制指令
                self.handle_control_command(cmd_name, paras)
            else:
                self.get_logger().warning(f"未知指令: {cmd_name}")
                
        except Exception as e:
            self.get_logger().error(f"处理MQTT指令错误: {str(e)}")
    
    def handle_upload_command(self):
        """处理上传指令"""
        try:
            self.get_logger().info("开始执行上传任务")
            
            # 获取小车状态
            status_data = self.status_manager.get_all_status()
            
            # 上传状态到MQTT
            self.upload_status_to_mqtt(status_data)
            
            # 处理并上传识别画面
            self.upload_recognition_image()
            
            # 上传固定图片
            self.upload_fixed_image()
            
            self.get_logger().info("上传任务完成")
            
        except Exception as e:
            self.get_logger().error(f"上传任务失败: {str(e)}")
    
    def handle_control_command(self, cmd_name, paras):
        """处理控制指令"""
        try:
            with self.task_lock:
                if cmd_name == "go_forward":
                    result = self.car_controller.move("forward", duration=1.0)
                elif cmd_name == "go_back":
                    result = self.car_controller.move("backward", duration=1.0)
                elif cmd_name == "go_left":
                    result = self.car_controller.move("left", duration=1.0)
                elif cmd_name == "go_right":
                    result = self.car_controller.move("right", duration=1.0)
                elif cmd_name == "A":  # 下移机械臂
                    current_angle = self.car_controller.arm_angle
                    new_angle = max(self.car_controller.arm_min_angle, 
                                  current_angle - self.car_controller.arm_step)
                    result = self.car_controller.control_arm(new_angle)
                elif cmd_name == "B":  # 右转
                    result = self.car_controller.rotate("right", angle=90.0)
                elif cmd_name == "X":  # 左转
                    result = self.car_controller.rotate("left", angle=90.0)
                elif cmd_name == "Y":  # 上移机械臂
                    current_angle = self.car_controller.arm_angle
                    new_angle = min(self.car_controller.arm_max_angle, 
                                  current_angle + self.car_controller.arm_step)
                    result = self.car_controller.control_arm(new_angle)
                elif cmd_name == "L":  # 机械爪释放
                    result = self.car_controller.control_claw("release")
                elif cmd_name == "R":  # 机械爪抓取
                    result = self.car_controller.control_claw("grasp")
                
                self.get_logger().info(f"控制指令 {cmd_name} 执行结果: {result}")
                
        except Exception as e:
            self.get_logger().error(f"控制指令 {cmd_name} 执行失败: {str(e)}")
    
    def upload_status_to_mqtt(self, status_data):
        """上传状态到MQTT"""
        try:
            # 构造华为云IoT格式的属性数据
            properties = {
                "services": [{
                    "service_id": "car_status",
                    "properties": {
                        "机械臂角度": str(status_data.get('arm_angle', 0.0)),
                        "小车全局坐标": f"x:{status_data['global_position']['x']:.2f}, y:{status_data['global_position']['y']:.2f}, z:{status_data['global_position']['z']:.2f}",
                        "物体画面坐标": f"x:{status_data['target_position']['x']:.2f}, y:{status_data['target_position']['y']:.2f}",
                        "小车姿态raw,yaw,pitch": f"roll:{status_data['pose']['roll']:.2f}, yaw:{status_data['pose']['yaw']:.2f}, pitch:{status_data['pose']['pitch']:.2f}",
                        "是否识别到指定物体": str(status_data.get('target_detected', False)),
                        "爪子是否处于抓取状态": status_data.get('claw_state', 'released'),
                        "小车姿态raw": str(status_data['pose']['roll']),
                        "小车姿态yaw": str(status_data['pose']['yaw']),
                        "小车姿态pitch": str(status_data['pose']['pitch']),
                        "小车速度Vx": str(status_data['velocity']['vx']),
                        "小车速度Vy": str(status_data['velocity']['vy']),
                        "小车速度w": str(status_data['velocity']['vyaw']),
                    }
                }]
            }
            
            payload = json.dumps(properties, separators=(',', ':'))
            result = self.mqtt_client.publish(self.ALINK_TOPIC_PROP_POST, payload)
            
            self.get_logger().info(f"状态上传结果: {result.is_published()}")
            
        except Exception as e:
            self.get_logger().error(f"状态上传失败: {str(e)}")
    
    def upload_recognition_image(self):
        """上传识别画面"""
        try:
            # 获取最新的识别画面
            latest_image = self.status_manager.get_latest_image()
            
            if latest_image is not None:
                # 保存为PNG文件
                recognition_path = Path("recognition_image.png")
                cv2.imwrite(str(recognition_path), latest_image)
                
                # 上传到OBS
                self.upload_image_to_obs(recognition_path, "recognition_image.png")
                
                self.get_logger().info("识别画面上传完成")
            else:
                self.get_logger().warning("没有可用的识别画面")
                
        except Exception as e:
            self.get_logger().error(f"识别画面上传失败: {str(e)}")
    
    def upload_fixed_image(self):
        """上传固定图片"""
        try:
            fixed_image_path = Path("2025.png")
            if fixed_image_path.exists():
                self.upload_image_to_obs(fixed_image_path, "image.png")
                self.get_logger().info("固定图片上传完成")
            else:
                self.get_logger().warning("固定图片 2025.png 不存在")
                
        except Exception as e:
            self.get_logger().error(f"固定图片上传失败: {str(e)}")
    
    def upload_image_to_obs(self, local_path, object_key):
        """上传图片到华为云OBS（从mqtt copy.py复制）"""
        try:
            method = "PUT"
            resource = f"/{self.bucket_name}/{object_key}"
            
            # 读取文件数据
            data = Path(local_path).read_bytes()
            
            # 计算 Content-MD5
            content_md5_b64 = base64.b64encode(hashlib.md5(data).digest()).decode("utf-8")
            content_type = "image/png"
            
            # 生成授权头
            auth, headers, sts = self.get_obs_authorization(
                method, resource,
                content_type=content_type,
                content_md5=content_md5_b64
            )
            
            headers["Content-Length"] = str(len(data))
            
            # 构造URL并上传
            url = f"https://{self.bucket_name}.obs.cn-north-4.myhuaweicloud.com/{object_key}"
            resp = requests.put(url, headers=headers, data=data, timeout=60)
            
            if resp.status_code == 200:
                self.get_logger().info(f"图片 {object_key} 上传成功")
            else:
                self.get_logger().error(f"图片 {object_key} 上传失败: {resp.status_code}")
                
        except Exception as e:
            self.get_logger().error(f"图片上传异常: {str(e)}")
    
    def rfc1123_date(self):
        """生成RFC1123格式时间（从mqtt copy.py复制）"""
        return datetime.now(timezone.utc).strftime('%a, %d %b %Y %H:%M:%S GMT')
    
    def get_obs_authorization(self, method, resource, *, content_type="", content_md5="", headers=None):
        """生成OBS授权（从mqtt copy.py复制）"""
        cano_headers = ""
        date = self.rfc1123_date()
        string_to_sign = f"{method}\n{content_md5}\n{content_type}\n{date}\n{cano_headers}{resource}"
        sig = base64.b64encode(
            hmac.new(self.SK.encode("utf-8"), string_to_sign.encode("utf-8"), hashlib.sha1).digest()
        ).decode("utf-8")
        auth = f"OBS {self.AK}:{sig}"
        signed_headers = {"Authorization": auth, "Date": date}
        if content_type:
            signed_headers["Content-Type"] = content_type
        if content_md5:
            signed_headers["Content-MD5"] = content_md5
        return auth, signed_headers, string_to_sign
    
    def mqtt_loop(self):
        """MQTT循环线程"""
        try:
            self.mqtt_client.connect(self.MQTT_SERVER, self.MQTT_PORT, 60)
            self.mqtt_client.loop_forever()
        except Exception as e:
            self.get_logger().error(f"MQTT连接错误: {str(e)}")
    
    def destroy_node(self):
        """节点销毁时清理资源"""
        self.get_logger().info("正在关闭MQTT连接...")
        if hasattr(self, 'mqtt_client'):
            self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = CarClientNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        # 停止小车
        if hasattr(node, 'car_controller'):
            node.car_controller.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
