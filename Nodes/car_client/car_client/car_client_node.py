#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import json
import threading
import time
from .car_status import CarStatusManager
from .car_control import CarController

class CarClientNode(Node):
    """
    小车客户端节点，用于响应网站请求并控制小车
    """
    def __init__(self):
        super().__init__('car_client_node')
        
        # 初始化状态管理器和控制器
        self.status_manager = CarStatusManager(self)
        self.car_controller = CarController(self)
        
        # 创建订阅器用于接收网站请求
        self.request_subscription = self.create_subscription(
            String,
            'car_request',
            self.handle_request_callback,
            10
        )
        
        # 创建发布器用于返回响应
        self.response_publisher = self.create_publisher(
            String,
            'car_response',
            10
        )
        
        # 创建状态发布器，定期发布小车状态
        self.status_publisher = self.create_publisher(
            String,
            'car_status',
            10
        )
        
        # 定时器，定期发布状态信息
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # 当前执行的任务
        self.current_task = None
        self.task_lock = threading.Lock()
        
        self.get_logger().info('小车客户端节点已启动')
    
    def handle_request_callback(self, msg):
        """处理来自网站的请求"""
        try:
            request_data = json.loads(msg.data)
            command = request_data.get('command', '')
            params = request_data.get('params', {})
            request_id = request_data.get('id', '')
            
            self.get_logger().info(f'收到请求: {command}, ID: {request_id}')
            
            # 根据命令类型处理请求
            response = self.process_command(command, params, request_id)
            
            # 发布响应
            response_msg = String()
            response_msg.data = json.dumps(response)
            self.response_publisher.publish(response_msg)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析错误: {str(e)}')
            self.send_error_response('JSON格式错误', request_id if 'request_id' in locals() else '')
        except Exception as e:
            self.get_logger().error(f'处理请求错误: {str(e)}')
            self.send_error_response(str(e), request_id if 'request_id' in locals() else '')
    
    def process_command(self, command, params, request_id):
        """处理具体的命令"""
        response = {
            'id': request_id,
            'status': 'success',
            'data': {},
            'timestamp': time.time()
        }
        
        try:
            if command == 'get_status':
                # 获取小车状态
                response['data'] = self.status_manager.get_all_status()
                
            elif command == 'move':
                # 移动控制
                direction = params.get('direction', 'stop')
                duration = params.get('duration', 1.0)
                speed = params.get('speed', 0.2)
                result = self.car_controller.move(direction, duration, speed)
                response['data'] = {'result': result}
                
            elif command == 'rotate':
                # 旋转控制
                direction = params.get('direction', 'left')
                angle = params.get('angle', 90.0)
                result = self.car_controller.rotate(direction, angle)
                response['data'] = {'result': result}
                
            elif command == 'arm_control':
                # 机械臂控制
                angle = params.get('angle', 0.0)
                result = self.car_controller.control_arm(angle)
                response['data'] = {'result': result}
                
            elif command == 'claw_control':
                # 爪子控制
                action = params.get('action', 'grasp')  # grasp or release
                result = self.car_controller.control_claw(action)
                response['data'] = {'result': result}
                
            elif command == 'stop':
                # 停止所有动作
                self.car_controller.stop_all()
                response['data'] = {'result': '已停止所有动作'}
                
            else:
                response['status'] = 'error'
                response['data'] = {'error': f'未知命令: {command}'}
                
        except Exception as e:
            response['status'] = 'error'
            response['data'] = {'error': str(e)}
        
        return response
    
    def send_error_response(self, error_msg, request_id):
        """发送错误响应"""
        error_response = {
            'id': request_id,
            'status': 'error',
            'data': {'error': error_msg},
            'timestamp': time.time()
        }
        
        response_msg = String()
        response_msg.data = json.dumps(error_response)
        self.response_publisher.publish(response_msg)
    
    def publish_status(self):
        """定期发布小车状态"""
        try:
            status_data = self.status_manager.get_all_status()
            status_msg = String()
            status_msg.data = json.dumps({
                'type': 'status_update',
                'data': status_data,
                'timestamp': time.time()
            })
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'发布状态错误: {str(e)}')


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
        node.car_controller.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
