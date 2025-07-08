#!/usr/bin/env python3

import sys
import os
import subprocess
import threading
import time
import json
import signal
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Float32, Int32

# 尝试导入CV Bridge，如果失败则给出提示
try:
    from cv_bridge import CvBridge
except ImportError:
    print("Warning: cv_bridge not available. Image viewing will be disabled.")
    CvBridge = None

# 尝试导入PyQt5，如果失败则给出提示
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                 QHBoxLayout, QTabWidget, QTreeWidget, QTreeWidgetItem,
                                 QPushButton, QLabel, QTextEdit, QComboBox, QGroupBox,
                                 QScrollArea, QSplitter, QFrame, QMessageBox, QProgressBar,
                                 QCheckBox, QSpinBox, QDoubleSpinBox)
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread, QObject
    from PyQt5.QtGui import QPixmap, QFont, QIcon, QPalette, QColor, QImage
    PYQT5_AVAILABLE = True
except ImportError:
    print("Error: PyQt5 not available. Please install it with:")
    print("sudo apt-get install python3-pyqt5")
    PYQT5_AVAILABLE = False

# 尝试导入OpenCV，如果失败则给出提示
try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    print("Warning: OpenCV not available. Image processing will be disabled.")
    OPENCV_AVAILABLE = False


class RobotUINode(Node):
    """机器人UI控制节点"""
    
    def __init__(self):
        super().__init__('robot_ui_node')
        
        # CV Bridge for image conversion
        if CvBridge is not None:
            self.cv_bridge = CvBridge()
        else:
            self.cv_bridge = None
        
        # 存储图像数据
        self.latest_images = {}
        
        # 创建图像订阅器（动态创建）
        self.image_subscriptions = {}
        
        # 存储话题数据
        self.topic_data = {}
        
        # 其他订阅器
        self.other_subscriptions = {}
        
        # 控制器发布器
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 存储当前激活的控制器
        self.active_controller = None
        self.controller_processes = {}
        
        self.get_logger().info('Robot UI Node initialized')

    def subscribe_to_image_topic(self, topic_name: str):
        """订阅图像话题"""
        if self.cv_bridge is None:
            self.get_logger().warn('CV Bridge not available, cannot subscribe to image topics')
            return
            
        if topic_name not in self.image_subscriptions:
            self.image_subscriptions[topic_name] = self.create_subscription(
                Image,
                topic_name,
                lambda msg, topic=topic_name: self.image_callback(msg, topic),
                10
            )
            self.get_logger().info(f'Subscribed to image topic: {topic_name}')

    def image_callback(self, msg, topic_name):
        """图像话题回调"""
        if self.cv_bridge is None:
            return
            
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_images[topic_name] = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image from {topic_name}: {e}')

    def subscribe_to_topic(self, topic_name: str, msg_type: str):
        """订阅其他类型话题"""
        # 这里可以根据msg_type动态创建订阅器
        # 简化实现，主要处理常见的消息类型
        pass

    def get_node_list(self) -> List[str]:
        """获取当前运行的节点列表"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                   capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return [node.strip() for node in result.stdout.split('\n') if node.strip()]
            return []
        except Exception as e:
            self.get_logger().error(f'Error getting node list: {e}')
            return []

    def get_topic_list(self) -> Dict:
        """获取话题列表和类型信息"""
        try:
            result = subprocess.run(['ros2', 'topic', 'list', '-t'], 
                                   capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                topics = {}
                for line in result.stdout.split('\n'):
                    if line.strip() and '[' in line:
                        parts = line.strip().split('[')
                        topic_name = parts[0].strip()
                        msg_type = parts[1].replace(']', '').strip()
                        topics[topic_name] = msg_type
                return topics
            return {}
        except Exception as e:
            self.get_logger().error(f'Error getting topic list: {e}')
            return {}

    def start_controller(self, controller_type: str):
        """启动控制器"""
        # 停止当前控制器
        self.stop_current_controller()
        
        try:
            if controller_type == 'handle':
                process = subprocess.Popen(['ros2', 'run', 'handlecontroler', 'handle_control_node'])
                self.controller_processes['handle'] = process
                self.active_controller = 'handle'
                
            elif controller_type == 'keyboard':
                process = subprocess.Popen(['ros2', 'run', 'keyboardcontroler', 'keyboard_control_node'])
                self.controller_processes['keyboard'] = process
                self.active_controller = 'keyboard'
                
            # 其他控制器可以在这里添加
            
            self.get_logger().info(f'Started {controller_type} controller')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error starting {controller_type} controller: {e}')
            return False

    def stop_current_controller(self):
        """停止当前控制器"""
        if self.active_controller and self.active_controller in self.controller_processes:
            try:
                process = self.controller_processes[self.active_controller]
                process.terminate()
                process.wait(timeout=5)
                del self.controller_processes[self.active_controller]
                self.get_logger().info(f'Stopped {self.active_controller} controller')
            except Exception as e:
                self.get_logger().error(f'Error stopping controller: {e}')
            finally:
                self.active_controller = None

    def kill_node(self, node_name: str):
        """终止指定节点"""
        try:
            # 使用pkill终止节点进程
            subprocess.run(['pkill', '-f', node_name], timeout=5)
            self.get_logger().info(f'Killed node: {node_name}')
            return True
        except Exception as e:
            self.get_logger().error(f'Error killing node {node_name}: {e}')
            return False

    def subscribe_to_other_topic(self, topic_name: str, msg_type: str):
        """订阅其他类型话题"""
        # 先清理现有的订阅器，避免重复订阅
        self.cleanup_subscriptions()
        
        try:
            # 根据消息类型创建相应的订阅器
            if 'String' in msg_type:
                self.other_subscriptions[topic_name] = self.create_subscription(
                    String,
                    topic_name,
                    lambda msg, topic=topic_name: self.other_topic_callback(msg, topic, 'String'),
                    10
                )
            elif 'Bool' in msg_type:
                self.other_subscriptions[topic_name] = self.create_subscription(
                    Bool,
                    topic_name,
                    lambda msg, topic=topic_name: self.other_topic_callback(msg, topic, 'Bool'),
                    10
                )
            elif 'Float32' in msg_type:
                self.other_subscriptions[topic_name] = self.create_subscription(
                    Float32,
                    topic_name,
                    lambda msg, topic=topic_name: self.other_topic_callback(msg, topic, 'Float32'),
                    10
                )
            elif 'Int32' in msg_type:
                self.other_subscriptions[topic_name] = self.create_subscription(
                    Int32,
                    topic_name,
                    lambda msg, topic=topic_name: self.other_topic_callback(msg, topic, 'Int32'),
                    10
                )
            elif 'Twist' in msg_type:
                self.other_subscriptions[topic_name] = self.create_subscription(
                    Twist,
                    topic_name,
                    lambda msg, topic=topic_name: self.other_topic_callback(msg, topic, 'Twist'),
                    10
                )
            elif 'Odometry' in msg_type:
                self.other_subscriptions[topic_name] = self.create_subscription(
                    Odometry,
                    topic_name,
                    lambda msg, topic=topic_name: self.other_topic_callback(msg, topic, 'Odometry'),
                    10
                )
            elif 'TFMessage' in msg_type:
                try:
                    from tf2_msgs.msg import TFMessage
                    self.other_subscriptions[topic_name] = self.create_subscription(
                        TFMessage,
                        topic_name,
                        lambda msg, topic=topic_name: self.other_topic_callback(msg, topic, 'TFMessage'),
                        10
                    )
                except ImportError:
                    self.get_logger().info(f'tf2_msgs not available for {topic_name}, will use alternative display')
                    return False
            elif 'JointState' in msg_type:
                try:
                    from sensor_msgs.msg import JointState
                    self.other_subscriptions[topic_name] = self.create_subscription(
                        JointState,
                        topic_name,
                        lambda msg, topic=topic_name: self.other_topic_callback(msg, topic, 'JointState'),
                        10
                    )
                except ImportError:
                    self.get_logger().info(f'sensor_msgs not available for {topic_name}, will use alternative display')
                    return False
            else:
                # 对于不支持的类型，不输出错误信息
                self.get_logger().info(f'Message type {msg_type} not directly supported, will use alternative display')
                return False
                
            self.get_logger().info(f'Subscribed to topic: {topic_name} ({msg_type})')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error subscribing to topic {topic_name}: {e}')
            return False
    def cleanup_subscriptions(self):
        """清理所有其他话题订阅器"""
        for topic_name, subscription in list(self.other_subscriptions.items()):
            try:
                subscription.destroy()
            except Exception as e:
                self.get_logger().warning(f'Error destroying subscription for {topic_name}: {e}')
        self.other_subscriptions.clear()

    def other_topic_callback(self, msg, topic_name, msg_type):
        """其他话题回调"""
        try:
            if msg_type == 'String':
                data = msg.data
            elif msg_type == 'Bool':
                data = str(msg.data)
            elif msg_type == 'Float32':
                data = f"{msg.data:.6f}"
            elif msg_type == 'Int32':
                data = str(msg.data)
            elif msg_type == 'Twist':
                data = f"Linear: x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, z={msg.linear.z:.3f}\n"
                data += f"Angular: x={msg.angular.x:.3f}, y={msg.angular.y:.3f}, z={msg.angular.z:.3f}"
            elif msg_type == 'Odometry':
                pos = msg.pose.pose.position
                ori = msg.pose.pose.orientation
                data = f"Position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}\n"
                data += f"Orientation: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}"
            elif msg_type == 'TFMessage':
                transforms_info = []
                for transform in msg.transforms:
                    frame_id = transform.header.frame_id
                    child_frame_id = transform.child_frame_id
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    transforms_info.append(
                        f"{frame_id} -> {child_frame_id}: "
                        f"pos({translation.x:.3f}, {translation.y:.3f}, {translation.z:.3f}) "
                        f"rot({rotation.x:.3f}, {rotation.y:.3f}, {rotation.z:.3f}, {rotation.w:.3f})"
                    )
                data = "\n".join(transforms_info)
            elif msg_type == 'JointState':
                joint_info = []
                for i, name in enumerate(msg.name):
                    pos = msg.position[i] if i < len(msg.position) else 0.0
                    vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                    effort = msg.effort[i] if i < len(msg.effort) else 0.0
                    joint_info.append(f"{name}: pos={pos:.3f}, vel={vel:.3f}, effort={effort:.3f}")
                data = "\n".join(joint_info)
            else:
                data = str(msg)
                
            # 存储最新数据，只保留最新的10条记录
            if topic_name not in self.topic_data:
                self.topic_data[topic_name] = []
            
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            self.topic_data[topic_name].append(f"[{timestamp}] {data}")
            
            # 保持最新的10条记录
            if len(self.topic_data[topic_name]) > 10:
                self.topic_data[topic_name] = self.topic_data[topic_name][-10:]
                
        except Exception as e:
            self.get_logger().error(f'Error processing topic data for {topic_name}: {e}')


class ROSWorker(QObject):
    """ROS工作线程"""
    nodes_updated = pyqtSignal(list)
    topics_updated = pyqtSignal(dict)
    image_updated = pyqtSignal(str, np.ndarray)
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.running = True
        
    def run(self):
        """工作线程主循环"""
        while self.running:
            try:
                # 更新节点列表
                nodes = self.ros_node.get_node_list()
                self.nodes_updated.emit(nodes)
                
                # 更新话题列表
                topics = self.ros_node.get_topic_list()
                self.topics_updated.emit(topics)
                
                # 发送图像更新
                for topic_name, image in self.ros_node.latest_images.items():
                    self.image_updated.emit(topic_name, image)
                
                time.sleep(1.0)  # 1秒更新一次
                
            except Exception as e:
                print(f"Worker error: {e}")
                
    def stop(self):
        self.running = False


class MainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.image_labels = {}  # 存储图像显示标签
        self.topic_text_displays = {}  # 存储话题文本显示
        self.current_topic_display = None  # 当前显示的话题
        self.topic_update_timer = None  # 话题更新定时器
        self.is_updating_selectors = False  # 防止递归更新选择器
        
        # 设置窗口
        self.setWindowTitle('Robot Control UI')
        self.setGeometry(100, 100, 1200, 800)
        
        # 创建中央widget和标签页
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        
        # 创建标签页widget
        self.tab_widget = QTabWidget()
        layout.addWidget(self.tab_widget)
        
        # 创建四个标签页
        self.create_node_manager_tab()
        self.create_topic_viewer_tab()
        self.create_content_viewer_tab()
        self.create_controller_tab()
        
        # 启动ROS工作线程
        self.setup_ros_worker()
        
        # 设置深色主题样式
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2d2d2d;
                color: #ffffff;
            }
            QTabWidget::pane {
                border: 1px solid #5a5a5a;
                background-color: #3d3d3d;
            }
            QTabBar::tab {
                background-color: #4a4a4a;
                color: #ffffff;
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: #0078d4;
            }
            QTabBar::tab:hover {
                background-color: #5a5a5a;
            }
            QPushButton {
                background-color: #0078d4;
                color: #ffffff;
                border: none;
                padding: 8px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #106ebe;
            }
            QPushButton:pressed {
                background-color: #005a9e;
            }
            QPushButton:disabled {
                background-color: #666666;
                color: #cccccc;
            }
            QTreeWidget {
                background-color: #2d2d2d;
                color: #ffffff;
                border: 1px solid #5a5a5a;
                alternate-background-color: #3d3d3d;
            }
            QTreeWidget::item {
                padding: 4px;
                border-bottom: 1px solid #404040;
            }
            QTreeWidget::item:selected {
                background-color: #0078d4;
            }
            QTreeWidget::item:hover {
                background-color: #4a4a4a;
            }
            QLabel {
                color: #ffffff;
            }
            QTextEdit {
                background-color: #2d2d2d;
                color: #ffffff;
                border: 1px solid #5a5a5a;
                border-radius: 4px;
                padding: 4px;
            }
            QComboBox {
                background-color: #4a4a4a;
                color: #ffffff;
                border: 1px solid #5a5a5a;
                padding: 4px 8px;
                border-radius: 4px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox::down-arrow {
                border: 2px solid #ffffff;
                border-top-color: transparent;
                border-left-color: transparent;
                border-right-color: transparent;
                width: 0px;
                height: 0px;
            }
            QComboBox QAbstractItemView {
                background-color: #4a4a4a;
                color: #ffffff;
                selection-background-color: #0078d4;
            }
            QGroupBox {
                color: #ffffff;
                border: 2px solid #5a5a5a;
                border-radius: 4px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QScrollArea {
                background-color: #2d2d2d;
                border: 1px solid #5a5a5a;
                border-radius: 4px;
            }
        """)

    def setup_ros_worker(self):
        """设置ROS工作线程"""
        self.worker_thread = QThread()
        self.worker = ROSWorker(self.ros_node)
        self.worker.moveToThread(self.worker_thread)
        
        # 连接信号
        self.worker.nodes_updated.connect(self.update_node_list)
        self.worker.topics_updated.connect(self.update_topic_list)
        self.worker.image_updated.connect(self.update_image_display)
        
        # 启动线程
        self.worker_thread.started.connect(self.worker.run)
        self.worker_thread.start()

    def create_node_manager_tab(self):
        """创建节点管理标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 标题
        title = QLabel("Node Manager")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(title)
        
        # 刷新按钮
        refresh_btn = QPushButton("Refresh Nodes")
        refresh_btn.clicked.connect(self.refresh_nodes)
        layout.addWidget(refresh_btn)
        
        # 节点树
        self.node_tree = QTreeWidget()
        self.node_tree.setHeaderLabels(["Node Name", "Status", "Actions"])
        layout.addWidget(self.node_tree)
        
        self.tab_widget.addTab(tab, "Node Manager")

    def create_topic_viewer_tab(self):
        """创建话题查看器标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 标题
        title = QLabel("Topic Viewer")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(title)
        
        # 控制按钮
        btn_layout = QHBoxLayout()
        
        refresh_topics_btn = QPushButton("Refresh Topics")
        refresh_topics_btn.clicked.connect(self.refresh_topics)
        btn_layout.addWidget(refresh_topics_btn)
        
        show_graph_btn = QPushButton("Show RQT Graph")
        show_graph_btn.clicked.connect(self.show_rqt_graph)
        btn_layout.addWidget(show_graph_btn)
        
        layout.addLayout(btn_layout)
        
        # 话题树 - 移除了Publishers和Subscribers列
        self.topic_tree = QTreeWidget()
        self.topic_tree.setHeaderLabels(["Topic Name", "Message Type"])
        layout.addWidget(self.topic_tree)
        
        self.tab_widget.addTab(tab, "Topic Viewer")

    def create_content_viewer_tab(self):
        """创建内容查看器标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 标题
        title = QLabel("Content Viewer")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(title)
        
        # 选择器
        selector_layout = QHBoxLayout()
        
        self.content_type_combo = QComboBox()
        self.content_type_combo.addItems(["Image Topics", "Other Topics", "URDF Viewer", "Navigation Viewer"])
        self.content_type_combo.currentTextChanged.connect(self.on_content_type_changed)
        selector_layout.addWidget(QLabel("Content Type:"))
        selector_layout.addWidget(self.content_type_combo)
        
        self.content_selector = QComboBox()
        self.content_selector.currentTextChanged.connect(self.on_content_selected)
        selector_layout.addWidget(QLabel("Select:"))
        selector_layout.addWidget(self.content_selector)
        
        layout.addLayout(selector_layout)
        
        # 内容显示区域
        self.content_display = QScrollArea()
        self.content_display.setWidgetResizable(True)
        self.content_display.setMinimumHeight(400)
        
        # 默认显示标签
        default_label = QLabel("Select content type and item to display")
        default_label.setAlignment(Qt.AlignCenter)
        default_label.setStyleSheet("font-size: 14px; color: #888;")
        self.content_display.setWidget(default_label)
        
        layout.addWidget(self.content_display)
        
        self.tab_widget.addTab(tab, "Content Viewer")

    def create_controller_tab(self):
        """创建控制器选择器标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 标题
        title = QLabel("Controller Selector")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        layout.addWidget(title)
        
        # 当前控制器状态
        status_group = QGroupBox("Current Status")
        status_layout = QVBoxLayout(status_group)
        
        self.controller_status_label = QLabel("No controller active")
        status_layout.addWidget(self.controller_status_label)
        
        layout.addWidget(status_group)
        
        # 控制器选择
        controller_group = QGroupBox("Available Controllers")
        controller_layout = QVBoxLayout(controller_group)
        
        # 手柄控制器
        handle_layout = QHBoxLayout()
        self.handle_btn = QPushButton("Start Handle Controller")
        self.handle_btn.clicked.connect(lambda: self.start_controller('handle'))
        handle_layout.addWidget(self.handle_btn)
        handle_layout.addWidget(QLabel("Control robot with gamepad"))
        controller_layout.addLayout(handle_layout)
        
        # 键盘控制器
        keyboard_layout = QHBoxLayout()
        self.keyboard_btn = QPushButton("Start Keyboard Controller")
        self.keyboard_btn.clicked.connect(lambda: self.start_controller('keyboard'))
        keyboard_layout.addWidget(self.keyboard_btn)
        keyboard_layout.addWidget(QLabel("Control robot with keyboard"))
        controller_layout.addLayout(keyboard_layout)
        
        # 自动控制器（未完成）
        auto_layout = QHBoxLayout()
        self.auto_btn = QPushButton("Start Auto Controller")
        self.auto_btn.setEnabled(False)
        self.auto_btn.setStyleSheet("background-color: #666; color: #ccc;")
        auto_layout.addWidget(self.auto_btn)
        auto_layout.addWidget(QLabel("Autonomous navigation (Coming Soon)"))
        controller_layout.addLayout(auto_layout)
        
        # 网络控制器（未完成）
        network_layout = QHBoxLayout()
        self.network_btn = QPushButton("Start Network Controller")
        self.network_btn.setEnabled(False)
        self.network_btn.setStyleSheet("background-color: #666; color: #ccc;")
        network_layout.addWidget(self.network_btn)
        network_layout.addWidget(QLabel("Remote network control (Coming Soon)"))
        controller_layout.addLayout(network_layout)
        
        # 停止按钮
        stop_layout = QHBoxLayout()
        self.stop_controller_btn = QPushButton("Stop Current Controller")
        self.stop_controller_btn.clicked.connect(self.stop_controller)
        self.stop_controller_btn.setStyleSheet("background-color: #d13438;")
        stop_layout.addWidget(self.stop_controller_btn)
        controller_layout.addLayout(stop_layout)
        
        layout.addWidget(controller_group)
        
        # 手动控制区域
        manual_group = QGroupBox("Manual Control (Test)")
        manual_layout = QVBoxLayout(manual_group)
        
        # 创建方向控制按钮
        direction_layout = QVBoxLayout()
        
        # 上排按钮
        top_row = QHBoxLayout()
        top_row.addStretch()
        forward_btn = QPushButton("↑")
        forward_btn.pressed.connect(lambda: self.send_velocity(0.2, 0.0, 0.0))
        forward_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        top_row.addWidget(forward_btn)
        top_row.addStretch()
        direction_layout.addLayout(top_row)
        
        # 中间排按钮
        middle_row = QHBoxLayout()
        left_btn = QPushButton("←")
        left_btn.pressed.connect(lambda: self.send_velocity(0.0, 0.2, 0.0))
        left_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        middle_row.addWidget(left_btn)
        
        stop_btn = QPushButton("STOP")
        stop_btn.clicked.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        stop_btn.setStyleSheet("background-color: #d13438;")
        middle_row.addWidget(stop_btn)
        
        right_btn = QPushButton("→")
        right_btn.pressed.connect(lambda: self.send_velocity(0.0, -0.2, 0.0))
        right_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        middle_row.addWidget(right_btn)
        direction_layout.addLayout(middle_row)
        
        # 下排按钮
        bottom_row = QHBoxLayout()
        bottom_row.addStretch()
        backward_btn = QPushButton("↓")
        backward_btn.pressed.connect(lambda: self.send_velocity(-0.2, 0.0, 0.0))
        backward_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        bottom_row.addWidget(backward_btn)
        bottom_row.addStretch()
        direction_layout.addLayout(bottom_row)
        
        manual_layout.addLayout(direction_layout)
        
        # 旋转控制
        rotation_layout = QHBoxLayout()
        rotate_left_btn = QPushButton("Rotate Left")
        rotate_left_btn.pressed.connect(lambda: self.send_velocity(0.0, 0.0, 0.5))
        rotate_left_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        rotation_layout.addWidget(rotate_left_btn)
        
        rotate_right_btn = QPushButton("Rotate Right")
        rotate_right_btn.pressed.connect(lambda: self.send_velocity(0.0, 0.0, -0.5))
        rotate_right_btn.released.connect(lambda: self.send_velocity(0.0, 0.0, 0.0))
        rotation_layout.addWidget(rotate_right_btn)
        
        manual_layout.addLayout(rotation_layout)
        
        layout.addWidget(manual_group)
        
        self.tab_widget.addTab(tab, "Controller")

    def update_node_list(self, nodes):
        """更新节点列表"""
        self.node_tree.clear()
        
        for node_name in nodes:
            item = QTreeWidgetItem([node_name, "Running"])
            
            # 添加终止按钮
            kill_btn = QPushButton("Kill")
            kill_btn.clicked.connect(lambda checked, name=node_name: self.kill_node(name))
            kill_btn.setStyleSheet("background-color: #d13438; max-width: 60px;")
            
            self.node_tree.addTopLevelItem(item)
            self.node_tree.setItemWidget(item, 2, kill_btn)

    def update_topic_list(self, topics):
        """更新话题列表"""
        self.topic_tree.clear()
        
        for topic_name, msg_type in topics.items():
            # 只显示话题名称和消息类型
            item = QTreeWidgetItem([topic_name, msg_type])
            self.topic_tree.addTopLevelItem(item)
            
        # 更新内容选择器
        self.update_content_selectors(topics)

    def update_content_selectors(self, topics):
        """更新内容选择器选项"""
        if self.is_updating_selectors:
            return
            
        self.is_updating_selectors = True
        
        try:
            current_type = self.content_type_combo.currentText()
            current_selection = self.content_selector.currentText()
            
            # 暂时断开信号连接
            self.content_selector.currentTextChanged.disconnect()
            
            self.content_selector.clear()
            
            if current_type == "Image Topics":
                image_topics = [topic for topic, msg_type in topics.items() 
                               if 'Image' in msg_type]
                self.content_selector.addItems(image_topics)
                
                # 订阅新的图像话题
                for topic in image_topics:
                    self.ros_node.subscribe_to_image_topic(topic)
                    
            elif current_type == "Other Topics":
                other_topics = [topic for topic, msg_type in topics.items() 
                               if 'Image' not in msg_type]
                self.content_selector.addItems(other_topics)
                
                # 尝试恢复之前的选择
                if current_selection in other_topics:
                    index = self.content_selector.findText(current_selection)
                    if index >= 0:
                        self.content_selector.setCurrentIndex(index)
            
            # 重新连接信号
            self.content_selector.currentTextChanged.connect(self.on_content_selected)
            
        finally:
            self.is_updating_selectors = False

    def update_image_display(self, topic_name, image):
        """更新图像显示"""
        if not OPENCV_AVAILABLE:
            return
            
        current_selection = self.content_selector.currentText()
        
        if (self.content_type_combo.currentText() == "Image Topics" and 
            current_selection == topic_name):
            
            # 转换图像格式并显示
            height, width, channel = image.shape
            bytes_per_line = 3 * width
            
            # 调整图像大小以适应显示
            max_width = 640
            max_height = 480
            
            scale = min(max_width/width, max_height/height)
            if scale < 1:
                new_width = int(width * scale)
                new_height = int(height * scale)
                image = cv2.resize(image, (new_width, new_height))
            
            # 转换为RGB
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # 创建QPixmap
            q_image = QImage(rgb_image.data, rgb_image.shape[1], rgb_image.shape[0], 
                           rgb_image.strides[0], QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            
            # 更新显示
            if topic_name not in self.image_labels:
                self.image_labels[topic_name] = QLabel()
                self.image_labels[topic_name].setAlignment(Qt.AlignCenter)
                self.image_labels[topic_name].setScaledContents(False)
            
            self.image_labels[topic_name].setPixmap(pixmap)
            self.content_display.setWidget(self.image_labels[topic_name])

    def refresh_nodes(self):
        """刷新节点列表"""
        nodes = self.ros_node.get_node_list()
        self.update_node_list(nodes)

    def refresh_topics(self):
        """刷新话题列表"""
        topics = self.ros_node.get_topic_list()
        self.update_topic_list(topics)

    def show_rqt_graph(self):
        """显示RQT图形界面"""
        try:
            subprocess.Popen(['rqt_graph'])
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to start rqt_graph: {e}")


    def on_content_type_changed(self, content_type):
        """内容类型改变时的处理"""
        if self.is_updating_selectors:
            return
            
        # 停止定时器
        if self.topic_update_timer:
            self.topic_update_timer.stop()
            self.topic_update_timer = None
        
        # 重置当前显示
        self.current_topic_display = None
        
        # 清理所有订阅
        self.ros_node.cleanup_subscriptions()
        
        # 暂时断开信号连接
        self.content_selector.currentTextChanged.disconnect()
        
        # 清空选择器
        self.content_selector.clear()
        
        if content_type == "URDF Viewer":
            self.content_selector.addItems(["Robot URDF"])
        elif content_type == "Navigation Viewer":
            self.content_selector.addItems(["Costmap", "Path", "Robot Pose"])
        
        # 重新连接信号
        self.content_selector.currentTextChanged.connect(self.on_content_selected)

    def on_content_selected(self, selection):
        """内容选择改变时的处理"""
        if self.is_updating_selectors or not selection:
            return
            
        content_type = self.content_type_combo.currentText()
        
        # 停止之前的定时器
        if self.topic_update_timer:
            self.topic_update_timer.stop()
            self.topic_update_timer = None
        
        # 重置当前显示的话题
        self.current_topic_display = None
        
        if content_type == "URDF Viewer" and selection == "Robot URDF":
            self.show_urdf_viewer()
        elif content_type == "Navigation Viewer":
            self.show_navigation_viewer(selection)
        elif content_type == "Other Topics" and selection:
            self.show_topic_content(selection)

    def show_topic_content(self, topic_name):
        """显示话题内容"""
        try:
            # 获取话题的消息类型
            topics = self.ros_node.get_topic_list()
            if topic_name not in topics:
                error_label = QLabel(f"Topic {topic_name} not found")
                error_label.setAlignment(Qt.AlignCenter)
                error_label.setStyleSheet("color: #ff6b6b; font-size: 14px; padding: 20px;")
                self.content_display.setWidget(error_label)
                return
            
            msg_type = topics[topic_name]
            
            # 清理之前的话题数据
            if topic_name in self.ros_node.topic_data:
                del self.ros_node.topic_data[topic_name]
            
            # 尝试订阅话题
            success = self.ros_node.subscribe_to_other_topic(topic_name, msg_type)
            
            if success:
                # 创建新的文本显示区域
                text_widget = QTextEdit()
                text_widget.setReadOnly(True)
                text_widget.setStyleSheet("""
                    QTextEdit {
                        background-color: #1e1e1e;
                        color: #ffffff;
                        border: 1px solid #5a5a5a;
                        border-radius: 4px;
                        padding: 8px;
                        font-family: 'Courier New', monospace;
                        font-size: 12px;
                    }
                """)
                
                # 清理旧的widget
                if topic_name in self.topic_text_displays:
                    old_widget = self.topic_text_displays[topic_name]
                    if old_widget:
                        old_widget.hide()
                        old_widget.deleteLater()
                
                self.topic_text_displays[topic_name] = text_widget
                
                # 设置当前显示的话题
                self.current_topic_display = topic_name
                
                # 先设置widget，然后再启动定时器
                self.content_display.setWidget(text_widget)
                
                # 创建新的定时器
                self.topic_update_timer = QTimer()
                self.topic_update_timer.timeout.connect(self.update_topic_content_display)
                self.topic_update_timer.start(1000)  # 每1秒更新一次
                
                # 显示初始提示信息
                text_widget.append(f"Waiting for data from topic: {topic_name} (type: {msg_type})")
                
            else:
                # 如果订阅失败，使用ros2 topic echo
                self.show_topic_with_echo(topic_name)
                
        except Exception as e:
            error_label = QLabel(f"Error showing topic content: {e}")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setStyleSheet("color: #ff6b6b; font-size: 14px; padding: 20px;")
            self.content_display.setWidget(error_label)

    def show_topic_with_echo(self, topic_name):
        """使用备用方法显示话题内容"""
        try:
            # 尝试不同的终端模拟器，优先使用xfce4-terminal
            terminal_commands = [
                ['xfce4-terminal', '--hold', '-e', f'ros2 topic echo {topic_name}'],
                ['x-terminal-emulator', '-e', 'bash', '-c', f'ros2 topic echo {topic_name}; read -p "Press enter to close..."'],
                ['xterm', '-hold', '-e', f'ros2 topic echo {topic_name}'],
                ['konsole', '--hold', '-e', 'bash', '-c', f'ros2 topic echo {topic_name}'],
                ['gnome-terminal', '--', 'bash', '-c', f'ros2 topic echo {topic_name}; read -p "Press enter to close..."']
            ]
            
            terminal_launched = False
            for cmd in terminal_commands:
                try:
                    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    terminal_launched = True
                    self.get_logger().info(f'Launched terminal with command: {" ".join(cmd[:2])}')
                    break
                except (FileNotFoundError, subprocess.SubprocessError, OSError) as e:
                    self.get_logger().debug(f'Failed to launch {cmd[0]}: {e}')
                    continue
            
            if terminal_launched:
                info_label = QLabel(f"Topic echo launched for: {topic_name}\nCheck the terminal window to view the topic data")
                info_label.setAlignment(Qt.AlignCenter)
                info_label.setWordWrap(True)
                info_label.setStyleSheet("font-size: 14px; padding: 20px; color: #ffffff;")
                self.content_display.setWidget(info_label)
            else:
                # 如果无法启动终端，显示一个简单的信息页面
                self.show_topic_info_page(topic_name)
                
        except Exception as e:
            self.show_topic_info_page(topic_name, str(e))

    def show_topic_info_page(self, topic_name, error_msg=None):
        """显示话题信息页面（当无法启动终端时的备用方案）"""
        try:
            # 获取话题信息
            result = subprocess.run(['ros2', 'topic', 'info', topic_name], 
                                   capture_output=True, text=True, timeout=5)
            
            info_text = f"Topic: {topic_name}\n"
            info_text += "=" * 50 + "\n\n"
            
            if error_msg:
                info_text += f"Terminal launch error: {error_msg}\n\n"
            
            if result.returncode == 0:
                info_text += "Topic Info:\n"
                info_text += result.stdout
            else:
                info_text += "Could not get topic info\n"
            
            info_text += "\n" + "=" * 50 + "\n"
            info_text += "Note: This topic type is not directly supported for real-time viewing.\n"
            info_text += "To view real-time data, please use one of these commands in a terminal:\n\n"
            info_text += f"ros2 topic echo {topic_name}\n"
            info_text += f"ros2 topic hz {topic_name}\n"
            info_text += f"ros2 topic bw {topic_name}\n\n"
            info_text += "Available terminals: xfce4-terminal, xterm, konsole, gnome-terminal"
            
            # 创建文本显示widget
            text_widget = QTextEdit()
            text_widget.setReadOnly(True)
            text_widget.setStyleSheet("""
                QTextEdit {
                    background-color: #1e1e1e;
                    color: #ffffff;
                    border: 1px solid #5a5a5a;
                    border-radius: 4px;
                    padding: 8px;
                    font-family: 'Courier New', monospace;
                    font-size: 12px;
                }
            """)
            text_widget.setText(info_text)
            
            self.content_display.setWidget(text_widget)
            
        except Exception as e:
            error_label = QLabel(f"Error showing topic info: {e}\n\nTopic: {topic_name}")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setStyleSheet("color: #ff6b6b; font-size: 14px; padding: 20px;")
            self.content_display.setWidget(error_label)

    def update_topic_content_display(self):
        """更新话题内容显示"""
        try:
            if (self.current_topic_display and 
                self.current_topic_display in self.topic_text_displays and
                self.current_topic_display in self.ros_node.topic_data):
                
                topic_name = self.current_topic_display
                text_widget = self.topic_text_displays[topic_name]
                
                # 检查widget是否仍然有效
                if text_widget and hasattr(text_widget, 'isVisible') and not text_widget.isHidden():
                    # 获取新数据
                    topic_data = self.ros_node.topic_data[topic_name]
                    
                    if topic_data:
                        # 清空并重新填充内容
                        text_widget.clear()
                        text_widget.append(f"Topic: {topic_name}\n" + "="*50)
                        for data_line in topic_data:
                            text_widget.append(data_line)
                        
                        # 滚动到底部
                        cursor = text_widget.textCursor()
                        cursor.movePosition(cursor.End)
                        text_widget.setTextCursor(cursor)
                        
        except RuntimeError as e:
            # QTextEdit被删除的情况
            if "wrapped C/C++ object" in str(e):
                # 停止定时器并清理
                if self.topic_update_timer:
                    self.topic_update_timer.stop()
                    self.topic_update_timer = None
                if self.current_topic_display in self.topic_text_displays:
                    del self.topic_text_displays[self.current_topic_display]
        except Exception as e:
            # 如果出现其他错误，停止定时器
            if self.topic_update_timer:
                self.topic_update_timer.stop()
                self.topic_update_timer = None
            print(f'Error updating topic display: {e}')


    def show_urdf_viewer(self):
        """显示URDF查看器"""
        try:
            # 获取sam_bot_description包的路径
            pkg_share_result = subprocess.run([
                'ros2', 'pkg', 'prefix', 'sam_bot_description'
            ], capture_output=True, text=True)
            
            if pkg_share_result.returncode == 0:
                pkg_path = pkg_share_result.stdout.strip()
                rviz_config_path = os.path.join(pkg_path, 'share', 'sam_bot_description', 'rviz', 'urdf_config.rviz')
                
                # 检查配置文件是否存在
                if os.path.exists(rviz_config_path):
                    # 启动RViz并加载URDF配置
                    subprocess.Popen(['rviz2', '-d', rviz_config_path])
                else:
                    # 如果配置文件不存在，直接启动RViz
                    subprocess.Popen(['rviz2'])
            else:
                # 如果找不到包，直接启动RViz
                subprocess.Popen(['rviz2'])
            
            info_label = QLabel("RViz URDF Viewer launched\nThe robot model should be visible if sam_bot_description is running\nAdd 'RobotModel' display if not visible")
            info_label.setAlignment(Qt.AlignCenter)
            info_label.setWordWrap(True)
            info_label.setStyleSheet("font-size: 14px; padding: 20px; color: #ffffff;")
            self.content_display.setWidget(info_label)
            
        except Exception as e:
            error_label = QLabel(f"Error launching URDF viewer: {e}")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setStyleSheet("color: #ff6b6b; font-size: 14px; padding: 20px;")
            self.content_display.setWidget(error_label)

    def show_navigation_viewer(self, viewer_type):
        """显示导航查看器"""
        try:
            # 根据类型启动不同的查看器
            if viewer_type == "Costmap":
                # 启动RViz显示导航相关信息
                nav2_config = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
                if os.path.exists(nav2_config):
                    subprocess.Popen(['rviz2', '-d', nav2_config])
                else:
                    subprocess.Popen(['rviz2'])
            elif viewer_type == "Robot Pose":
                subprocess.Popen(['rqt_plot', '/odom/pose/pose/position/x:y'])
            elif viewer_type == "Path":
                subprocess.Popen(['rqt_plot', '/plan'])
                
            info_label = QLabel(f"{viewer_type} viewer launched\nCheck the new window to view the data")
            info_label.setAlignment(Qt.AlignCenter)
            info_label.setWordWrap(True)
            info_label.setStyleSheet("font-size: 14px; padding: 20px; color: #ffffff;")
            self.content_display.setWidget(info_label)
            
        except Exception as e:
            error_label = QLabel(f"Error launching {viewer_type} viewer: {e}")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setStyleSheet("color: #ff6b6b; font-size: 14px; padding: 20px;")
            self.content_display.setWidget(error_label)

    def start_controller(self, controller_type):
        """启动控制器"""
        success = self.ros_node.start_controller(controller_type)
        if success:
            self.controller_status_label.setText(f"Active: {controller_type.title()} Controller")
            self.controller_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            QMessageBox.warning(self, "Error", f"Failed to start {controller_type} controller")

    def stop_controller(self):
        """停止当前控制器"""
        self.ros_node.stop_current_controller()
        self.controller_status_label.setText("No controller active")
        self.controller_status_label.setStyleSheet("color: #888;")

    def kill_node(self, node_name):
        """终止节点"""
        reply = QMessageBox.question(self, 'Confirm Kill Node', 
                                    f'Are you sure you want to kill node: {node_name}?',
                                    QMessageBox.Yes | QMessageBox.No, 
                                    QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            success = self.ros_node.kill_node(node_name)
            if success:
                QMessageBox.information(self, "Success", f"Node {node_name} killed successfully")
                self.refresh_nodes()
            else:
                QMessageBox.warning(self, "Error", f"Failed to kill node {node_name}")

    def send_velocity(self, linear_x, linear_y, angular_z):
        """发送速度命令"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.ros_node.cmd_vel_publisher.publish(msg)

    def closeEvent(self, event):
        """窗口关闭事件"""
        # 停止定时器
        if self.topic_update_timer:
            self.topic_update_timer.stop()
            self.topic_update_timer = None
        
        # 清理订阅器
        self.ros_node.cleanup_subscriptions()
        
        # 安全清理文本显示widgets
        for topic_name, widget in list(self.topic_text_displays.items()):
            try:
                if widget and hasattr(widget, 'deleteLater'):
                    widget.hide()
                    widget.deleteLater()
            except RuntimeError:
                # widget已经被删除
                pass
        self.topic_text_displays.clear()
        
        # 停止工作线程
        if hasattr(self, 'worker'):
            self.worker.stop()
        if hasattr(self, 'worker_thread'):
            self.worker_thread.quit()
            self.worker_thread.wait()
        
        # 停止所有控制器
        self.ros_node.stop_current_controller()
        
        event.accept()


def main(args=None):
    # 检查PyQt5是否可用
    if not PYQT5_AVAILABLE:
        print("Cannot start UI without PyQt5. Exiting.")
        return
    
    # 初始化ROS
    rclpy.init(args=args)
    
    # 创建ROS节点
    ros_node = RobotUINode()
    
    # 创建Qt应用
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Control UI")
    
    # 设置应用图标（如果有的话）
    # app.setWindowIcon(QIcon('path/to/icon.png'))
    
    # 创建主窗口
    main_window = MainWindow(ros_node)
    main_window.show()
    
    # 创建ROS执行器并在单独线程中运行
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    def ros_spin():
        executor.spin()
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    # 处理关闭信号
    def signal_handler(sig, frame):
        main_window.close()
        app.quit()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # 运行Qt应用
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        if hasattr(main_window, 'worker'):
            main_window.worker.stop()
        executor.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
