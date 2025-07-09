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
            try:
                self.content_selector.currentTextChanged.disconnect()
            except TypeError:
                # 如果没有连接的信号，会抛出TypeError，可以忽略
                pass
            
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
            elif current_type == "URDF Viewer":
                self.content_selector.addItems(["Robot URDF"])
            elif current_type == "Navigation Viewer":
                self.content_selector.addItems(["Costmap", "Path", "Robot Pose"])
            
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
            
        # 暂时断开信号连接
        try:
            self.content_selector.currentTextChanged.disconnect()
        except TypeError:
            # 如果没有连接的信号，会抛出TypeError，可以忽略
            pass
        
        # 清空选择器
        self.content_selector.clear()
        
        if content_type == "URDF Viewer":
            self.content_selector.addItems(["Robot URDF"])
        elif content_type == "Navigation Viewer":
            self.content_selector.addItems(["Costmap", "Path", "Robot Pose"])
        
        # 重新连接信号
        self.content_selector.currentTextChanged.connect(self.on_content_selected)
        
        # 如果有默认选项，触发选择事件
        if self.content_selector.count() > 0:
            # 手动触发选择事件
            current_text = self.content_selector.currentText()
            if current_text:
                self.on_content_selected(current_text)

    def on_content_selected(self, selection):
        """内容选择改变时的处理"""
        if self.is_updating_selectors or not selection:
            return
            
        content_type = self.content_type_combo.currentText()
        
        if content_type == "URDF Viewer" and selection == "Robot URDF":
            self.show_urdf_viewer()
        elif content_type == "Navigation Viewer":
            self.show_navigation_viewer(selection)
        elif content_type == "Other Topics" and selection:
            self.show_topic_with_terminal(selection)

    def show_topic_with_terminal(self, topic_name):
        """使用xfce4-terminal显示话题内容"""
        try:
            # 直接使用xfce4-terminal启动ros2 topic echo
            subprocess.Popen([
                'xfce4-terminal', 
                '--hold', 
                '--title', f'ROS2 Topic: {topic_name}',
                '-e', f'ros2 topic echo {topic_name}'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # 在UI中显示确认信息
            info_label = QLabel(f"Terminal opened for topic: {topic_name}\n\nShowing real-time data with:\nros2 topic echo {topic_name}\n\nThe terminal window should appear separately.")
            info_label.setAlignment(Qt.AlignCenter)
            info_label.setWordWrap(True)
            info_label.setStyleSheet("font-size: 14px; padding: 20px; color: #ffffff;")
            self.content_display.setWidget(info_label)
            
            self.ros_node.get_logger().info(f'Opened terminal for topic: {topic_name}')
            
        except Exception as e:
            # 如果启动失败，显示错误信息和备用方案
            error_label = QLabel(f"Failed to open terminal for topic: {topic_name}\n\nError: {str(e)}\n\nPlease manually run in terminal:\nros2 topic echo {topic_name}")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setWordWrap(True)
            error_label.setStyleSheet("color: #ff6b6b; font-size: 14px; padding: 20px;")
            self.content_display.setWidget(error_label)
            
            self.ros_node.get_logger().error(f'Failed to open terminal for {topic_name}: {e}')

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
